import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import joblib
from collections import defaultdict, deque
import warnings
warnings.filterwarnings('ignore')
import sys
sys.path.insert(0, '/home/faisal-mazloum/Desktop/Machine_Learning/ml_venv/lib/python3.12/site-packages')
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
import tensorflow as tf
import threading
import queue
import time
import tracemalloc
import psutil

CONFIGS = {
    'deg': {
        'threshold':       0.6973,
        'window_size':     55,
        'feature_indices': [10, 20, 30, 40, 41],
        'scaler_path':     '/home/faisal-mazloum/Desktop/Machine_Learning/PERSONAL-CODE/my_models/trained_models/degradation/deg_scaler_tuned_classifier_final.pkl',
        'model_path':      '/home/faisal-mazloum/Desktop/Machine_Learning/PERSONAL-CODE/my_models/trained_models/degradation/deg_best_tuned_lstm_classifier_model_final.keras',
    },
    'if': {
        'threshold':       0.5856,
        'window_size':     52,
        'feature_indices': list(range(1, 42)),
        'scaler_path':     '/home/faisal-mazloum/Desktop/Machine_Learning/PERSONAL-CODE/my_models/trained_models/IF/IF_scaler_tuned_classifier_final.pkl',
        'model_path':      '/home/faisal-mazloum/Desktop/Machine_Learning/PERSONAL-CODE/my_models/trained_models/IF/IF_best_tuned_lstm_classifier_model_final.keras',
    },
}

WARMUP_CALLS  = 100
BENCH_SAMPLES = 1000


class LSTM_FaultDetectorMetricsNode(Node):
    def __init__(self):
        super().__init__('lstm_fault_detector_metrics')

        self.declare_parameter('fault_type', 'deg')
        fault_type = self.get_parameter('fault_type').get_parameter_value().string_value

        if fault_type not in CONFIGS:
            self.get_logger().error(f'Invalid fault_type: {fault_type}. Choose deg or if.')
            raise ValueError(f'Invalid fault_type: {fault_type}')

        cfg = CONFIGS[fault_type]
        self.get_logger().info(f'fault_type: {fault_type}')

        self.threshold       = cfg['threshold']
        self.window_size     = cfg['window_size']
        self.feature_indices = cfg['feature_indices']
        self.fault_publishers = {}
        self.num_robots      = 10
        self.buffers         = defaultdict(lambda: deque(maxlen=self.window_size))
        self.infer_queue     = queue.Queue()

        _proc         = psutil.Process(os.getpid())
        _rss_baseline = _proc.memory_info().rss

        # load scaler
        self.scaler = joblib.load(cfg['scaler_path'])
        self.get_logger().info(f'Scaler loaded from {cfg["scaler_path"]}')

        # load model
        self.model = tf.keras.models.load_model(cfg['model_path'])
        self.get_logger().info(f'Model loaded from {cfg["model_path"]}')

        _rss_post_load = _proc.memory_info().rss

        # ------------------------------------------------------------------
        # Startup metrics
        # ------------------------------------------------------------------
        self._log_disk_sizes(cfg['scaler_path'], cfg['model_path'])
        self._log_static_ram(_rss_baseline, _rss_post_load)
        self._benchmark_latency_and_ram(cfg['window_size'], len(cfg['feature_indices']))

        # start inference worker thread
        self.infer_thread = threading.Thread(
            target=self.inference_worker, daemon=True
        )
        self.infer_thread.start()
        self.get_logger().info('Inference worker thread started')

        # subscribe + publish per robot
        for i in range(self.num_robots):
            self.create_subscription(
                Float32MultiArray,
                f'/remora{i}/behavioral_features',
                lambda msg, rid=i: self.callback(msg, rid),
                10,
            )
            self.fault_publishers[i] = self.create_publisher(
                Float32MultiArray,
                f'/remora{i}/fault_status_lstm',
                10,
            )

        self.get_logger().info('LSTM Fault detector metrics node started')

    # ------------------------------------------------------------------
    # Startup metrics
    # ------------------------------------------------------------------

    def _log_disk_sizes(self, scaler_path: str, model_path: str) -> None:
        scaler_kb = os.path.getsize(scaler_path) / 1024.0
        model_kb  = os.path.getsize(model_path)  / 1024.0
        self.get_logger().info(f'[METRIC] scaler disk size = {scaler_kb:.2f} KB')
        self.get_logger().info(f'[METRIC] model disk size  = {model_kb:.2f} KB')

    def _log_static_ram(self, rss_before: int, rss_after: int) -> None:
        baseline_mb   = rss_before / (1024.0 * 1024.0)
        model_only_mb = (rss_after - rss_before) / (1024.0 * 1024.0)
        total_mb      = rss_after  / (1024.0 * 1024.0)
        self.get_logger().info(f'[METRIC] static RAM baseline        = {baseline_mb:.2f} MB')
        self.get_logger().info(f'[METRIC] static RAM model-only      = {model_only_mb:.2f} MB')
        self.get_logger().info(f'[METRIC] static RAM total (process) = {total_mb:.2f} MB')

    def _benchmark_latency_and_ram(self, window_size: int, n_features: int) -> None:
        for batch_size in (1, 9):
            dummy = np.zeros((batch_size, window_size, n_features), dtype=np.float32)

            with tf.device('/CPU:0'):
                for _ in range(WARMUP_CALLS):
                    self.model(dummy, training=False)

                latencies_ns   = np.empty(BENCH_SAMPLES, dtype=np.float64)
                peak_ram_bytes = np.empty(BENCH_SAMPLES, dtype=np.float64)

                for k in range(BENCH_SAMPLES):
                    tracemalloc.start()
                    t0 = time.process_time_ns()
                    self.model(dummy, training=False)
                    t1 = time.process_time_ns()
                    _, peak = tracemalloc.get_traced_memory()
                    tracemalloc.stop()

                    latencies_ns[k]   = t1 - t0
                    peak_ram_bytes[k] = peak

            latencies_us = latencies_ns / 1_000.0
            mean_us = float(np.mean(latencies_us))
            std_us  = float(np.std(latencies_us))

            peak_ram_kb_arr = peak_ram_bytes / 1024.0
            mean_ram_kb = float(np.mean(peak_ram_kb_arr))
            std_ram_kb  = float(np.std(peak_ram_kb_arr))

            self.get_logger().info(
                f'[METRIC] inference latency batch_size={batch_size} over {BENCH_SAMPLES} samples (CPU): '
                f'mean={mean_us:.2f} µs  std={std_us:.2f} µs'
            )
            self.get_logger().info(
                f'[METRIC] inference peak RAM batch_size={batch_size} over {BENCH_SAMPLES} samples: '
                f'mean={mean_ram_kb:.2f} KB  std={std_ram_kb:.2f} KB'
            )

    # ------------------------------------------------------------------
    # Runtime — unchanged from original
    # ------------------------------------------------------------------

    def callback(self, msg, observer_id):
        data = np.array(msg.data)
        observed_id = int(data[0])

        features = data[self.feature_indices].reshape(1, -1)
        features_scaled = self.scaler.transform(features)[0]

        key = (observer_id, observed_id)
        self.buffers[key].append(features_scaled)

        if len(self.buffers[key]) < self.window_size:
            return

        X = np.array(self.buffers[key])[np.newaxis]
        self.infer_queue.put((X, observer_id, observed_id))

    def inference_worker(self):
        while True:
            X_list = []
            meta   = []

            X, observer_id, observed_id = self.infer_queue.get()
            X_list.append(X)
            meta.append((observer_id, observed_id))

            try:
                while True:
                    X2, o2, ob2 = self.infer_queue.get_nowait()
                    X_list.append(X2)
                    meta.append((o2, ob2))
            except queue.Empty:
                pass

            X_batch = np.concatenate(X_list, axis=0)
            probs   = self.model(X_batch, training=False).numpy().reshape(-1)

            for (observer_id, observed_id), prob in zip(meta, probs):
                y_hat  = 1 if prob >= self.threshold else 0

                out_msg = Float32MultiArray()
                out_msg.data = [float(observed_id), float(prob), float(y_hat)]
                self.fault_publishers[observer_id].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LSTM_FaultDetectorMetricsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
