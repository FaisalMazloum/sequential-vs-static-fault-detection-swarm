import os
import time
import tracemalloc
import psutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import joblib
import pandas as pd
import warnings
warnings.filterwarnings('ignore', category=UserWarning)

FEATURE_COLS = [f'f{i}_{j}' for i in range(1, 5) for j in range(10)] + ['f5']

CONFIGS = {
    'deg': {
        'threshold':  0.8504,
        'model_path': '/home/faisal-mazloum/Desktop/Machine_Learning/PERSONAL-CODE/my_models/trained_models/degradation/deg_gbdt_model_test_final.joblib',
    },
    'if': {
        'threshold':  0.2843,
        'model_path': '/home/faisal-mazloum/Desktop/Machine_Learning/PERSONAL-CODE/my_models/trained_models/IF/IF_gbdt_model_test_final.joblib',
    },
}

WARMUP_CALLS  = 10
BENCH_SAMPLES = 1000


class GBDT_FaultDetectorMetricsNode(Node):
    def __init__(self):
        super().__init__('gbdt_fault_detector_metrics')

        self.declare_parameter('fault_type', 'deg')
        fault_type = self.get_parameter('fault_type').get_parameter_value().string_value

        if fault_type not in CONFIGS:
            self.get_logger().error(f'Invalid fault_type: {fault_type}. Choose deg or if.')
            raise ValueError(f'Invalid fault_type: {fault_type}')

        cfg = CONFIGS[fault_type]
        self.get_logger().info(f'fault_type: {fault_type}')

        self.threshold       = cfg['threshold']
        self.fault_publishers = {}
        self.num_robots      = 10

        _proc = psutil.Process(os.getpid())
        _rss_baseline  = _proc.memory_info().rss
        self.model = joblib.load(cfg['model_path'])
        _rss_post_load = _proc.memory_info().rss
        self.get_logger().info(f'Model loaded from {cfg["model_path"]}')

        baseline_mb   = _rss_baseline  / (1024.0 * 1024.0)
        model_only_mb = (_rss_post_load - _rss_baseline) / (1024.0 * 1024.0)
        total_mb      = _rss_post_load  / (1024.0 * 1024.0)

        self._log_model_size(cfg['model_path'])
        self.get_logger().info(f'[METRIC] static RAM baseline        = {baseline_mb:.2f} MB')
        self.get_logger().info(f'[METRIC] static RAM model-only      = {model_only_mb:.2f} MB')
        self.get_logger().info(f'[METRIC] static RAM total (process) = {total_mb:.2f} MB')
        self._benchmark_latency_and_ram()

        for i in range(self.num_robots):
            self.create_subscription(
                Float32MultiArray,
                f'/remora{i}/behavioral_features',
                lambda msg, rid=i: self.callback(msg, rid),
                10)
            self.fault_publishers[i] = self.create_publisher(
                Float32MultiArray,
                f'/remora{i}/fault_status',
                10)

        self.get_logger().info('GBDT Fault detector metrics node started')

    # ------------------------------------------------------------------
    # Startup metrics
    # ------------------------------------------------------------------

    def _log_model_size(self, path: str) -> None:
        size_kb = os.path.getsize(path) / 1024.0
        self.get_logger().info(f'[METRIC] model_size = {size_kb:.2f} KB')

    def _make_dummy_input(self) -> pd.DataFrame:
        return pd.DataFrame(
            np.zeros((1, len(FEATURE_COLS))), columns=FEATURE_COLS
        )

    def _benchmark_latency_and_ram(self) -> None:
        dummy = self._make_dummy_input()

        # warm-up — excluded from stats
        for _ in range(WARMUP_CALLS):
            self.model.predict_proba(dummy)

        latencies_ns = np.empty(BENCH_SAMPLES, dtype=np.float64)
        peak_ram_bytes = np.empty(BENCH_SAMPLES, dtype=np.float64)

        for k in range(BENCH_SAMPLES):
            tracemalloc.start()
            t0 = time.process_time_ns()
            self.model.predict_proba(dummy)
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
            f'[METRIC] predict_proba latency over {BENCH_SAMPLES} samples: '
            f'mean={mean_us:.2f} µs  std={std_us:.2f} µs'
        )
        self.get_logger().info(
            f'[METRIC] predict_proba peak RAM over {BENCH_SAMPLES} samples: '
            f'mean={mean_ram_kb:.2f} KB  std={std_ram_kb:.2f} KB'
        )

    # ------------------------------------------------------------------
    # Runtime callback
    # ------------------------------------------------------------------

    def callback(self, msg, observer_id):
        data = np.array(msg.data)
        observed_id = int(data[0])
        features = pd.DataFrame(data[1:].reshape(1, -1), columns=FEATURE_COLS)

        prob  = float(self.model.predict_proba(features)[0][1])
        y_hat = 1 if prob >= self.threshold else 0

        out_msg = Float32MultiArray()
        out_msg.data = [float(observed_id), prob, float(y_hat)]
        self.fault_publishers[observer_id].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GBDT_FaultDetectorMetricsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
