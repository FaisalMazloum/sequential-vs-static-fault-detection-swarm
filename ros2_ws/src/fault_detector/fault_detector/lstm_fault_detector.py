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
import tensorflow as tf
import threading
import queue

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


class LSTM_FaultDetectorNode(Node):
    def __init__(self):
        super().__init__('lstm_fault_detector')

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

        # load scaler
        self.scaler = joblib.load(cfg['scaler_path'])
        self.get_logger().info(f'Scaler loaded from {cfg["scaler_path"]}')

        # load model
        self.model = tf.keras.models.load_model(cfg['model_path'])
        self.get_logger().info(f'Model loaded from {cfg["model_path"]}')

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

        self.get_logger().info('LSTM Fault detector node started')

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
                status = 'FAULTY' if y_hat == 1 else 'healthy'

                self.get_logger().info(
                    f'observer={observer_id} observed={observed_id} '
                    f'→ {status} (p={prob:.3f})'
                )

                out_msg = Float32MultiArray()
                out_msg.data = [float(observed_id), float(prob), float(y_hat)]
                self.fault_publishers[observer_id].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LSTM_FaultDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()