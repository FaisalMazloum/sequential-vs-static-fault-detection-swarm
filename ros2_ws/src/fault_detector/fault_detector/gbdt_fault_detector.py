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


class GBDT_FaultDetectorNode(Node):
    def __init__(self):
        super().__init__('gbdt_fault_detector')

        self.declare_parameter('fault_type', 'deg')
        fault_type = self.get_parameter('fault_type').get_parameter_value().string_value

        if fault_type not in CONFIGS:
            self.get_logger().error(f'Invalid fault_type: {fault_type}. Choose deg or if.')
            raise ValueError(f'Invalid fault_type: {fault_type}')

        cfg = CONFIGS[fault_type]
        self.get_logger().info(f'fault_type: {fault_type}')

        self.threshold    = cfg['threshold']
        self.fault_publishers = {}
        self.num_robots   = 10

        self.model = joblib.load(cfg['model_path'])
        self.get_logger().info(f'Model loaded from {cfg["model_path"]}')

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

        self.get_logger().info('GBDT Fault detector node started')

    def callback(self, msg, observer_id):
        data = np.array(msg.data)
        observed_id = int(data[0])
        features = pd.DataFrame(data[1:].reshape(1, -1), columns=FEATURE_COLS)

        prob  = float(self.model.predict_proba(features)[0][1])
        y_hat = 1 if prob >= self.threshold else 0

        status = 'FAULTY' if y_hat == 1 else 'healthy'
        self.get_logger().info(
            f'observer={observer_id} observed={observed_id} → {status} (p={prob:.3f})')

        out_msg = Float32MultiArray()
        out_msg.data = [float(observed_id), prob, float(y_hat)]
        self.fault_publishers[observer_id].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GBDT_FaultDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()