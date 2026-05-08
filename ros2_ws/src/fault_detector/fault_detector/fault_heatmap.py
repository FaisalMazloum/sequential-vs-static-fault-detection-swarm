import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from threading import Thread

NUM_ROBOTS = 10

class HeatmapNode(Node):
    def __init__(self):
        super().__init__('fault_heatmap')
        
        self.gbdt_matrix = np.full((NUM_ROBOTS, NUM_ROBOTS), -1.0)
        self.lstm_matrix = np.full((NUM_ROBOTS, NUM_ROBOTS), -1.0)
        self.gbdt_pred   = np.full((NUM_ROBOTS, NUM_ROBOTS), -1)
        self.lstm_pred   = np.full((NUM_ROBOTS, NUM_ROBOTS), -1)

        for i in range(NUM_ROBOTS):
            self.create_subscription(
                Float32MultiArray,
                f'/remora{i}/fault_status',
                lambda msg, rid=i: self.callback_gbdt(msg, rid), 10)
            self.create_subscription(
                Float32MultiArray,
                f'/remora{i}/fault_status_lstm',
                lambda msg, rid=i: self.callback_lstm(msg, rid), 10)

    def callback_gbdt(self, msg, observer_id):
        observed_id = int(msg.data[0])
        self.gbdt_matrix[observer_id, observed_id] = msg.data[1]
        self.gbdt_pred[observer_id, observed_id]   = int(msg.data[2])

    def callback_lstm(self, msg, observer_id):
        observed_id = int(msg.data[0])
        self.lstm_matrix[observer_id, observed_id] = msg.data[1]
        self.lstm_pred[observer_id, observed_id]   = int(msg.data[2])


def render_loop(node):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Fault Detection Heatmap  (row=observer, col=observed)', fontsize=13)
    plt.ion()
    plt.show()

    labels = [f'r{i}' for i in range(NUM_ROBOTS)]

    while rclpy.ok():
        for ax, matrix, pred, title in [
            (axes[0], node.gbdt_matrix, node.gbdt_pred, 'GBDT'),
            (axes[1], node.lstm_matrix, node.lstm_pred, 'LSTM'),
        ]:
            ax.clear()
            ax.set_title(title, fontsize=12)

            # build color matrix: grey=-1, green=0, red=1
            color_matrix = np.zeros((NUM_ROBOTS, NUM_ROBOTS, 3))
            for i in range(NUM_ROBOTS):
                for j in range(NUM_ROBOTS):
                    if i == j:
                        color_matrix[i, j] = [0.4, 0.4, 0.4]  # diagonal grey
                    elif pred[i, j] == -1:
                        color_matrix[i, j] = [0.25, 0.25, 0.25]  # no data
                    elif pred[i, j] == 1:
                        color_matrix[i, j] = [1.0, 0.0, 0.0]  # faulty red
                    else:
                        color_matrix[i, j] = [0.0, 0.8, 0.0]  # healthy green

            ax.imshow(color_matrix, aspect='equal')

            # probability text
            for i in range(NUM_ROBOTS):
                for j in range(NUM_ROBOTS):
                    if i == j:
                        ax.text(j, i, '—', ha='center', va='center',
                                fontsize=8, color='white')
                    elif matrix[i, j] >= 0:
                        ax.text(j, i, f'{matrix[i,j]:.2f}', ha='center',
                                va='center', fontsize=7, color='white')

            ax.set_xticks(range(NUM_ROBOTS))
            ax.set_yticks(range(NUM_ROBOTS))
            ax.set_xticklabels(labels, fontsize=8)
            ax.set_yticklabels(labels, fontsize=8)
            ax.set_xlabel('observed', fontsize=9)
            ax.set_ylabel('observer', fontsize=9)

        plt.tight_layout()
        plt.pause(1.0)  # refresh every 1 second


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapNode()

    # spin ROS2 in background thread
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # render in main thread (matplotlib requires main thread)
    render_loop(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()