"""A ROS Node to plot the current best control sequence."""

from threading import Thread

import matplotlib.pyplot as plt

from mesh_mppi_msgs.msg import ControlSequenceStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class PlotControlSequenceNode(Node):
    """The node."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('control_sequence_plot_node')

        self.__qos = QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.__sub = self.create_subscription(
            ControlSequenceStamped,
            '/move_base_flex/optimal_sequence',
            self.__on_sequence,
            self.__qos
        )

        self.__figure = plt.figure()
        self.__axes = self.__figure.add_subplot()
        self.__figure.suptitle('Optimal Control Sequence')
        self.__figure.show()

    def __on_sequence(self, sequence: ControlSequenceStamped):
        data = np.array(sequence.sequence.data)
        data = np.reshape(data, (sequence.sequence.num_timesteps, sequence.sequence.num_signals))

        self.__axes.clear()
        # Plot all control signals
        x = np.arange(sequence.sequence.num_timesteps)
        for i in range(sequence.sequence.num_signals):
            self.__axes.plot(x, data[:, i], label=str(i))

        self.__axes.set_ylim(bottom=0.0, top=1.0)
        self.__axes.legend()
        self.__figure.canvas.draw_idle()


def main():
    """Run the node."""
    rclpy.init()
    node = PlotControlSequenceNode()

    thread = Thread(target=rclpy.spin, kwargs={'node': node})
    thread.start()

    plt.show()
    rclpy.shutdown()
