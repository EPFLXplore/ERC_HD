import yaml
import rclpy
from rclpy.node import Node
from numpy import ndarray
from abc import ABC, abstractmethod


class PipelineInterface(ABC):
    def __init__(self, config_file: str, node: Node, draw_results: bool = True):
        """
        Constructor that initializes the pipeline with a configuration file and a ROS 2 node.

        :param config_file: Path to the configuration file.
        :param node: The ROS 2 node object, used to create publishers.
        :param draw_results: Whether to draw the results on the frame Default is True.
        """
        # Load configuration from the config file
        self._logger = node.get_logger()
        self.draw_results = draw_results
        with open(config_file, "r") as file:
            self.config = yaml.safe_load(file)

        # Use the node to create ROS 2 publishers
        self._initialize_publishers(node)

        # Initialize any other necessary components based on the config
        self._initialize_pipeline()

    @abstractmethod
    def _initialize_publishers(self, node: Node):
        """
        Initializes ROS 2 publishers using the node. This method should be overridden by subclasses
        to create specific publishers needed by the pipeline.

        :param node: The ROS 2 node object, used to create publishers.
        """
        raise NotImplementedError(
            "The _initialize_publishers method should be overridden by subclasses"
        )

    @abstractmethod
    def _initialize_pipeline(self):
        """
        Initializes the pipeline components based on the configuration.
        This method can be overridden in subclasses to initialize specific components.
        """
        raise NotImplementedError("This method should be overridden by subclasses")

    @abstractmethod
    def run_rgbd(self, rgb_image: ndarray, depth_image: ndarray) -> None:
        """
        Process the RGB and depth images.

        :param rgb_image: The RGB image input.
        :param depth_image: The depth image input.
        :return: The processed result (this can vary depending on the implementation).
        """
        raise NotImplementedError(
            "The run_rgbd method must be implemented by the subclass."
        )

    @abstractmethod
    def draw(self, frame: ndarray):
        """
        Draws the results of the pipeline on the given frame. but does not display the results another node displays the modified image
        does not draw itself but calls the draw method of its modules

        :param frame: The frame on which the results will be drawn.
        :return: The frame with the drawn results.
        """
        raise NotImplementedError(
            "The draw method must be implemented by the subclass."
        )

    @abstractmethod
    def _publish(self):
        """"
        called from the run_rgbd method publishes the results.
        """
        raise NotImplementedError(
            "The draw method must be implemented by the subclass."
        )
    
    @property
    @abstractmethod
    def name(self):
        """Each pipeline step must have a 'name' attribute"""
        pass