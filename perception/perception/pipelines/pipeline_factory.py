from .buttons_pipeline import ButtonsPipeline
from .metal_bar_pipeline import MetalBarPipeline
from .ethernet_pipeline import EthernetPipeline
from .rocks_pipeline import RocksPipeline
from .surface_sample_pipeline import SurfaceSamplePipeline
from .bricks_pipeline import BricksPipeline
from .probes_pipeline import ProbesPipeline
from .identity_pipeline import IdentityPipeline

from rclpy.node import Node
import os


class PipelineFactory:
    @staticmethod
    def create_pipeline(
        pipeline_type: str, node: Node, camera_matrix=None, dist_coeffs=None, camera_depth_scale=None
    ):
        """
        Factory method to create a pipeline instance based on the type.

        :param pipeline_type: The type of pipeline to create (e.g., 'buttonsA', 'metal_bar').
        :param node: The ROS 2 node object, used to create publishers.
        :param camera_matrix: (Optional) The camera matrix for calibration.
        :param dist_coeffs: (Optional) The distortion coefficients for calibration.
        :return: An instance of a pipeline subclass.
        """

        # Dictionary mapping pipeline types to their respective classes and configuration files
        # https://black.readthedocs.io/en/stable/usage_and_configuration/the_basics.html turn off black formatting
        # fmt: off
        configs_dir = f"{os.getcwd()}/src/perception/configs/"
        if not os.path.exists(configs_dir): # parce que matthias n'utilise pas DOCKER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAIN !!!!!!!!!!!!!!!!!!!!!!!!!!
            configs_dir = configs_dir.replace('src/', '')
        pipelines = {
            "buttonsA":       {"cls": ButtonsPipeline,       "config": f"{configs_dir}buttonsA.yaml"},
            "big_buttons":    {"cls": ButtonsPipeline,       "config": f"{configs_dir}big_buttons.yaml"},
            "electric_plugs": {"cls": ButtonsPipeline,       "config": f"{configs_dir}electric_plugs.yaml"},
            "metal_bar":      {"cls": MetalBarPipeline,      "config": f"{configs_dir}metal_bar.yaml"},
            "ethernet":       {"cls": EthernetPipeline,      "config": f"{configs_dir}ethernet.yaml"},
            "rocks":          {"cls": RocksPipeline,         "config": f"{configs_dir}rocks.yaml"},
            "surface_sample": {"cls": SurfaceSamplePipeline, "config": f"{configs_dir}surface_sample.yaml"},
            "bricks":         {"cls": BricksPipeline,        "config": f"{configs_dir}bricks.yaml"},
            "probes":         {"cls": ProbesPipeline,        "config": f"{configs_dir}probes.yaml"},
            "identity":       {"cls": IdentityPipeline,      "config": f"{configs_dir}identity.yaml"},
        }
        # fmt: on

        if pipeline_type not in pipelines:
            raise ValueError(f"Unknown pipeline type: {pipeline_type}")

        # Get the class and config file for the specified pipeline type
        pipeline_info = pipelines[pipeline_type]
        pipeline_class = pipeline_info["cls"]
        config_file = pipeline_info["config"]

        # Check if the pipeline requires calibration data and provide them if available
        if camera_matrix is not None and dist_coeffs is not None:
            return pipeline_class(config_file, node, camera_matrix, dist_coeffs)
        elif camera_matrix is not None and camera_depth_scale is not None:
            return pipeline_class(config_file, node, camera_matrix, camera_depth_scale)
        else:
            return pipeline_class(config_file, node)
