from .buttons_pipeline import ButtonsPipeline
from .metal_bar_pipeline import MetalBarPipeline
from .ethernet_pipeline import EthernetPipeline
from .rocks_pipeline import RocksPipeline
from .surface_sample_pipeline import SurfaceSamplePipeline
from .bricks_pipeline import BricksPipeline
from .probes_pipeline import ProbesPipeline


class PipelineFactory:
    @staticmethod
    def create_pipeline(pipeline_type: str, config_file: str, node):
        """
        Factory method to create a pipeline instance based on the type.

        :param pipeline_type: The type of pipeline to create (e.g., 'buttons', 'metal_bar').
        :param config_file: Path to the configuration file.
        :param node: The ROS 2 node object, used to create publishers.
        :return: An instance of a pipeline subclass.
        """
        pipelines = {
            "buttons": ButtonsPipeline,
            "metal_bar": MetalBarPipeline,
            "ethernet": EthernetPipeline,
            "rocks": RocksPipeline,
            "surface_sample": SurfaceSamplePipeline,
            "bricks": BricksPipeline,
            "probes": ProbesPipeline,
        }

        if pipeline_type not in pipelines:
            raise ValueError(
                f"Unknown pipeline type: {pipeline_type} (valid types: {list(pipelines.keys())})"
            )

        pipeline_class = pipelines[pipeline_type]
        return pipeline_class(config_file, node)
