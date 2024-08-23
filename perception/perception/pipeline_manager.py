class PipelineManager:
    def __init__(self, config):
        self.config = config
        self.pipeline = None

    def set_pipeline(self, pipeline):
        self.pipeline = pipeline

    def get_pipeline(self):
        return self.pipeline


@dataclass
class PipelineWrapper:
    pipeline: Pipeline
    ros_publisher: Publisher
    name: str

    def process(self, rgb, rgbd):
        msg = self.pipeline(rgb, rgbd)
        self.ros_publisher.publish(msg)


class PipelineInterface:

    def init_segmentation_model(self):
        self.segmentation_model = ...

    def init_aruco_detector(self):
        self.aruco_detector = ...

    def call_segmentation_model(self, rgb: RGB) -> Mask:
        if self.segmentation_model is None:
            raise ValueError("Segmentation model not initialized")
            # or maybe: self.init_segmentation_model()
        return self.segmentation_model(rgb)

    def call_aruco_detector(self, rgb: RGB) -> Tuple:
        if self.aruco_detector is None:
            raise ValueError("Aruco detector not initialized")
            # or maybe: self.init_aruco_detector()
        return self.aruco_detector(rgb)

    def __call__(self, rgb: RGB, rgbd: RGBD):
        raise NotImplementedError


class ButtonsPipeline(Pipeline):
    def __init__(self, config):
        super().__init__(config)
        self.init_aruco_detector()

    def __call__(self, rgb: RGB, rgbd: RGBD):
        rvec, tvec = self.aruco_detector.process_rgb(rgb)
        return PoseMsg.create_message(*translation_rotation([0, 0, 0], rvec, tvec))
