class PipelineManager:
    def __init__(self, config):
        self.config = config
        self.pipeline = None

    def set_pipeline(self, pipeline):
        self.pipeline = pipeline

    def get_pipeline(self):
        return self.pipeline
