class Publishers_manager:
    def __init__(self):
        self.publishers = {}

    def add_publisher(self, name, publisher):
        self.publishers[name] = publisher

    def get_publisher(self, name):
        return self.publishers[name]

    def get_publishers(self):
        return self.publishers

    def remove_publisher(self, name):
        del self.publishers[name]

    def remove_all_publishers(self):
        self.publishers.clear()
