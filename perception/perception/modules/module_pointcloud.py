from module_interface import ModuleInterface

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2

class ModulePointCloud(ModuleInterface):
    def __init__(self, config_file):
        # Initialize the RealSense pipeline
        self.pipeline, self.align, self.intrinsics = self.initialize_camera()

    def __call__(self, rgb_frame: np.ndarray, depth_frame: np.ndarray):
        # Generate point cloud from depth frame
        points, _ = self.generate_point_cloud(depth_frame)
        
        # Convert to Open3D point cloud format for further processing or visualization
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        return pc

    def draw(self, frame: np.ndarray) -> None:
        # Implement drawing or visualization (e.g., using Open3D visualization tools)
        o3d.visualization.draw_geometries([frame])

    def initialize_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        profile = pipeline.start(config)

        align = rs.align(rs.stream.color)
        intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        return pipeline, align, intrinsics

    def generate_point_cloud(self, depth_frame: np.ndarray):
        # Get depth intrinsics
        width = self.intrinsics.width
        height = self.intrinsics.height
        fx = self.intrinsics.fx
        fy = self.intrinsics.fy
        cx = self.intrinsics.ppx
        cy = self.intrinsics.ppy

        # Create a point cloud from the depth image
        points = []
        for v in range(height):
            for u in range(width):
                z = depth_frame[v, u] * self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
                if z == 0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        return np.array(points), depth_frame

if __name__ == "__main__":
    # Example usage
    module_pc = ModulePointCloud('/path/to/config_file')
    pipeline, align, _ = module_pc.initialize_camera()

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        point_cloud = module_pc(color_image, depth_image)
        module_pc.draw(point_cloud)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()
