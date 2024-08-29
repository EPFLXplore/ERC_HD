import pyrealsense2 as rs
import numpy as np
from module_rocks import ModuleRocks
def main():
    # Initialize the RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    
    align = rs.align(rs.stream.color)

    # Get depth scale and intrinsics
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    # Initialize the ModuleRocks instance
    module_instance = ModuleRocks()
    module_instance.set_camera_parameters(depth_scale, intrinsics)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert images to numpy arrays
            rgb_frame = np.asanyarray(color_frame.get_data())
            depth_frame = np.asanyarray(depth_frame.get_data())

            # Call the module's __call__ method to process the frames
            processed_frame, results = module_instance(rgb_frame, depth_frame)

            # Print all the output values here
            for result in results:
                center = result["center"]
                print(f"Rock at ({center[0]}, {center[1]}): Max Diameter = {result['max_dimension_cm']:.2f}cm, Min Diameter = {result['min_dimension_cm']:.2f}cm")
                print(f"Depth at Rock Surface: {result['depth_surface']:.2f} meters")
                print(f"Depth at Rock Center: {result['rock_center_depth']:.2f} meters")
                print(f"Rock Center 3D Coordinates: {result['rock_center_coordinates']}")
                print(f"Quaternion to align Z-axis: {result['quaternion']}")

            # Display the resulting frame
            module_instance.draw(processed_frame)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Stop the camera pipeline
        pipeline.stop()

if __name__ == "__main__":
    main()
