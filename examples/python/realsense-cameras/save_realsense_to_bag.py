import pyrealsense2 as rs
import cv2
import numpy as np

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
# different types of frames
config = rs.config()
resolution = (640, 480)
# resolution = (1280, 720)

config.enable_stream(rs.stream.depth, *resolution, rs.format.z16, 30)
config.enable_stream(rs.stream.color, *resolution, rs.format.bgr8, 30)


# Enable recording to file (ROS bag file)
config.enable_record_to_file("output.bag")

output = cv2.VideoWriter("bag.avi", cv2.VideoWriter_fourcc(*"XVID"), 30, resolution)

# Start the pipeline with the configured streams
pipeline.start(config)

try:
    print("Recording started... Press Ctrl+C to stop.")

    # Main loop to capture frames
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Get the individual frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data())
        print(color_image.shape)
        output.write(color_image)

        if not depth_frame or not color_frame:
            continue

        cv2.imshow("Frame", color_image)

        # Press S on keyboard
        # to stop the process
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break

        # The frames are automatically saved to the bag file
        # No need to manually save them here, as they are handled by the pipeline

except KeyboardInterrupt:
    print("Recording stopped.")

finally:
    # Stop the pipeline
    pipeline.stop()
    output.release()
