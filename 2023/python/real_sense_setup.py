import pyrealsense2 as rs

from stereoCamera import StereoCamera

pipe = rs.pipeline()
cfg = rs.config()  
profile = pipe.start()

profile1 = profile.get_stream(rs.stream.depth)
camera = StereoCamera(profile1)
cam_matrix = camera.get_intrinsic_camera_matrix()
coeffs = camera.get_coeffs()
intr = profile1.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

def get_frames():
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    color = np.asanyarray(color_frame.get_data())
    depth = np.asanyarray(depth_frame.get_data())

    frame = cv.cvtColor(color, cv.COLOR_BGR2RGB)
    gray_img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    return frame, depth_frame, gray_img