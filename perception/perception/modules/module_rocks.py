from .module_interface import ModuleInterface
import pyrealsense2 as rs
import numpy as np
import cv2
#from ultralytics import YOLO
import torch
from scipy.spatial.transform import Rotation as R  # Use scipy for quaternion operations
import rclpy

'''
PURPOSE OF THE MODULE
take rgb_frame, color_frame
return: a list of rocks with their positions (quaternions), max dimensions


'''
# Check if CUDA is available
cuda_available = torch.cuda.is_available()
device = torch.device("cuda" if cuda_available else "cpu")

class ModuleRocks(ModuleInterface):
    def __init__(self, camera_matrix, camera_depth_scale):
        # Placeholder for the current processed frame
        self.current_frame = None
        self.detected_objects = None

        # Depth scale and intrinsics must be initialized before processing frames
        self.depth_scale = None
        self.intrinsics = None

        self.depth_scale = camera_depth_scale

        intrin = rs.intrinsics()
        intrin.width = 640  # Example width, adjust as needed
        intrin.height = 480  # Example height, adjust as needed
        intrin.ppx = camera_matrix[0][2]
        intrin.ppy = camera_matrix[1][2]
        intrin.fx = camera_matrix[0][0]
        intrin.fy = camera_matrix[1][1]
        intrin.model = rs.distortion.none  # Example, adjust if using distortion
        intrin.coeffs = [0, 0, 0, 0, 0]  # Example, adjust if using distortion coefficients

        self.intrinsics = intrin


    def __call__(self, rgb_frame: np.ndarray, depth_frame: np.ndarray, segmentation_data):
        print("Called Rocks Module method")

        # Process the frame with the model
        self.current_frame    = rgb_frame
        self.detected_objects =  segmentation_data
        
        # Draw bounding boxes and masks
        self.current_frame, self.detected_objects = self.draw_bounding_boxes(self.current_frame, self.detected_objects)
        self.detected_objects = self.draw_masks_and_contours(self.detected_objects)
        
        results = []

        closest_index = 65535 # MAX (-1 not within bounds)
        closest_distance = float('inf')
        frame_center = (self.intrinsics.width, self.intrinsics.height)

        # Calculate axes, dimensions, and quasi-center for each detected object
        for i, obj in enumerate(self.detected_objects):
            # for obj in objs:
                contour = obj['contour']
                if contour is not None:
                    center = obj['center']
                    bounding_box = obj['bounding_box']
                    depth_surface = depth_frame[center[1], center[0]] * self.depth_scale  # Depth at the rock surface

                    max_dist, min_dist, max_pts, min_pts = self.calculate_axes(contour, center)
                    self.current_frame = self.draw_axes(self.current_frame, max_pts, min_pts)

                    self.current_frame, max_dim_cm, min_dim_cm = self.calculate_real_dimensions(
                        self.current_frame, obj, max_dist, min_dist, depth_surface, self.intrinsics)

                    # Calculate the quasi-center of the rock
                    rock_center_coordinates, rock_center_depth = self.calculate_rock_center(center, bounding_box, depth_frame, self.intrinsics, depth_surface)

                    # Check if the object's max dimension is greater than or equal to the minimum required
                    if max_dim_cm >= 15:
                        object_center = center
                        distance_to_center = np.linalg.norm(np.array(object_center) - np.array(frame_center))

                        # Update the closest object if this one is nearer
                        if distance_to_center < closest_distance:
                            closest_distance = distance_to_center
                            closest_index = i

                    # Calculate the minimal axis vector in 3D
                    # min_axis_vector = self.calculate_minimal_axis_vector(min_pts, rock_center_coordinates, depth_frame)

                    # Compute the quaternion for aligning the Z-axis with the minimal axis vector
                    # quaternion = self.calculate_quaternion_to_align_z(min_axis_vector)
                    angle = float(0)  #TODO

                    # Append the details to results
                    results.append({
                        "center": center,
                        "max_dimension_cm": max_dim_cm,
                        "min_dimension_cm": min_dim_cm,
                        "depth_surface": depth_surface, # from camera to rock
                        "rock_center_depth": rock_center_depth, # from camera to center of rock 
                        "rock_center_coordinates": rock_center_coordinates,
                        "angle": angle
                    })

        # target_idx = self.find_closest_result_to_center(results, self.intrinsics.width, self.intrinsics.height)

        # Return the results and target 
        return results, closest_index


    def find_closest_result_to_center(results, frame_width, frame_height, min_dimension_cm=15):
        # Calculate the center of the frame
        # frame_center = (frame_width // 2, frame_height // 2)
        frame_center = (320, 240)
        
        closest_index = -1
        closest_distance = float('inf')

        for i, result in enumerate(results):
            max_dimension_cm = result.get("max_dimension_cm", 0)

            # Check if the object's max dimension is greater than or equal to the minimum required
            if max_dimension_cm >= min_dimension_cm:
                object_center = result.get("center", (0, 0))
                distance_to_center = np.linalg.norm(np.array(object_center) - np.array(frame_center))

                # Update the closest object if this one is nearer
                if distance_to_center < closest_distance:
                    closest_distance = distance_to_center
                    closest_index = i

        return closest_index


    def calculate_minimal_axis_vector(self, min_pts, rock_center_coordinates, depth_frame: np.ndarray):
        # Deproject the minimal axis points to 3D
        if self.intrinsics is None:
            print("Error: 'self.intrinsics' is None.")
        else:
            print("self.intrinsics is correctly initialized.")

        p1_3d = np.array(rs.rs2_deproject_pixel_to_point(self.intrinsics, min_pts[0], depth_frame[min_pts[0][1], min_pts[0][0]] * self.depth_scale))
        p2_3d = np.array(rs.rs2_deproject_pixel_to_point(self.intrinsics, min_pts[1], depth_frame[min_pts[1][1], min_pts[1][0]] * self.depth_scale))
            
        # p1_3d = np.array(rs.rs2_deproject_pixel_to_point(self.intrinsics, min_pts[0], 2 * self.depth_scale))
        # p2_3d = np.array(rs.rs2_deproject_pixel_to_point(self.intrinsics, min_pts[1], 2 * self.depth_scale))

        # Calculate the minimal axis vector from the rock center
        min_axis_vector = p1_3d - p2_3d
        min_axis_vector /= np.linalg.norm(min_axis_vector)  # Normalize the vector

        return min_axis_vector

    def calculate_quaternion_to_align_z(self, min_axis_vector):
        # Calculate the rotation vector that aligns the Z-axis with the minimal axis vector
        z_axis = np.array([0, 0, 1])
        rotation_vector = np.cross(z_axis, min_axis_vector)
        sin_angle = np.linalg.norm(rotation_vector)
        cos_angle = np.dot(z_axis, min_axis_vector)
        rotation_vector = rotation_vector / sin_angle

        # Create the quaternion from the rotation vector and angle
        angle = np.arctan2(sin_angle, cos_angle)
        quaternion = R.from_rotvec(rotation_vector * angle).as_quat()

        return quaternion

    def draw(self, frame: np.ndarray) -> None:
        cv2.imshow("Rock Segmentation", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()


    def draw_bounding_boxes(self, image, results):
        detected_objects = []
        for result in results:  # segment in segments 
            if result.masks is not None:
                for mask, box in zip(result.masks, result.boxes):

                    mask_np = np.array(mask.mask_pixel, dtype=np.float32)
                    # Create the binary mask
                    binary_mask = (mask_np > 0.5).astype(np.uint8)
                    # binary_mask = (mask.mask_pixel.astype(np.uint8) > 0.5).astype(np.uint8)

                    # Extract bounding box data
                    x1 = int(box.x1)  # Top-left x coordinate
                    y1 = int(box.y1)  # Top-left y coordinate
                    x2 = int(box.x2)  # Bottom-right x coordinate
                    y2 = int(box.y2)  # Bottom-right y coordinate
                    confidence = box.confidence  # Confidence score of the detection
                    class_id = box.class_id  # Class ID of the detected object
                    track_id = box.track_id  # Tracking ID

                    # Draw bounding box on the image
                    # cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue color for bounding box

                    # Draw the center of the bounding box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    cv2.circle(image, (center_x, center_y), 5, (255, 255, 0), -1)  # Cyan color for center

                    detected_objects.append({
                        'mask': binary_mask, #binary_mask
                        'contour': None,
                        'center': (center_x, center_y),
                        'bounding_box': (x1, y1, x2, y2)
                    })
        return image, detected_objects


    def draw_masks_and_contours(self, segmentation_data):
        for obj in segmentation_data:
            binary_mask = obj['mask']

            # Find contours
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            if contours:
                contour = max(contours, key=cv2.contourArea)
                obj['contour'] = contour

        return segmentation_data


    def calculate_axes(self, contour, center):
        max_distance = 0
        min_distance = float('inf')
        max_points = (None, None)
        min_points = (None, None)

        for i in range(len(contour)):
            for j in range(i + 1, len(contour)):
                p1 = contour[i][0]  # Accessing the point coordinates (x, y)
                p2 = contour[j][0]  # Accessing the point coordinates (x, y)

                distance_to_center = cv2.pointPolygonTest(np.array([p1, p2]), center, True)
                if abs(distance_to_center) < 1.0:
                    distance = np.linalg.norm(p1 - p2)
                    if distance > max_distance:
                        max_distance = distance
                        max_points = (tuple(p1), tuple(p2))
                    if distance < min_distance:
                        min_distance = distance
                        min_points = (tuple(p1), tuple(p2))

        return max_distance, min_distance, max_points, min_points

    def draw_axes(self, image, max_points, min_points):
        if all(max_points):
            cv2.line(image, max_points[0], max_points[1], (0, 255, 255), 2)  # Yellow color for max axis
        if all(min_points):
            cv2.line(image, min_points[0], min_points[1], (255, 0, 255), 2)  # Magenta color for min axis
        return image

    def calculate_real_dimensions(self, image, obj, max_distance, min_distance, depth_value, intrinsics):
        if depth_value == 0:
            return image, 0, 0

        fx = intrinsics.fx
        fy = intrinsics.fy
        # fx = intrinsics[0][0]
        # fy = intrinsics[1][1]

        max_dimension = (max_distance * depth_value) / fx
        min_dimension = (min_distance * depth_value) / fy

        max_dimension_cm = max_dimension * 100
        min_dimension_cm = min_dimension * 100

        if max_dimension_cm > 15:
            mask = obj['mask']
            green_mask = np.zeros_like(image, dtype=np.uint8)
            green_mask[:, :] = (0, 255, 0)  # Green color for the mask
            masked_image = cv2.bitwise_and(green_mask, green_mask, mask=mask)
            image = cv2.addWeighted(image, 1, masked_image, 0.5, 0)

        return image, max_dimension_cm, min_dimension_cm
    
    def calculate_rock_center(self, center_pixel, bounding_box, depth_frame, intrinsics, depth_surface):
        """
        Calculate the quasi-center of a detected rock.
        
        Parameters:
        center_pixel (tuple): (x, y) coordinates of the center pixel in the bounding box.
        bounding_box (tuple): (x1, y1, x2, y2) coordinates of the bounding box corners.
        depth_frame (np.ndarray): Depth frame corresponding to the RGB image.
        intrinsics (rs.intrinsics): Camera intrinsics used for deprojection.

        Returns:
        np.ndarray: The 3D point in space representing the quasi-center of the rock.
        float: The distance from the camera to the quasi-center.
        """
        x1, y1, x2, y2 = bounding_box
        rclpy
        print(f'bb corners: x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}')
        print(f'depth frame shape {depth_frame.shape}')

        # Calculate depth values at the four corners of the bounding box (distance to ground)
        depth_corners = [
            depth_frame[y1, x1] * self.depth_scale,  # top-left corner
            depth_frame[y1, x2-1] * self.depth_scale,  # top-right corner
            depth_frame[y2-1, x1] * self.depth_scale,  # bottom-left corner
            depth_frame[y2-1, x2-1] * self.depth_scale   # bottom-right corner
        ]
        depth_ground = np.mean(depth_corners)

        # depth_ground = depth_frame[y1, x1] * self.depth_scale,  # top-left corner

        # Calculate the distance to the quasi-center as the average of the depth to the rock and the depth to the ground
        rock_center_depth = (depth_surface + depth_ground) / 2

        # Deproject the center pixel to 3D space based on the quasi depth
        rock_center_coordinates = rs.rs2_deproject_pixel_to_point(intrinsics, center_pixel, rock_center_depth)

        # Convert the 3D point to a numpy array for easy manipulation
        rock_center_coordinates = np.array(rock_center_coordinates)

        return rock_center_coordinates, rock_center_depth