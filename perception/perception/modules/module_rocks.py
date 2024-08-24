from module_interface import ModuleInterface
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import torch

# Check if CUDA is available
cuda_available = torch.cuda.is_available()
device = torch.device("cuda" if cuda_available else "cpu")

class ModuleRocks(ModuleInterface):
    def __init__(self, config_file):
        # Initialize the camera
        self.pipeline, self.align, self.depth_scale, self.intrinsics = self.initialize_camera()

        # Load the YOLO model
        self.model = YOLO(config_file)
        self.model.to(device)
        
        # Placeholder for the current processed frame
        self.current_frame = None
        self.detected_objects = None

    def __call__(self, rgb_frame: np.ndarray, depth_frame: np.ndarray):
        # Process the frame with the model
        self.current_frame, self.detected_objects = self.process_frame(rgb_frame)
        
        # Draw bounding boxes and masks
        self.current_frame, self.detected_objects = self.draw_bounding_boxes(self.current_frame, self.detected_objects)
        self.current_frame, self.detected_objects = self.draw_masks_and_contours(self.current_frame, self.detected_objects)
        
        # Calculate axes and dimensions for each detected object
        for obj in self.detected_objects:
            contour = obj['contour']
            if contour is not None:
                center = obj['center']
                depth_value = depth_frame[center[1], center[0]] * self.depth_scale

                max_dist, min_dist, max_pts, min_pts = self.calculate_axes(contour, center)
                self.current_frame = self.draw_axes(self.current_frame, max_pts, min_pts)

                self.current_frame, max_dim_cm, min_dim_cm = self.calculate_real_dimensions(
                    self.current_frame, obj, max_dist, min_dist, depth_value, self.intrinsics)

                cv2.putText(self.current_frame,
                            f"Max: {max_dim_cm:.2f}cm, Min: {min_dim_cm:.2f}cm",
                            (center[0] + 10, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            2)

                print(f"Rock at ({center[0]}, {center[1]}): Max Diameter = {max_dim_cm:.2f}cm, Min Diameter = {min_dim_cm:.2f}cm")

        return self.current_frame

    def draw(self, frame: np.ndarray) -> None:
        cv2.imshow("Rock Segmentation", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def initialize_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        profile = pipeline.start(config)

        align = rs.align(rs.stream.color)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        return pipeline, align, depth_scale, intrinsics

    def process_frame(self, image):
        results = self.model(image, imgsz=640)
        return image, results

    def draw_bounding_boxes(self, image, results):
        detected_objects = []
        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                boxes = result.boxes.xyxy.cpu().numpy()
                for mask, box in zip(masks, boxes):
                    mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    binary_mask = (mask > 0.5).astype(np.uint8)

                    x1, y1, x2, y2 = box.astype(int)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue color for bounding box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    cv2.circle(image, (center_x, center_y), 5, (255, 255, 0), -1)  # Cyan color for center

                    detected_objects.append({
                        'mask': binary_mask,
                        'contour': None,
                        'center': (center_x, center_y),
                        'bounding_box': (x1, y1, x2, y2)
                    })
        return image, detected_objects

    def draw_masks_and_contours(self, image, detected_objects):
        for obj in detected_objects:
            binary_mask = obj['mask']

            # Find contours
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
            if contours:
                contour = max(contours, key=cv2.contourArea)
                obj['contour'] = contour

        return image, detected_objects

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