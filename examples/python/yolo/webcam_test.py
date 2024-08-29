import cv2
import numpy as np
import os
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
import time


def draw_fps(frame, fps):
    """
    Draw the FPS on the frame.
    """
    cv2.putText(
        frame,
        f"FPS: {fps:4.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

def test_yolov8_segmentation_with_tracking_on_realsense(model_path):
    """
    Tests a YOLOv8 segmentation model with instance tracking on a RealSense camera stream.
    :param model_path: Path to the YOLOv8 segmentation model (.pt file).
    """
    # Load the YOLOv8 segmentation model
    model = YOLO(model_path)
    
    # Create a pipeline for RealSense
    cap = cv2.VideoCapture(0)    # Create a config and configure the pipeline to stream color frames
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # Set the frame width and height (optional)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    prev_time = 0

    try:
        # Create a window to display the livestream
        cv2.namedWindow("YOLOv8 Segmentation with Tracking on RealSense", cv2.WINDOW_AUTOSIZE)

        img_count = 0  # Image counter for naming the files
        
        while True:
            # Wait for a coherent pair of frames: color frame
            ret, color_image = cap.read()

            current_time = time.time()
            time_diff = current_time - prev_time
            fps = 1.0 / time_diff
            prev_time = current_time


            # Convert the color image to a numpy array

            # Perform instance segmentation and tracking on the color image
            # results = model.track(color_image, persist=True)
            results = False
            # Check if there are any results
            if results:
                annotator = Annotator(color_image, line_width=2)
                for result in results:
                    masks = result.masks  # Extract the masks
                    boxes = result.boxes  # Extract the bounding boxes
                    track_ids = result.boxes.id.int().cpu().tolist() if result.boxes.id is not None else []

                    # Check if masks are not None
                    if masks is not None:
                        # Convert the masks to a binary format and apply them to the frame
                        for mask, track_id in zip(masks.data, track_ids):  # Iterate over masks with corresponding track IDs
                            mask = mask.cpu().numpy()  # Convert the mask to a NumPy array
                            mask = mask.squeeze()  # Remove unnecessary dimensions
                            mask = (mask > 0.5).astype(np.uint8)  # Convert to binary mask

                            # Create a 3-channel mask for color blending
                            mask_img = np.zeros_like(color_image, dtype=np.uint8)
                            mask_img[:, :, 1] = mask * 255  # Apply green color to the mask

                            # Blend the mask with the original frame
                            color_image = cv2.addWeighted(color_image, 1, mask_img, 0.5, 0)

                    # Check if boxes are not None
                    if boxes is not None:
                        # Draw bounding boxes with track IDs
                        for box, track_id in zip(boxes, track_ids):
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            conf = box.conf[0]
                            cls = box.cls[0]
                            # Draw the bounding box on the frame
                            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red color box
                            # Annotate the frame with the class, confidence score, and track ID
                            cv2.putText(color_image, f"{model.names[int(cls)]} {track_id}: {conf:.2f}", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Display the frame with masks, bounding boxes, and track IDs
            draw_fps(color_image, fps)
            cv2.imshow("YOLOv8 Segmentation with Tracking on RealSense", color_image)

            # Exit loop if 'q' is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
           

    finally:
        # Stop the pipeline and close OpenCV windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Replace 'yolov8s-seg.pt' with your model path
    obj_type = 'rock'
    model_name = 'rock_m_200.pt'
    print(os.getcwd())
    model_path = f'{os.getcwd()}/models/{model_name}'  # Example segmentation model path
    test_yolov8_segmentation_with_tracking_on_realsense(model_path)




        

