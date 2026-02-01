import cv2
import numpy as np

# Optional AI-based detection (YOLO)
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class ObjectDetector:
    """
    Object detector supporting:
    - AI mode: YOLO for any COCO class (80 object types)
    - Color mode: HSV-based detection for tomato/potato (fallback)
    """
    def __init__(self, use_ai=True):
        self.use_ai = use_ai and YOLO_AVAILABLE
        self._yolo_model = None

        # Define range for red color in HSV (Tomato)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        # Define range for brown/yellow color in HSV (Potato)
        self.lower_potato = np.array([15, 50, 50])
        self.upper_potato = np.array([35, 255, 255])

    def _get_yolo_model(self):
        if self._yolo_model is None:
            self._yolo_model = YOLO("yolov8n.pt")
        return self._yolo_model

    def detect_object_ai(self, frame):
        """
        Detect objects using YOLO (any COCO class).
        Returns a list of (cx, cy, area, label, x1, y1, x2, y2) detections.
        """
        model = self._get_yolo_model()
        results = model(frame, verbose=False)
        if not results or not results[0].boxes:
            return []

        boxes = results[0].boxes
        names = results[0].names
        detections = []

        for i in range(len(boxes)):
            xyxy = boxes.xyxy[i].cpu().numpy()
            conf = float(boxes.conf[i].cpu().numpy())
            cls_id = int(boxes.cls[i].cpu().numpy())
            label = names.get(cls_id, f"class_{cls_id}")
            x1, y1, x2, y2 = xyxy
            area = (x2 - x1) * (y2 - y1)
            
            if area > 500 and conf > 0.3:
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                detections.append((cx, cy, int(area), label, int(x1), int(y1), int(x2), int(y2)))

        return detections

    def detect_object_color(self, frame):
        """
        Detects objects (Tomato or Potato) using HSV color.
        Returns a list of (cx, cy, area, label, x1, y1, x2, y2) detections.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_tomato = mask_red1 + mask_red2
        mask_potato = cv2.inRange(hsv, self.lower_potato, self.upper_potato)

        kernel = np.ones((5, 5), np.uint8)
        mask_tomato = cv2.morphologyEx(mask_tomato, cv2.MORPH_OPEN, kernel)
        mask_tomato = cv2.morphologyEx(mask_tomato, cv2.MORPH_CLOSE, kernel)
        mask_potato = cv2.morphologyEx(mask_potato, cv2.MORPH_OPEN, kernel)
        mask_potato = cv2.morphologyEx(mask_potato, cv2.MORPH_CLOSE, kernel)

        detections = []

        for mask, label in [(mask_tomato, "tomato"), (mask_potato, "potato")]:
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        detections.append((cx, cy, int(area), label, x, y, x + w, y + h))
        
        return detections

    def detect_object(self, frame):
        """
        Backward compatibility: Detects the most prominent object in the frame.
        """
        all_objs = self.detect_all_objects(frame)
        if not all_objs:
            return None
        # Return the largest one
        return max(all_objs, key=lambda x: x[2])[:4]

    def detect_all_objects(self, frame):
        """
        Detects all prominent objects in the frame.
        Uses AI (YOLO) if use_ai=True and available, else color-based.
        Returns a list of (cx, cy, area, label, x1, y1, x2, y2) detections.
        """
        if self.use_ai:
            return self.detect_object_ai(frame)
        return self.detect_object_color(frame)
