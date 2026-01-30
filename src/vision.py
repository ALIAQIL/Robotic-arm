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
        Returns (cx, cy, area, label) of the largest/most confident detection, or None.
        """
        model = self._get_yolo_model()
        results = model(frame, verbose=False)
        if not results or not results[0].boxes:
            return None

        boxes = results[0].boxes
        names = results[0].names

        # Pick the detection with highest confidence (or largest area)
        best = None
        best_score = 0.0
        for i in range(len(boxes)):
            xyxy = boxes.xyxy[i].cpu().numpy()
            conf = float(boxes.conf[i].cpu().numpy())
            cls_id = int(boxes.cls[i].cpu().numpy())
            label = names.get(cls_id, f"class_{cls_id}")
            x1, y1, x2, y2 = xyxy
            area = (x2 - x1) * (y2 - y1)
            # Score = confidence * log(area) to prefer both confident and reasonably large
            score = conf * (1.0 + np.log1p(area) / 20.0)
            if score > best_score and area > 500:
                best_score = score
                best = (x1, y1, x2, y2, area, label)

        if best is None:
            return None
        x1, y1, x2, y2, area, label = best
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        return (cx, cy, int(area), label)

    def detect_object_color(self, frame):
        """
        Detects the largest object (Tomato or Potato) using HSV color.
        Returns (x, y, area, label) or None.
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

        contours_tomato, _ = cv2.findContours(mask_tomato, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_potato, _ = cv2.findContours(mask_potato, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest_object = None
        max_area = 0

        for contours, label in [(contours_tomato, "tomato"), (contours_potato, "potato")]:
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 500 and area > max_area:
                    max_area = area
                    largest_object = (c, label)

        if largest_object:
            contour, label = largest_object
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy, int(max_area), label)
        return None

    def detect_object(self, frame):
        """
        Detects the most prominent object in the frame.
        Uses AI (YOLO) if use_ai=True and available, else color-based.
        Returns (cx, cy, area, label) or None.
        """
        if self.use_ai:
            result = self.detect_object_ai(frame)
            if result is not None:
                return result
        return self.detect_object_color(frame)
