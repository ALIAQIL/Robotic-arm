import cv2
import numpy as np

class ObjectDetector:
    def __init__(self):
        # Define range for red color in HSV (Tomato)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        # Define range for brown/yellow color in HSV (Potato)
        # Adjust these values based on actual lighting/object color
        self.lower_potato = np.array([15, 50, 50])
        self.upper_potato = np.array([35, 255, 255])

    def detect_object(self, frame):
        """
        Detects the largest object (Tomato or Potato) in the frame.
        Returns (x, y, area, label) of the center, or None if not found.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- Detect Tomato (Red) ---
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_tomato = mask_red1 + mask_red2
        
        # --- Detect Potato (Brown) ---
        mask_potato = cv2.inRange(hsv, self.lower_potato, self.upper_potato)

        # Process masks
        kernel = np.ones((5, 5), np.uint8)
        mask_tomato = cv2.morphologyEx(mask_tomato, cv2.MORPH_OPEN, kernel)
        mask_tomato = cv2.morphologyEx(mask_tomato, cv2.MORPH_CLOSE, kernel)
        
        mask_potato = cv2.morphologyEx(mask_potato, cv2.MORPH_OPEN, kernel)
        mask_potato = cv2.morphologyEx(mask_potato, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours_tomato, _ = cv2.findContours(mask_tomato, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_potato, _ = cv2.findContours(mask_potato, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest_object = None
        max_area = 0

        # Check Tomatoes
        if contours_tomato:
            c = max(contours_tomato, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500 and area > max_area:
                max_area = area
                largest_object = (c, "tomato")

        # Check Potatoes
        if contours_potato:
            c = max(contours_potato, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500 and area > max_area:
                max_area = area
                largest_object = (c, "potato")

        if largest_object:
            contour, label = largest_object
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy, max_area, label)
        
        return None
