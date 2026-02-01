import cv2
import time
import threading
import json
from flask import Flask, render_template, Response, request, jsonify
from motor_control import RoboticArm
from vision import ObjectDetector
from config import get_box_for_label, get_color_for_label, get_base_angle_for_box

app = Flask(__name__)

# Global instances
arm = RoboticArm()
detector = ObjectDetector()
camera = None
camera_lock = threading.Lock()
current_camera_index = 0

def find_available_cameras(max_to_test=5):
    """Scans and returns indices of available cameras."""
    available = []
    for i in range(max_to_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available.append(i)
            cap.release()
    return available

def init_camera(index):
    """Safely (re)initializes the camera."""
    global camera, current_camera_index
    with camera_lock:
        if camera is not None:
            camera.release()
        camera = cv2.VideoCapture(index)
        current_camera_index = index
        print(f"Camera {index} initialized.")
    return camera.isOpened()

# Initial camera setup
init_camera(0)

# Global state
state = {
    "mode": "manual",  # manual, auto, select
    "detections": [],
    "target_object": None,
    "last_frame": None,
    "is_busy": False,
    "stats": {
        "detected_today": 0,
        "tasks_completed": 0,
        "success_rate": 98.5,
        "response_time": 1.2
    }
}

def pick_sequence(label, angle_to_obj):
    """Executes the pick and sort sequence."""
    state["is_busy"] = True
    try:
        print(f"Starting pick sequence for {label}...")
        # 1. Align
        arm.move_base(angle_to_obj)
        time.sleep(1)
        
        # 2. Extend arm
        arm.move_shoulder(45)
        arm.move_elbow(-45)
        time.sleep(1)
        
        # 3. Grab
        arm.close_gripper()
        time.sleep(1)
        
        # 4. Lift
        arm.move_shoulder(0)
        arm.move_elbow(0)
        time.sleep(1)
        
        # 5. Move to Box
        box_id = get_box_for_label(label)
        angle = get_base_angle_for_box(box_id)
        arm.move_base(angle)
        time.sleep(1)
        
        # 6. Drop
        arm.open_gripper()
        time.sleep(1)
        
        # 7. Return Home
        arm.reset()
        state["stats"]["tasks_completed"] += 1
        print("Pick sequence complete.")
    finally:
        state["is_busy"] = False
        state["target_object"] = None

def processing_loop():
    """Background loop for camera capture and detection."""
    global state
    while True:
        with camera_lock:
            if camera is None or not camera.isOpened():
                time.sleep(0.1)
                continue
            ret, frame = camera.read()
            
        if not ret:
            time.sleep(0.1)
            continue

        # Detect all objects
        detections = detector.detect_all_objects(frame)
        
        # Process detections for UI
        processed_detections = []
        for i, det in enumerate(detections):
            cx, cy, area, label, x1, y1, x2, y2 = det
            color = get_color_for_label(label)
            
            # Draw on frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label} (#{i})", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            processed_detections.append({
                "id": i,
                "label": label.capitalize(),
                "cx": int(cx),
                "cy": int(cy),
                "area": int(area),
                "bbox": [int(x1), int(y1), int(x2), int(y2)]
            })
        
        # Only update "detected_today" if we see more than before in a single frame (simple heuristic)
        if len(processed_detections) > state.get("_prev_count", 0):
            state["stats"]["detected_today"] += (len(processed_detections) - state.get("_prev_count", 0))
        state["_prev_count"] = len(processed_detections)

        state["detections"] = processed_detections
        state["last_frame"] = frame.copy()

        # Handle Auto Mode
        if state["mode"] == "auto" and not state["is_busy"] and detections:
            # Pick the largest one
            best = max(detections, key=lambda x: x[2])
            cx, cy, area, label, x1, y1, x2, y2 = best
            
            # Simple angle mapping (rough estimation)
            frame_center_x = frame.shape[1] // 2
            error_x = cx - frame_center_x
            target_angle = arm.base.angle + (error_x / 10.0) # Very rough P-control
            
            threading.Thread(target=pick_sequence, args=(label, target_angle)).start()

        time.sleep(0.01)

@app.route('/')
def index():
    return render_template('index.html')

def gen_frames():
    while True:
        if state["last_frame"] is not None:
            ret, buffer = cv2.imencode('.jpg', state["last_frame"])
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/state')
def get_state():
    return jsonify({
        "mode": state["mode"],
        "is_busy": state["is_busy"],
        "detections": state["detections"],
        "stats": state["stats"],
        "angles": {
            "base": arm.base.angle,
            "shoulder": arm.shoulder.angle,
            "elbow": arm.elbow.angle,
            "gripper": arm.gripper.angle
        }
    })

@app.route('/api/control', methods=['POST'])
def control():
    data = request.json
    motor = data.get('motor')
    angle = data.get('angle')
    
    if state["is_busy"]:
        return jsonify({"status": "error", "message": "Arm is busy"}), 400
        
    if motor == 'base': arm.move_base(angle)
    elif motor == 'shoulder': arm.move_shoulder(angle)
    elif motor == 'elbow': arm.move_elbow(angle)
    elif motor == 'gripper':
        if angle > 0: arm.close_gripper()
        else: arm.open_gripper()
        
    return jsonify({"status": "success"})

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    data = request.json
    state["mode"] = data.get('mode', 'manual')
    return jsonify({"status": "success", "mode": state["mode"]})

@app.route('/api/capture', methods=['POST'])
def capture():
    if state["is_busy"]:
        return jsonify({"status": "error", "message": "Arm is busy"}), 400
    
    data = request.json
    obj_id = data.get('id')
    
    # Find object in current detections
    target = next((d for d in state["detections"] if d["id"] == obj_id), None)
    if target:
        # Calculate angle (same rough mapping)
        frame_width = 640 # Default assuming standard cam
        if state["last_frame"] is not None:
            frame_width = state["last_frame"].shape[1]
            
        frame_center_x = frame_width // 2
        error_x = target["cx"] - frame_center_x
        target_angle = arm.base.angle + (error_x / 10.0)
        
        threading.Thread(target=pick_sequence, args=(target["label"], target_angle)).start()
        return jsonify({"status": "success"})
    
    return jsonify({"status": "error", "message": "Object not found"}), 404

@app.route('/api/command', methods=['POST'])
def command():
    """Processes verbal commands."""
    import re
    data = request.json
    text = data.get('text', '').lower().strip()
    print(f"User Voice: {text}")

    if state["is_busy"]:
        return jsonify({"status": "busy", "message": "Arm is busy"})

    # 1. Capture Command: "capture the apple", "pick up tomato"
    cap_match = re.search(r"(?:capture|pick up|grab|take)\s+(?:the\s+)?([a-zA-Z\s]+)", text)
    if cap_match:
        target_label = cap_match.group(1).split()[0] # Take first word (e.g. "bottle")
        # Reuse existing capture logic find by label
        target = next((d for d in state["detections"] if d["label"].lower() == target_label), None)
        if target:
            threading.Thread(target=pick_sequence, args=(target["label"], arm.base.angle)).start() # Rough angle
            return jsonify({"status": "success", "message": f"Capturing {target_label}"})
        return jsonify({"status": "error", "message": f"Object '{target_label}' not found"})

    # 2. Movement Commands: "move base to 45", "rotate shoulder to -10"
    mov_match = re.search(r"move\s+(base|shoulder|elbow)\s+to\s+(-?\d+)", text)
    if mov_match:
        motor = mov_match.group(1)
        angle = int(mov_match.group(2))
        if motor == 'base': arm.move_base(angle)
        elif motor == 'shoulder': arm.move_shoulder(angle)
        elif motor == 'elbow': arm.move_elbow(angle)
        return jsonify({"status": "success", "message": f"Moving {motor} to {angle}"})

    # 3. Actions: "open gripper", "close hand", "reset arm"
    if "open" in text and "gripper" in text:
        arm.open_gripper()
        return jsonify({"status": "success", "message": "Gripper opened"})
    if "close" in text and ("gripper" in text or "hand" in text):
        arm.close_gripper()
        return jsonify({"status": "success", "message": "Gripper closed"})
    if "reset" in text or "home" in text:
        arm.reset()
        return jsonify({"status": "success", "message": "Arm reset to home"})

    # 4. Mode Switches
    if "auto" in text:
        state["mode"] = "auto"
        return jsonify({"status": "success", "message": "Switched to Auto Mode"})
    if "manual" in text:
        state["mode"] = "manual"
        return jsonify({"status": "success", "message": "Switched to Manual Mode"})

    return jsonify({"status": "error", "message": "Command not recognized"})

@app.route('/api/cameras')
def get_cameras():
    """Returns a list of available camera indices."""
    return jsonify({"cameras": find_available_cameras(), "current": current_camera_index})

@app.route('/api/set_camera', methods=['POST'])
def set_camera():
    """Switches the active camera."""
    data = request.json
    index = data.get('index', 0)
    success = init_camera(index)
    if success:
        return jsonify({"status": "success", "index": index})
    return jsonify({"status": "error", "message": f"Failed to open camera {index}"}), 400

if __name__ == '__main__':
    # Start processing thread
    threading.Thread(target=processing_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False)
