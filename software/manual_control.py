import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from flask import Flask, render_template_string, Response, request, jsonify
import cv2
import threading
import numpy as np
import time

# --- GLOBAL CAMERA RESOURCE ---
# One producer, multiple consumers to prevent hardware resource contention
last_frame = None
frame_lock = threading.Lock()
line_center = 320  # Default to middle of 640px width

def camera_worker():
    """Dedicated thread to pull frames from hardware."""
    global last_frame
    # Use CAP_V4L2 for better compatibility with Pi 5 / Docker
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        success, frame = cap.read()
        if success:
            with frame_lock:
                last_frame = frame.copy()
        else:
            time.sleep(0.1)

# Start camera thread immediately
threading.Thread(target=camera_worker, daemon=True).start()

# --- ROS 2 NODE ---
class CockpitNode(Node):
    def __init__(self):
        super().__init__('cockpit_node')
        self.pan_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_pan = 0
        self.threshold_val = 150 
        
        self.create_subscription(Int32, '/servo_s1', self.pan_callback, 10)
        self.get_logger().info("4020 Project: Vision Cockpit Online")

    def pan_callback(self, msg): 
        self.current_pan = msg.data

app = Flask(__name__)
ros_node = None

HTML_PAGE = """
<html>
    <head>
        <title>4020 Mission Control</title>
        <script>
            function api(t, id, v) { fetch(`/api?t=${t}&id=${id}&v=${v}`); }
            let activeKeys = new Set();

            function updateDrive() {
                // We send the current state of WASD to the server
                let keys = Array.from(activeKeys).join('');
                fetch(`/api?t=d&v=${keys}`);
            }
            
            document.onkeydown = (e) => {
                const k = e.key.toLowerCase();
                if (['w','a','s','d'].includes(k) && !activeKeys.has(k)) {
                    activeKeys.add(k);
                    updateDrive();
                }
            };

            document.onkeyup = (e) => {
                const k = e.key.toLowerCase();
                if (activeKeys.has(k)) {
                    activeKeys.delete(k);
                    updateDrive();
                }
            };

            async function sync() {
                const res = await fetch('/get_state');
                const data = await res.json();
                const p = document.getElementById('pan');
                if (!p.matches(':active')) p.value = data.pan + 90;
            }
            setInterval(sync, 500);
        </script>
    </head>
    <body style="background:#111; color:#0f0; text-align:center; font-family:monospace;">
        <h1>PI-CAR X: MISSION CONTROL</h1>
        <div style="display:flex; justify-content:center; gap:20px;">
            <div><h3>RAW FEED + CENTROID</h3><img src="/video_feed" width="450"></div>
            <div><h3>VISION (THRESHOLD)</h3><img src="/vision_feed" width="450"></div>
        </div>
        <br>
        <div style="border:1px solid #333; padding:15px; display:inline-block; background:#222;">
            <b>PAN CALIBRATION (Hardware Center: 0)</b><br>
            <input type="range" id="pan" min="0" max="180" value="90" oninput="api('s', 's1', this.value)">
            <br><br>
            <b>VISION THRESHOLD (TUNE FOR ROAD)</b><br>
            <input type="range" min="0" max="255" value="150" oninput="api('v', 'th', this.value)">
        </div>
        <p>Drive with <b>WASD</b> | PID Error logic ready</p>
    </body>
</html>
"""

@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    def gen():
        global line_center
        while True:
            if last_frame is not None:
                with frame_lock:
                    display_frame = last_frame.copy()
                
                # Draw the detected lane center for debugging
                cv2.circle(display_frame, (line_center, 400), 10, (0, 0, 255), -1)
                cv2.line(display_frame, (320, 0), (320, 480), (0, 255, 0), 1) # Center line
                
                _, buffer = cv2.imencode('.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/vision_feed')
def vision_feed():
    def gen():
        global line_center
        while True:
            if last_frame is not None:
                with frame_lock:
                    gray = cv2.cvtColor(last_frame, cv2.COLOR_BGR2GRAY)
                
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                _, binary = cv2.threshold(blur, ros_node.threshold_val, 255, cv2.THRESH_BINARY)
                
                # --- CENTROID CALCULATION (The "EE" Math) ---
                # We look at a horizontal slice near the bottom of the image
                roi = binary[380:420, :] 
                M = cv2.moments(roi)
                if M['m00'] > 0:
                    line_center = int(M['m10'] / M['m00'])
                
                _, buffer = cv2.imencode('.jpg', binary)
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api')
def handle_api():
    t, id, v = request.args.get('t'), request.args.get('id'), request.args.get('v')
    
    if t == 's': # Servo logic remains same
        msg = Int32(data=int(v) - 90)
        ros_node.pan_pub.publish(msg)
    
    elif t == 'v': # Threshold logic remains same
        ros_node.threshold_val = int(v)
        
    elif t == 'd':
        tw = Twist()
        # Linear (Forward/Back)
        if 'w' in v: tw.linear.x = 0.3
        elif 's' in v: tw.linear.x = -0.3
        
        # Angular (Left/Right)
        if 'a' in v: tw.angular.z = 0.8
        elif 'd' in v: tw.angular.z = -0.8
        
        # If both 'w' and 'a' are in v, the Twist message 
        # now contains BOTH values, creating a curved path.
        ros_node.drive_pub.publish(tw)
        
    return "OK"

@app.route('/get_state')
def get_state():
    return jsonify({"pan": ros_node.current_pan})

def main():
    global ros_node
    rclpy.init()
    ros_node = CockpitNode()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True)).start()
    rclpy.spin(ros_node)

if __name__ == '__main__':
    main()