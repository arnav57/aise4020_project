import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, UInt16, Bool, String
from geometry_msgs.msg import Twist
from flask import Flask, render_template_string, Response, request, jsonify
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading
import time
from rcl_interfaces.msg import Parameter # Add this import at the top
from rcl_interfaces.srv import SetParameters, GetParameters

# --- GLOBALS FOR UI ---
last_frame = None
current_battery = 0.0
current_lane_error = 0
frame_lock = threading.Lock()

# --- ROS 2 UI NODE ---
class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.bridge = CvBridge()
        self.param_client = self.create_client(SetParameters, '/pid_node/set_parameters')
        self.get_param_client = self.create_client(GetParameters, '/pid_node/get_parameters')

        self.get_logger().info("Waiting for PID node parameters services...")
        self.param_client.wait_for_service(timeout_sec=2.0)
        self.get_param_client.wait_for_service(timeout_sec=2.0)

        # Publishers: Sending UI commands to the DriveNode
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.beep_pub = self.create_publisher(UInt16, '/beep', 10)
        self.thresh_pub = self.create_publisher(Int32, '/vision/threshold', 10)
        self.enable_pub = self.create_publisher(Bool, '/lane_keep/enable', 10)
        self.lane_cmd_pub = self.create_publisher(String, '/lane_cmd', 10)

        # Subscribers: Listening to the car's state
        self.create_subscription(Image, '/camera/mod_frame', self._image_cb, 10)
        self.create_subscription(UInt16, '/battery', self._batt_cb, 10)
        self.create_subscription(Int32, '/lane_error', self._error_cb, 10)

    def _image_cb(self, msg):
        global last_frame
        cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with frame_lock:
            last_frame = cv_frame

    def _batt_cb(self, msg):
        global current_battery
        current_battery = round(msg.data, 2)

    def _error_cb(self, msg):
        global current_lane_error
        current_lane_error = msg.data

app = Flask(__name__)
ros_node = None

HTML_PAGE = """
<html>
    <body style="background:#111; color:#0f0; text-align:center; font-family:monospace;">
        <h1>4020 Web Control</h1>
        <div style="font-size: 1.5em; border: 1px solid #0f0; display: inline-block; padding: 10px;">
            BATTERY: <span id="batt">--</span>% | LANE ERROR: <span id="err">0</span>px
        </div>
        <br><br>
        <div style="display:flex; justify-content:center; gap:20px;">
            <div><h3>FPV FEED</h3><img src="/video_feed" width="500" style="border:2px solid #333;"></div>
        </div>
        <br>
        <button onclick="api('b', '100')">TRIGGER BEEP</button>
        <br><br>
        <b>PAN CONTROL</b><br>
        <input type="range" min="-90" max="90" value="0" oninput="api('s', this.value)">
        <br><br>
        <b>VISION THRESHOLD</b><br>
        <input type="range" min="0" max= "255" value="150" oninput="api('t', this.value)">
        <span id="thresh_val">150</span>
        <div style="border: 2px solid #fff; padding: 20px; margin-top: 20px;">
            <h3>AUTOPILOT CONTROL</h3>
            <button onclick="api('auto', '1')" style="background: green; color: white; padding: 10px;">ENGAGE</button>
            <button onclick="api('auto', '0')" style="background: red; color: white; padding: 10px;">DISENGAGE (KILL)</button>
        </div>
        <div style="border: 1px solid #0f0; padding: 15px; margin-top: 10px; background: #000;">
            <h3>LIVE PID TUNING</h3>
            
            <div>
                Kp: <input type="range" id="kp_slider" min="0" max="100" value="5" oninput="updateParam('kp', this.value)">
                <span id="kp_val" style="color: #ff0;">0.005</span>
            </div>

            <div>
                Kd: <input type="range" id="kd_slider" min="0" max="50" value="1" oninput="updateParam('kd', this.value)">
                <span id="kd_val" style="color: #ff0;">0.001</span>
            </div>

            <div>
                Speed: <input type="range" id="speed_slider" min="0" max="100" value="15" oninput="updateParam('speed', this.value)">
                <span id="speed_val" style="color: #ff0;">0.15</span> m/s
            </div>
        </div>
        <div class="lane-controls" style="margin-top: 20px;">
            <h3>Lane Switching</h3>
            <button onclick="switchLane('left')" style="padding: 10px 20px; font-size: 16px;">?? Switch Left</button>
            <button onclick="switchLane('right')" style="padding: 10px 20px; font-size: 16px;">Switch Right ??</button>
        </div>
        <script>
            // Update the label next to the slider
            function updateThreshLabel(val) {
                document.getElementById('thresh_val').innerText = val;
                api('t', val);
            }

            function switchLane(direction) {
                fetch('/api/switch_lane', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({ direction: direction })
                })
                .then(response => response.json())
                .then(data => {
                    console.log('Lane shift commanded:', data.direction);
                })
                .catch(error => console.error('Error:', error));
            }

            let activeKeys = new Set();
            function api(t, v) { fetch(`/api?t=${t}&v=${v}`); }
            
            document.onkeydown = (e) => {
                const k = e.key.toLowerCase();
                if(['w','a','s','d'].includes(k) && !activeKeys.has(k)) {
                    activeKeys.add(k); api('d', Array.from(activeKeys).join(''));
                }
            };
            document.onkeyup = (e) => {
                const k = e.key.toLowerCase();
                if(activeKeys.has(k)) {
                    activeKeys.delete(k); api('d', Array.from(activeKeys).join(''));
                }
            };

            setInterval(async () => {
                const res = await fetch('/get_telemetry');
                const data = await res.json();
                document.getElementById('batt').innerText = data.battery;
                document.getElementById('err').innerText = data.error;
            }, 500);

           function updateParam(type, val) {
                // Update the local label immediately for responsiveness
                let displayVal = (type === 'speed') ? (val / 100).toFixed(2) : (val / 1000).toFixed(3);
                document.getElementById(type + '_val').innerText = displayVal;
                
                // Send to ROS
                fetch(`/api?t=${type}&v=${val}`);
            }

            // 2. Function to FETCH current state from the car
            async function syncLiveParams() {
                try {
                    const res = await fetch('/get_params');
                    const data = await res.json();
                    
                    if (!data.error) {
                        // Only update the slider if the user ISN'T currently touching it
                        // (prevents the slider from "fighting" your mouse)
                        if (document.activeElement.type !== 'range') {
                            document.getElementById('kp_slider').value = data.kp;
                            document.getElementById('kd_slider').value = data.kd;
                            document.getElementById('speed_slider').value = data.speed;
                            
                            document.getElementById('kp_val').innerText = (data.kp / 1000).toFixed(3);
                            document.getElementById('kd_val').innerText = (data.kd / 1000).toFixed(3);
                            document.getElementById('speed_val').innerText = (data.speed / 100).toFixed(2);
                        }
                    }
                } catch (e) { console.error("Sync failed", e); }
            }

            // 3. Start the Heartbeat (500ms)
            setInterval(syncLiveParams, 500);
        </script>
    </body>
</html>
"""

@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            if last_frame is not None:
                with frame_lock:
                    view = last_frame.copy()
                # Overlay the error signal visually
                cv2.line(view, (320, 0), (320, 480), (0, 255, 0), 1)
                cv2.circle(view, (320 + current_lane_error, 400), 10, (0, 0, 255), -1)
                _, buf = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 50])
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
            time.sleep(0.05)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api')
def handle_api():
    t, v = request.args.get('t'), request.args.get('v')
    if t == 'd':
        tw = Twist()
        if 'w' in v: tw.linear.x = 0.3
        elif 's' in v: tw.linear.x = -0.3
        if 'a' in v: tw.angular.z = 0.8
        elif 'd' in v: tw.angular.z = -0.8
        ros_node.drive_pub.publish(tw)
    elif t == 's':
        ros_node.pan_pub.publish(Int32(data=int(v)))
    elif t == 'b':
        ros_node.beep_pub.publish(UInt16(data=int(v)))
    elif t == 't': # 't' for threshold
        ros_node.thresh_pub.publish(Int32(data=int(v)))
    elif t == 'auto':
        is_enabled = (v == '1')
        ros_node.enable_pub.publish(Bool(data=is_enabled))
    elif t in ['kp', 'ki', 'kd', 'speed']:
        req = SetParameters.Request()
        float_val = float(v) / 1000.0 if t != 'speed' else float(v) / 100.0

        param = Parameter()
        param.name = t
        param.value.type = 3 # double
        param.value.double_value = float_val
        req.parameters.append(param)
        ros_node.param_client.call_async(req)
    return "OK"

@app.route('/get_params')
def get_params():
    req = GetParameters.Request()
    req.names = ['kp', 'kd', 'speed']
    
    # We use a synchronous-style call here for simplicity in Flask
    future = ros_node.get_param_client.call_async(req)
    
    # Wait up to 1 second for the PID node to respond
    start_time = time.time()
    while not future.done() and time.time() - start_time < 1.0:
        time.sleep(0.01)
        
    if future.done():
        res = future.result()
        # ROS stores values in a list; map them back to a dict
        # Remember: we stored them as (value / 1000) or (value / 100)
        return jsonify({
            "kp": int(res.values[0].double_value * 1000),
            "kd": int(res.values[1].double_value * 1000),
            "speed": int(res.values[2].double_value * 100)
        })
    return jsonify({"error": "timeout"})

@app.route('/get_telemetry')
def get_telemetry():
    return jsonify({"battery": current_battery, "error": current_lane_error})

@app.route('/api/switch_lane', methods=['POST'])
def switch_lane():
    data = request.json
    direction = data.get('direction')
    
    if direction in ['left', 'right']:
        msg = String()
        msg.data = direction
        ros_node.lane_cmd_pub.publish(msg) # Assuming 'ros_node' is your WebNode instance
        return jsonify({"status": "success", "direction": direction}), 200
        
    return jsonify({"status": "error"}), 400

def main():
    global ros_node
    rclpy.init()
    ros_node = WebNode()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True)).start()
    rclpy.spin(ros_node)

if __name__ == '__main__':
    main()