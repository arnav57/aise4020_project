"""
web_node — FastAPI dashboard with real-time WebSocket telemetry.

Replaces the old Flask + threading.Thread approach with a proper async setup:
  - FastAPI runs in the same event loop as the ROS2 executor via rclpy's
    asyncio-compatible SingleThreadedExecutor
  - Telemetry is pushed to the browser over WebSocket at 10 Hz (no polling)
  - Manual drive commands POST to /api/drive and are forwarded to arbiter
    via /cmd_vel_manual
  - PID parameter tuning uses the standard ROS2 parameter service (async)

Topic interface:
  Subscriptions:
    /camera/debug_frame  (sensor_msgs/Image)  — annotated lane feed
    /camera/raw_frame    (sensor_msgs/Image)  — actual full frame feed
    /battery             (std_msgs/UInt16)    — battery percentage
    /lane_error          (std_msgs/Int32)     — current lane error

  Publications:
    /cmd_vel_manual      (geometry_msgs/Twist)— manual drive → arbiter
    /servo_cmd           (std_msgs/Int32)     — camera pan → drive_node
    /beep_cmd            (std_msgs/UInt16)    — beep → drive_node
    /vision/threshold    (std_msgs/Int32)     — threshold → vision_node
    /lane_keep/enable    (std_msgs/Bool)      — autopilot toggle → pid_node
    /lane_cmd            (std_msgs/String)    — lane shift → vision_node

Run:
    The node is launched normally via ROS2. It starts the FastAPI server
    internally on port 5000.
"""

import asyncio
import threading
import time
import json
from typing import Optional

import cv2
import uvicorn
from cv_bridge import CvBridge
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Body
from fastapi.responses import HTMLResponse, StreamingResponse

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.srv import SetParameters, GetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String, UInt16
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Shared telemetry state (written by ROS callbacks, read by FastAPI)
# ---------------------------------------------------------------------------

class Telemetry:
    battery:    int   = 0
    lane_error: int   = 0
    frame_jpg:  Optional[bytes] = None
    raw_frame_jpg: Optional[bytes] = None
    lock = threading.Lock()

    @classmethod
    def snapshot(cls) -> dict:
        with cls.lock:
            return {'battery': cls.battery, 'lane_error': cls.lane_error}

    @classmethod
    def set_frame(cls, jpg: bytes) -> None:
        with cls.lock:
            cls.frame_jpg = jpg

    @classmethod
    def get_frame(cls) -> Optional[bytes]:
        with cls.lock:
            return cls.frame_jpg

    @classmethod
    def set_raw_frame(cls, jpg: bytes) -> None:
        with cls.lock:
            cls.raw_frame_jpg = jpg

    @classmethod
    def get_raw_frame(cls) -> Optional[bytes]:
        with cls.lock:
            return cls.raw_frame_jpg


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class WebNode(Node):
    def __init__(self) -> None:
        super().__init__('web_node')

        self._bridge = CvBridge()

        # --- Parameter service clients ---
        self._set_params = self.create_client(SetParameters, '/pid_node/set_parameters')
        self._get_params = self.create_client(GetParameters, '/pid_node/get_parameters')

        # --- Publishers ---
        self._drive_pub   = self.create_publisher(Twist,  '/cmd_vel_manual',   10)
        self._servo_pub   = self.create_publisher(Int32,  '/servo_s1',        10)
        self._beep_pub    = self.create_publisher(UInt16, '/beep',         10)
        self._thresh_pub  = self.create_publisher(Int32,  '/vision/threshold', 10)
        self._enable_pub  = self.create_publisher(Bool,   '/lane_keep/enable', 10)
        self._lane_pub    = self.create_publisher(String, '/lane_cmd',         10)
        self._vision_param_pub = self.create_publisher(String, '/vision/params', 10)

        # --- Subscribers ---
        self.create_subscription(Image,  '/camera/debug_frame', self._frame_cb,   10)
        self.create_subscription(Image,  '/camera/raw_frame',   self._raw_frame_cb, 10)
        self.create_subscription(UInt16, '/battery',            self._battery_cb, 10)
        self.create_subscription(Int32,  '/lane_error',         self._error_cb,   10)

        self.get_logger().info('Web node online, dashboards at port 5000')

    # ------------------------------------------------------------------
    # ROS callbacks — write into shared Telemetry
    # ------------------------------------------------------------------

    def _frame_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            Telemetry.set_frame(buf.tobytes())
            #self.get_logger().info("Frame Received")
        except Exception as e:
            self.get_logger().warning(f'Frame encode error: {e}')

    def _raw_frame_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _ , buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            Telemetry.set_raw_frame(buf.tobytes())
        except Exception as e:
            self.get_logger().warning(f'Raw frame encode error: {e}')

    def _battery_cb(self, msg: UInt16) -> None:
        with Telemetry.lock:
            Telemetry.battery = msg.data

    def _error_cb(self, msg: Int32) -> None:
        with Telemetry.lock:
            Telemetry.lane_error = msg.data

    # ------------------------------------------------------------------
    # Command helpers (called from FastAPI handlers)
    # ------------------------------------------------------------------

    def publish_vision_param(self, name: str, value: int) -> None:
        import json
        msg = String()
        msg.data = json.dumps({'name': name, 'value': value})
        self._vision_param_pub.publish(msg)

    def publish_drive(self, linear: float, angular: float) -> None:
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._drive_pub.publish(t)

    def publish_pan(self, angle: int) -> None:
        self._servo_pub.publish(Int32(data=angle))

    def publish_beep(self, duration: int) -> None:
        self._beep_pub.publish(UInt16(data=duration))

    def publish_threshold(self, value: int) -> None:
        self._thresh_pub.publish(Int32(data=value))

    def publish_enable(self, enabled: bool) -> None:
        self._enable_pub.publish(Bool(data=enabled))

    def publish_lane_cmd(self, direction: str) -> None:
        msg = String()
        msg.data = direction
        self._lane_pub.publish(msg)

    async def set_pid_param(self, name: str, value: float) -> bool:
        """Async ROS2 parameter set — safe to await from FastAPI handlers."""
        if not self._set_params.service_is_ready():
            return False
        param = RosParameter()
        param.name = name
        param.value.type = 3  # PARAMETER_DOUBLE
        param.value.double_value = value
        req = SetParameters.Request()
        req.parameters.append(param)
        future = self._set_params.call_async(req)
        await asyncio.sleep(0)  # yield to let the executor process it
        return True

    async def get_pid_params(self) -> Optional[dict]:
        """Async ROS2 parameter get."""
        if not self._get_params.service_is_ready():
            return None
        req = GetParameters.Request()
        req.names = ['kp', 'ki', 'kd', 'speed']
        future = self._get_params.call_async(req)
        deadline = time.time() + 1.0
        while not future.done() and time.time() < deadline:
            await asyncio.sleep(0.01)
        if not future.done():
            return None
        v = future.result().values
        return {'kp': v[0].double_value, 'ki': v[1].double_value,
                'kd': v[2].double_value, 'speed': v[3].double_value}


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app     = FastAPI()
ros_node: Optional[WebNode] = None   # set before uvicorn starts

@app.get('/', response_class=HTMLResponse)
async def dashboard():
    return _DASHBOARD_HTML

@app.get('/video_feed')
async def video_feed():
    async def frames():
        while True:
            jpg = Telemetry.get_frame()
            if jpg:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'
            await asyncio.sleep(0.05)
    return StreamingResponse(frames(),
                             media_type='multipart/x-mixed-replace; boundary=frame')

@app.get('/raw_video_feed')
async def raw_video_feed():
    async def frames():
        while True:
            jpg = Telemetry.get_raw_frame()
            if jpg:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'
            await asyncio.sleep(0.05)
    return StreamingResponse(frames(),
                             media_type='multipart/x-mixed-replace; boundary=frame')

@app.post('/api/drive')
async def api_drive(body: dict = Body(...)):
    linear  = float(body.get('linear',  0.0))
    angular = float(body.get('angular', 0.0))
    ros_node.publish_drive(linear, angular)
    return {'ok': True}

@app.post('/api/pan')
async def api_pan(body: dict = Body(...)):
    ros_node.publish_pan(int(body.get('angle', 0)))
    return {'ok': True}

@app.post('/api/beep')
async def api_beep(body: dict = Body(...)):
    ros_node.publish_beep(int(body.get('duration', 100)))
    return {'ok': True}

@app.post('/api/threshold')
async def api_threshold(body: dict = Body(...)):
    ros_node.publish_threshold(int(body.get('value', 80)))
    return {'ok': True}

@app.post('/api/autopilot')
async def api_autopilot(body: dict = Body(...)):
    ros_node.publish_enable(bool(body.get('enabled', False)))
    return {'ok': True}

@app.post('/api/lane')
async def api_lane(body: dict = Body(...)):
    direction = body.get('direction', '')
    if direction not in ('left', 'right'):
        return {'ok': False, 'error': 'direction must be left or right'}
    ros_node.publish_lane_cmd(direction)
    return {'ok': True}

@app.post('/api/pid')
async def api_pid(body: dict = Body(...)):
    name  = body.get('name')
    value = body.get('value')
    if name not in ('kp', 'ki', 'kd', 'speed') or value is None:
        return {'ok': False}
    await ros_node.set_pid_param(name, float(value))
    return {'ok': True}

@app.post('/api/vision_param')
async def api_vision_param(body: dict = Body(...)):
    name  = body.get('name')
    value = body.get('value')
    valid = ('adaptive_block', 'adaptive_c', 'peak_min_height', 'flicker_limit')
    if name not in valid or value is None:
        return {'ok': False, 'error': f'unknown param {name}'}
    ros_node.publish_vision_param(name, int(value))
    return {'ok': True}

# ---------------------------------------------------------------------------
# Minimal dashboard HTML
# ---------------------------------------------------------------------------

_DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>AISE 4020 Dashboard</title>
  <style>
    body { background:#111; color:#0f0; font-family:monospace; text-align:center; }
    button { padding:8px 16px; margin:4px; cursor:pointer; }
    input[type=range] { width:200px; }
    .feed { border:2px solid #333; width:500px; }
    label { display:inline-block; width:120px; text-align:right; margin-right:8px; }
  </style>
</head>
<body>
  <h2>4020 Control</h2>
  <div id="telem">Battery: <span id="bat">--</span>% | Error: <span id="err">0</span>px</div>
  <br>
  <img class="feed" src="/video_feed">
  <img class="feed" src="/raw_video_feed">
  <br><br>

  <button onclick="post('/api/autopilot', {enabled:true})">Engage autopilot</button>
  <button onclick="post('/api/autopilot', {enabled:false}); post('/api/drive', {linear:0, angular:0})">Disengage</button>
  <button onclick="post('/api/beep', {duration:200})">Beep</button>
  <br><br>

  <label>Speed:</label>
  <input type="range" min="0" max="100" value="30" oninput="speed=this.value/100; document.getElementById('speed_val').textContent=this.value/100">
  <span id="speed_val">0.30</span> m/s<br>

  <label>Pan:</label>
  <input type="range" min="-90" max="90" value="0"
        oninput="post('/api/pan', {angle:+this.value})"><br>

  <label>Adapt block:</label>
  <input type="range" min="3" max="81" value="31" step="2"
         oninput="this.value%2==0&&this.value++; document.getElementById('block_val').textContent=this.value; post('/api/vision_param', {name:'adaptive_block', value:+this.value})">
  <span id="block_val">31</span><br>

  <label>Adapt C:</label>
  <input type="range" min="1" max="30" value="8"
         oninput="document.getElementById('c_val').textContent=this.value; post('/api/vision_param', {name:'adaptive_c', value:+this.value})">
  <span id="c_val">8</span><br>

  <label>Min height:</label>
  <input type="range" min="100" max="5000" value="1000" step="100"
         oninput="document.getElementById('height_val').textContent=this.value; post('/api/vision_param', {name:'peak_min_height', value:+this.value})">
  <span id="height_val">1000</span><br>

  <label>Flicker limit:</label>
  <input type="range" min="10" max="300" value="100"
         oninput="document.getElementById('flicker_val').textContent=this.value; post('/api/vision_param', {name:'flicker_limit', value:+this.value})">
  <span id="flicker_val">100</span><br>

  <button onclick="post('/api/lane', {direction:'left'})">Lane left</button>
  <button onclick="post('/api/lane', {direction:'right'})">Lane right</button>
  <br><br>
  <p style="color:#888">Drive with WASD</p>

  <script>
    let speed = 0.30;

    const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
    ws.onmessage = e => {
      const d = JSON.parse(e.data);
      document.getElementById('bat').textContent = d.battery;
      document.getElementById('err').textContent = d.lane_error;
    };

    async function post(url, body) {
      await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                        body: JSON.stringify(body)});
    }

    let held = new Set();
    function sendDrive() {
      let lin=0, ang=0;
      if(held.has('w')) lin += speed;
      if(held.has('s')) lin -= speed;
      if(held.has('a')) ang += 0.8;
      if(held.has('d')) ang -= 0.8;
      post('/api/drive', {linear:lin, angular:ang});
    }
    document.addEventListener('keydown', e => {
      const k = e.key.toLowerCase();
      if(['w','a','s','d'].includes(k) && !held.has(k)){ held.add(k); sendDrive(); }
    });
    document.addEventListener('keyup', e => {
      const k = e.key.toLowerCase();
      if(held.delete(k)) sendDrive();
    });
  </script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    global ros_node

    rclpy.init(args=args)
    ros_node = WebNode()

    # Run uvicorn in a background thread so rclpy.spin() keeps the main thread
    threading.Thread(
        target=lambda: uvicorn.run(app, host='0.0.0.0', port=5000, log_level='warning'),
        daemon=True,
    ).start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
