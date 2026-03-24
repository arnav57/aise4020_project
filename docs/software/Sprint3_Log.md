# Software & Autonomy (Sprint 3)

This sprint was spent actually implementing things on the real car. The things I got working are listed below. Note that the setup assumes that a single piece of black tape on a background with enough contrast between itself and the tape, is what will be used for the in-person demo

1. Lane following
2. Lane switching
3. Brake Light Control
4. Left/Right Turn Light Control
5. ArUco "Traffic Sign" compliance
6. Web UI For Control

The topics are described in more detail below

## Lane Following 

A *lane* is defined to the car as a single piece of black tape. In our setup we used black electrical tape on top of a large cardboard box. Lane following works right now by performing a determinisitic image processing algorithm. This algorithm is present in `smart_car/smart_car/lib/lane_detect.py` 

We start by cropping the whole image, to only bottom 60% of the image for lane detection. This provides a balance between lookahead and current state (for steep turns). This was tuned empirically

We then add a gaussian blur to the image, and perform adaptive thresholding (locally based contrast finder) tunable through a web UI

we then perform a morphlogical opening on the image, this basically removes small patches that survive and only larger patches (like the lane) will fall through this step. Note that by this point the surviving points like the tape, are all white pixels

we then get a 2d histogram, summing up the count of white pixels in eahc column. This gives us a graph, that we then find the peak of, and deem that the lane, this is done through `scipy`'s `find_peaks` implementation. We also add some memory here to avoid constant flickering and random jumps. This means tracking the current position of the lane, and enforcing that new lanes must be within some set deviation limit of the previous one. We then find the distance of the peak to the center, and feed that into PID Control for the lane following.

## Lane Switching

Lane Switching is implemented by first rotating the camera left/right, and updating the memory for the lane-following algo to be all the way to the right of the screen, this ensures the camera tracks the left-most lane and doesnt retrack the original lane.


## Brake Light Control

We poll the current topic that holds the velocity of the car, and if its linear + angular velkocity is 0 we send a command over the Pi <-> Arduino link that tells the arduino to turn the brake lights on or off.

## Left/Right Turn Light Control

When lane switching happens, we also send a serial command over the Pi <-> Arduino Link that enables a blinker in the corresponding direction (left or right) before turning them off

## ArUco "Traffic Sign" Compliance

We define ArUco patches as traffic signs, as this would provide the easiest implementation for us. Addy created a baseline standalone ROS2 node (`aruco_detector`) developed and tested independently in VSCode, using `opencv-contrib-python` to validate detection before integration. The node subscribes to the raw camera stream and publishes detected marker IDs and instruction strings on edge transitions only — repeated detections of the same marker do not re-trigger commands. Arnav then improved this with edge-detection based logging/publishing, as well as adding time-based memory and integrated into the codebase, connecting to our existing infrastructure for speed control, turning left or right, and stopping.

Memory of the last seen marker clears after 1 second of no detection, allowing re-detection on the next encounter. The five markers and their assigned instructions are:

| Marker ID | Instruction | Handled By |
|-----------|-------------|------------|
| 0 | STOP | `arbiter_node.py` |
| 1 | SPEED_60 | `pid_node.py` |
| 2 | SPEED_100 | `pid_node.py` |
| 3 | TURN_LEFT | `aruco_node.py` → `/lane_cmd` |
| 4 | TURN_RIGHT | `aruco_node.py` → `/lane_cmd` |

All markers are 30×30mm, generated using the **4x4 (50, 100, 250, 1000)** dictionary at https://chev.me/arucogen/. To regenerate or reprint, set Dictionary to `4x4 (50, 100, 250, 1000)`, enter the corresponding Marker ID, and set Marker size to `30mm`. Ensure the white border is preserved when printing.

*Photos of the printed markers used in testing:*

![Marker ID 0 — STOP](software/aruco_markers/ArucoTag0.jpg)
||![Marker ID 1 — SPEED_60](software/aruco_markers/ArucoTag1.jpg)
||![Marker ID 2 — SPEED_100](software/aruco_markers/ArucoTag2.jpg)
||![Marker ID 3 — TURN_LEFT](software/aruco_markers/ArucoTag3.jpg)
||![Marker ID 4 — TURN_RIGHT](software/aruco_markers/ArucoTag4.jpg)

## Web UI For Control

This is just a FastAPI server that shows us the camera feed (raw + lane following debug) lets us tune the image processing algorithm for lane following and lets us enable/disable the self-driving along a predefined track.

### File Reference by Topic

#### 1. Lane Following
*Core vision processing and PID steering control.*

- **`software/smart_car/smart_car/nodes/vision_node.py`**: Handles camera capture, calls detection logic, and publishes pixel error.
- **`software/smart_car/smart_car/nodes/pid_node.py`**: Consumes `/lane_error` to calculate steering commands.
- **`software/smart_car/smart_car/lib/lane_detect.py`**: Implements Gaussian blur, adaptive thresholding, morphological opening, and peak detection.
- **`software/smart_car/smart_car/lib/pid.py`**: PID controller implementation.

#### 2. Lane Switching
*Active gaze control and lane memory manipulation.*

- **`software/smart_car/smart_car/nodes/vision_node.py`**: Implements "Active Gaze" (panning camera and offsetting lane memory).
- **`software/smart_car/smart_car/nodes/drive_node.py`**: Forwards `/servo_cmd` to physical hardware.

#### 3. Brake & Turn Light Control
*Serial communication between Pi and Arduino.*

- **`software/smart_car/smart_car/nodes/arduino_node.py`**: Monitors `/cmd_vel` for braking and `/lane_cmd` for blinkers.
- **`software/smart_car/smart_car/lib/arduino.py`**: Low-level serial interface for Arduino commands.

#### 4. ArUco "Traffic Sign" Compliance
*Marker detection and command overrides.*

- **`software/smart_car/smart_car/nodes/aruco_node.py`**: Detects markers and publishes sign types.
- **`software/smart_car/smart_car/nodes/arbiter_node.py`**: Triggers ESTOP latches for STOP signs.
- **`software/smart_car/smart_car/nodes/pid_node.py`**: Scales linear velocity based on speed signs.

#### 5. Web UI For Control
*Dashboard and live tuning.*

- **`software/smart_car/smart_car/nodes/web_node.py`**: FastAPI server

#### 6. ROS2 Launch Points
*What happens when we type `ros2 launch smart_car web.launch.py`*
- **`software/smart_car/launch/web.launch.py`**: Main launch file for the complete stack.
- **`software/smart_car/setup.py`**: Entry points for all ROS 2 nodes.
