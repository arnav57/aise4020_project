# Software & Autonomy (Sprint 2)

This sprint was set up creating the ROS2 system on the Pi creating some base classes and testing a sensor.

# ROS2 System

The ROS2 system can be found in the `$ROOT/software/src/car_sys/car_sys/` directory. Some basic classes are described below

- `device.py` : ABC (abstract base class) for an input/output node for our ROS2 network, its the direct connection to a hardware point. All types of sensors we use should extend this base class

- `type.py` : Enum to hold device types (types of sensors etc)

- `display.py`  : Node that controls a 2x16 LCD display. Lets us write whatever we want to it, power up and powerdowns safely.

# Testing the LCD Sensor

I tested the ABC implenentation through a child class created for this 2x16 LCD Display. It works well, powers up, powers down safely, etc. So stuff works.

Going forward adding sensors will be much easier and standardized.

# Next Steps

Set things up on the car's Pi, and add more sensors to get a whole sensor network up at once. Also start work on the vision system for traffic sign detection.