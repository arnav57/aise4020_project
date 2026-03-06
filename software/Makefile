.PHONY: build clean launch

# Compiles the workspace
build:
	@colcon build

# Nukes the cache so you can do a fresh build
clean:
	@rm -rf build/ install/ log/

# Sources the environment and launches the car
launch:
	@bash -c "source install/setup.bash && ros2 launch car_sys run.launch.py"