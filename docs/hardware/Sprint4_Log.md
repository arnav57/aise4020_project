# **Hardware Sprint 4**

During this Sprint I focused on a working car alongside the software lead, we worked testing the car and all the integrated sensors, and the actuators. We tested the implemented sign recognition, and finally demo the working prototype.

## 1. **Sensing Data**

- The 4 sharp IR sensors mounted on the PiCar, two in the front and two in the back.
- The data was visible using serial comunications while the car was following its path, anything farther than 30cm was ignored, and we tested it with obstacles were the pressions was confirmed.
- Two capapcitors as bulk capacitors were added as the Arduino had problems while using the four sensosrs.

## 2. **Turning and Braking Lights**

- Turning lights were added through Arduino to the PiCar, they were tested manually.
- The turning lights were made to be controlled by the PICar's RaspberryPI, when the car decides to cahnge lines is informs by blinking that side lights until the line changing is finished.
- The braking lights as well are controlled internally, when the car slows downs or breaks the red light turns on, but when it stops the light remains on until forward march is started again.

## 3. **Power delivery**

- A 9V battery was found to not be enought to drive all the sensors and lights at the same time, therefore a second one is to be used to power the four IR sensors, alongside a 5.1V zener diode to keep the supply voltage at 5V exclusively for the IR sensors.


## 4. **Demo**

- The demo was sucessfull as the car followed the path and followed the signs put in its way in a random order, showing that it can recognize the signs and act accordingtly, the battery for the turning and breaking lights run down due to the lenght of the demo time, and that it was not easily remplasable, something that needs to be addresed for further improvements. The data of the IR sensors was accurate as expected by the prior test runs.