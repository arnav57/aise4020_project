# **Hardware Sprint 3**

During this Sprint I focused on getting all the sensors for the redundancy working and installed in the PiCar.
It was also part of this sprint to get the Arduino and the Raspberry Pi to communicate among them, to get and receive the data from the sensors and from the action the car plans to take.

## 1. **Redundancy**

- The 4 sharp IR sensors were mounted on the PiCar, two in the front and two in the back.
- The working code was uploaded to the Arduino and tested in the working car.

![Front sensor mounted](https://drive.google.com/uc?export=view&id=12spLaX_IR20rjfzXdpwDYv_KAFAdK_mu)

![Rear sensor mounted](https://drive.google.com/uc?export=view&id=1JmDCP-XKa0r78l6luUVodBsXqEV2hJ9T)

## 2. **Arduino UNO**

- The Arduino was mounted to the PiCar.
- Serial communications between the car Raspberry Pi and the Arduino Uno were established.

![Arduino mounted in the side of PiCar](https://drive.google.com/uc?export=view&id=1Yp9ARZcAnOAEUTpk_Dib3HKJLUZtMRa_)

## 3. **Turning and Braking Lights**

- Turning lights were added through Arduino to the PiCar, they were tested manually.
- Then modified to be controlled by the decisions made by the PiCar.

![Red braking lights, and green turning lights](https://drive.google.com/uc?export=view&id=1L3KWC3AuvQMMq-tv-amcoCOG6JTHWSv-)

## 4. **Mounted to the PiCar**

- All the above listed components were mounted to the PiCar.
- A 9V battery was added to power the Arduino independently.

![Board with shield and circuit](https://drive.google.com/uc?export=view&id=1ZCxpqL7eldWIDql6oPNwheb0z_It4GR7)

## 5. **Test Run**

- Finally a test run was conducted to be able to see how the acquisition of data and the communication between the controllers was working.
- In the video, we can see the PiCar turning the blinker on to the line it wants to go to.

[Watch Test Run Video](https://drive.google.com/file/d/1jh_ZF8tgPilZ9c-5-zuWsLnqEr8I3ssX/view?usp=sharing)