# Echo Technologies

## Problem Statement

*Previous ocean floor inspections, sampling, and recovery operations have been done by deploying a ship with a tethered vehicle to dive to the ocean floor. However, this method cannot be done without drawing unwanted attention to a potentially proprietary mission given a large ship is visible for all to see. Today’s technology still has not yet offered a solution as radio waves fail to penetrate the water to any useful depth.*

ECHO Technologies intends to design and build a UAV/UUV that can be deployed autonomously and wirelessly by using cutting edge sensors and programing along with a stable drone platform that can be used to secretively inspect the ocean floor.

## Underwater Subsystem

The underwater control system consists of:
- an Arduino
- 6 [Electronic Speed Controllers](https://www.bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/) for 4 vertical and 2 horizontal motors
- an [Inertial Measurement Unit](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
- a [Barometer](https://www.bluerobotics.com/store/electronics/bar02-sensor-r1-rp/)

All software for the Arduino can be found in the [underwater folder](https://github.com/tedklin/Echo/tree/master/Megalodon/src/underwater) of this repository. 

**The main file that synthesizes all algorithms is *[navigation](https://github.com/tedklin/Echo/tree/master/Megalodon/src/underwater/navigation)*.**

**There is also an option for RC control with stability assistance: *[teleop](https://github.com/tedklin/Echo/blob/master/Megalodon/src/underwater/teleop/teleop.ino)*.**

## Vision Subsystem

The vision system consists of:
- a Raspberry Pi
- a [waterproof USB camera](https://www.amazon.com/dp/B07N5DX18T/ref=sspa_dk_detail_2?psc=1&pd_rd_i=B07N5DX18T)

The [vision folder](https://github.com/tedklin/Echo/tree/master/Megalodon/src/vision) contains code for full 3D pose estimation of visual fiducial markers known as [AprilTags](https://april.eecs.umich.edu/software/apriltag), as well as modified colored object detection based off of [NerdyVision](https://github.com/tedklin/nerdyvision).

The vision algorithm works in tandem with the underwater autonomous object retreival algorithm. We implemented a serial connection between the Raspberry Pi and the Arduino, which allows the Arduino to send data such as current robot pose and the Raspberry Pi to reply with data about the location and orientation of the object of interest.
