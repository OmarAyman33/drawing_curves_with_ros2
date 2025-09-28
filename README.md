# ROS 2 Turtlesim Curve Sketcher

This project sketches curves using the `turtlesim` node in **ROS 2**.

Code for the drawing node is in the `project.py` file

## Overview
The sketches originate from **parametric equations** (x(t), y(t)), which are converted into **linear** and **angular** velocity commands (v(t), w(t)) for the turtle:

These are published as `geometry_msgs/Twist` messages to trace the desired path.

## Included Sketches
- **Lissajous curve**
- **Rose curve**
- **Butterfly curve**

## Video Demo
[Watch on YouTube](https://youtu.be/SG_Js0KPgnk?si=2IAzR9HpfCCyBRp2)

>
> [![Watch the demo](https://img.youtube.com/vi/SG_Js0KPgnk/0.jpg)](https://youtu.be/SG_Js0KPgnk?si=2IAzR9HpfCCyBRp2)

---

**Requirements:** ROS 2 + `turtlesim` package.  

