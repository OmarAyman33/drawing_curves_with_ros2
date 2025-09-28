# ROS 2 Turtlesim Curve Sketcher

This project sketches curves using the `turtlesim` node in **ROS 2**.

## Overview
The sketches originate from **parametric equations** \(x(t), y(t)\), which are converted into **linear** and **angular** velocity commands \(v(t), \omega(t)\) for the turtle:

- \(v(t) = \sqrt{\dot{x}(t)^2 + \dot{y}(t)^2}\)
- \(\omega(t) = \dfrac{\dot{x}(t)\,\ddot{y}(t) - \dot{y}(t)\,\ddot{x}(t)}{\dot{x}(t)^2 + \dot{y}(t)^2}\)

These are published as `geometry_msgs/Twist` messages to trace the desired path.

## Included Sketches
- **Lissajous curve**
- **Rose curve**
- **Butterfly curve**

## Video Demo
[Watch on YouTube](https://youtu.be/SG_Js0KPgnk?si=2IAzR9HpfCCyBRp2)

> Tip: Many Markdown renderers don’t support direct video embeds. You can use a clickable thumbnail like this:
>
> [![Watch the demo](https://img.youtube.com/vi/SG_Js0KPgnk/0.jpg)](https://youtu.be/SG_Js0KPgnk?si=2IAzR9HpfCCyBRp2)

---

**Requirements:** ROS 2 + `turtlesim` package.  
**Node output:** Publishes `Twist (linear.x = v, angular.z = ω)` to control the turtle.
