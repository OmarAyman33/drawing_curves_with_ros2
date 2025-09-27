#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

frequency = 1000  # Hz


def butterfly(t) -> tuple[float, float]:
    """
    Returns (v, omega) for the butterfly curve at time t.
    x = S(t) sin(t), y = S(t) cos(t),
    S = e^{cos t} - 2 cos(4t) - sin^5(t/12)
    v = sqrt((dx)^2 + (dy)^2), omega = (dx*ddy - dy*ddx)/(dx^2 + dy^2)
    """
    SCALE = 0.7

    s = math.sin(t)
    c = math.cos(t)
    u = math.sin(t / 12.0)
    cu = math.cos(t / 12.0)

    # S, S', S''
    S = math.exp(c) - 2.0 * math.cos(4.0 * t) - u**5
    Sp = -math.exp(c) * s + 8.0 * math.sin(4.0 * t) - (5.0 / 12.0) * (u**4) * cu
    Spp = (
        math.exp(c) * (s * s - c)
        + 32.0 * math.cos(4.0 * t)
        - (5.0 / 36.0) * (u**3) * (cu**2)
        + (5.0 / 144.0) * (u**5)
    )

    # derivatives of x = S sin t, y = S cos t
    dx = SCALE * (Sp * s + S * c)
    dy = SCALE * (Sp * c - S * s)
    ddx = SCALE * (Spp * s + 2.0 * Sp * c - S * s)
    ddy = SCALE * (Spp * c - 2.0 * Sp * s - S * c)

    v2 = dx*dx + dy*dy
    if v2 < 1e-9:
        return 0.0, 0.0
    v = math.sqrt(v2)
    omega = (dx * ddy - dy * ddx) / v2
    return v, omega


def lissajous(t) -> tuple[float, float]:
    """
    Returns (v, omega) for a Lissajous curve at time t.
    x = SCALE * Ax * sin(a t + δ), y = SCALE * Ay * sin(b t)
    v = sqrt((dx)^2 + (dy)^2), omega = (dx*ddy - dy*ddx)/(dx^2 + dy^2)
    """
    Ax, Ay = 5.0, 5.0
    a, b = 3.0, 2.0
    delta = math.pi / 2
    SCALE = 0.4

    s1, c1 = math.sin(a * t + delta), math.cos(a * t + delta)
    s2, c2 = math.sin(b * t), math.cos(b * t)

    # first derivatives
    dx = SCALE * Ax * a * c1
    dy = SCALE * Ay * b * c2

    # second derivatives
    ddx = SCALE * (-Ax) * (a * a) * s1
    ddy = SCALE * (-Ay) * (b * b) * s2

    v2 = dx*dx + dy*dy
    if v2 < 1e-12:
        return 0.0, 0.0

    v = math.sqrt(v2)
    omega = (dx * ddy - dy * ddx) / v2
    return v, omega


def rosace(t) -> tuple[float, float]:
    """
    Returns (v, omega) for a rosace (rose/rhodonea) curve at time t.
    r(θ) = a * cos(k*θ), with θ = t
    x = r * cos(θ), y = r * sin(θ)
    v = sqrt((dx)^2 + (dy)^2), omega = (dx*ddy - dy*ddx)/(dx^2 + dy^2)
    """
    a = 2.5     # petal size (radius of outer tips)
    k = 2.25    # petals: odd k→k petals; even k→2k petals

    theta = t
    c, s = math.cos(theta), math.sin(theta)

    r   = a * math.cos(k * theta)
    dr  = -a * k * math.sin(k * theta)
    d2r = -a * (k**2) * math.cos(k * theta)

    # x = r cosθ, y = r sinθ  (θ = t)
    dx  = (dr * c - r * s)
    dy  = (dr * s + r * c)
    ddx = (d2r * c - 2.0 * dr * s - r * c)
    ddy = (d2r * s + 2.0 * dr * c - r * s)

    v2 = dx*dx + dy*dy
    if v2 < 1e-9:
        return 0.0, 0.0
    v = math.sqrt(v2)
    omega = (dx * ddy - dy * ddx) / v2
    return v, omega


def choose_shape():
    options = {
        "butterfly": butterfly,
        "lissajous": lissajous,
        "rose": rosace,
    }
    print("Choose a shape to draw:")
    print(" - butterfly")
    print(" - lissajous")
    print(" - rose")
    choice = input("Shape [butterfly/lissajous/rose] (default: butterfly): ").strip().lower()
    return options.get(choice, butterfly), (choice if choice in options else "butterfly")


class DrawNode(Node):
    def __init__(self, shape_fn, shape_name: str):
        super().__init__("draw_shapes")
        self.t = 0.0
        self.shape_fn = shape_fn
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(1.0 / frequency, self.send_velocity_cmd)
        self.get_logger().info(f"DrawNode initialized. Shape: {shape_name}. Frequency: {frequency} Hz.")

    def send_velocity_cmd(self):
        v, w = self.shape_fn(self.t)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        self.t += 1.0 / frequency


def main(args=None):
    shape_fn, shape_name = choose_shape()

    rclpy.init(args=args)
    node = DrawNode(shape_fn, shape_name)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
