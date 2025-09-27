# Turtle Curves: Drawing Parametric Shapes with ROS 2 (v, ω)

This project drives a **turtlesim** turtle along beautiful 2D curves by publishing **unicycle** commands:
- linear speed \(v(t)\) on `Twist.linear.x`
- angular rate \(\omega(t)\) on `Twist.angular.z`

You pick a shape in the terminal (e.g., **butterfly**, **lissajous**, **rose**) and the node streams \(v,\omega\) at a high rate to trace the curve.

---

## Contents

- [Demo Shapes](#demo-shapes)
- [How It Works (Math)](#how-it-works-math)
  - [General parametric curves \(x(t),y(t)\)](#general-parametric-curves-xtyt)
  - [Polar curves \(r(\theta)\) with \(\theta=t\)](#polar-curves-rtheta-with-thetat)
  - [Worked mini-examples](#worked-mini-examples)
- [Run It](#run-it)
- [Customize / Add Your Own Curve](#customize--add-your-own-curve)
- [Notes & Tips](#notes--tips)

---

## Demo Shapes

All shapes follow the same pattern: define \(x(t), y(t)\), compute first and second time-derivatives, then form \(v,\omega\).

- **Butterfly**  
  \(x=S(t)\sin t,\quad y=S(t)\cos t\)  
  \(S(t)=e^{\cos t}-2\cos(4t)-\sin^5\!\big(t/12\big)\)

- **Lissajous**  
  \(x=A_x\sin(a t+\delta),\quad y=A_y\sin(b t)\)

- **Rosace (Rose / Rhodonea)**  
  Polar: \(r(\theta)=a\cos(k\theta)\) with \(\theta=t\).  
  Converted to Cartesian for derivatives.

---

## How It Works (Math)

### General parametric curves \(x(t),y(t)\)

Given any smooth planar curve parameterized by time \(t\),
- **Speed** (arc-length rate)
  \[
  v(t)=\sqrt{\dot x(t)^2+\dot y(t)^2}.
  \]
- **Heading rate** (unicycle angular velocity)
  \[
  \omega(t)=\frac{\dot x(t)\,\ddot y(t)-\dot y(t)\,\ddot x(t)}{\dot x(t)^2+\dot y(t)^2}.
  \]

Here \(\dot{(\cdot)}=\frac{d}{dt}\), \(\ddot{(\cdot)}=\frac{d^2}{dt^2}\).  
This follows from the Frenet–Serret relations in 2D: \(\omega=\kappa\,v\) and
\[
\kappa(t)=\frac{\dot x\,\ddot y-\dot y\,\ddot x}{(\dot x^2+\dot y^2)^{3/2}}.
\]

**Implementation recipe**
1. Start from \(x(t)\), \(y(t)\).
2. Compute \(\dot x, \dot y, \ddot x, \ddot y\).
3. Plug into the two boxed formulas above → get \(v(t), \omega(t)\).
4. Publish \(v,\omega\) at your control frequency.

---

### Polar curves \(r(\theta)\) with \(\theta=t\)

Sometimes a curve is more natural in polar form:
\[
x=r(\theta)\cos\theta,\qquad y=r(\theta)\sin\theta,\qquad \theta=t.
\]
Then
\[
\begin{aligned}
\dot x &= r'\cos\theta - r\sin\theta, &
\dot y &= r'\sin\theta + r\cos\theta,\\
\ddot x&= r''\cos\theta - 2r'\sin\theta - r\cos\theta, &
\ddot y&= r''\sin\theta + 2r'\cos\theta - r\sin\theta,
\end{aligned}
\]
where \(r'=\frac{dr}{d\theta}\), \(r''=\frac{d^2r}{d\theta^2}\) and \(\theta=t\).  
Feed \((\dot x,\dot y,\ddot x,\ddot y)\) into the general \(v,\omega\) formulas.

A very handy identity (when \(T(t)=t+d\) is a simple rotation): if
\[
x=R(t)\cos T(t),\quad y=R(t)\sin T(t),\quad \dot T=1,
\]
then
\[
v=\sqrt{R^2+(R')^2},\qquad
\omega=\frac{R^2+2(R')^2-RR''}{R^2+(R')^2},
\]
with derivatives taken w.r.t. \(t\). (We use this trick in several shapes.)

---

### Worked mini-examples

**1) Lissajous**  
\(x=A_x\sin(a t+\delta),\quad y=A_y\sin(b t).\)  
\(\dot x=A_x a\cos(a t+\delta),\quad \ddot x=-A_x a^2\sin(a t+\delta),\)  
\(\dot y=A_y b\cos(b t),\quad\ \ \ddot y=-A_y b^2\sin(b t).\)  
Then apply the general \(v,\omega\) formulas.

**2) Rosace (Rose)**  
\(r(\theta)=a\cos(k\theta),\quad \theta=t.\)  
\(r'=-ak\sin(k t),\quad r''=-ak^2\cos(k t),\)  
plug into the polar derivative identities above to get \(\dot x,\dot y,\ddot x,\ddot y\), then compute \(v,\omega\).

---

## Run It

### Prerequisites
- **ROS 2** (any recent distro)
- **turtlesim** package installed

### Start turtlesim
```bash
ros2 run turtlesim turtlesim_node
```

### Run the curve driver
```bash
python3 draw_shapes.py
```

You’ll be prompted:
```
Choose a shape to draw:
 - butterfly
 - lissajous
 - rose
Shape [butterfly/lissajous/rose] (default: butterfly):
```

Pick one (or press Enter for the default). The node will start publishing \(v,\omega\) at `frequency = 1000 Hz`.

---

## Customize / Add Your Own Curve

1. **Define** \(x(t), y(t)\) (or \(r(\theta)\) with \(\theta=t\)).
2. **Differentiate** to get \(\dot x, \dot y, \ddot x, \ddot y\).  
   - For polar: compute \(r', r''\) and use the identities above.
3. **Return** \((v,\omega)\) from your function:
   ```python
   v = math.sqrt(dx*dx + dy*dy)
   denom = dx*dx + dy*dy
   omega = (dx*ddy - dy*ddx) / denom if denom != 0.0 else 0.0
   return v, omega
   ```
4. **Register** your function in `choose_shape()` so it appears in the menu.

**Tuning tips**
- Change amplitudes/frequencies to fit the turtlesim window and pacing.
- Integer ratios (e.g., \(a:b=3:2\) for Lissajous, or integer \(k\) in rose) give closed, symmetric figures.

---

## Notes & Tips

- The code guards division by zero by checking \( \dot x^2+\dot y^2 \) before computing \(\omega\). If it’s (near) zero, the function returns \((0,0)\) for that tick.
- Higher `frequency` (e.g., 1000 Hz) makes motion silky; reduce if your system needs to save CPU.
- You can easily add a `/clear` call or keyboard control if you want stop/clear behavior—left out here to keep the core example minimal.

---

Happy tracing! 🐢✨
