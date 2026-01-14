# 🐭 ESP32 Diagonal Wall Follower

> **"I don't just follow walls; I anticipate them."**

Welcome to the **Diagonal Wall Follower** project! This isn't your average "bang-bang" maze solver. This bot uses **diagonal sensor geometry** (North-West, North-East, North) to see walls *before* it hits them, allowing for smoother, smarter, and faster maze solving.

## 🌟 Features
- **Predictive PID**: Uses angled sensors to detect subtle changes in wall distance.
- **Smart Recovery**: Detects when it's stuck and performs an intelligent backup-and-scan maneuver.
- **Gap Detection**: Accurately identifies left-hand turns and corridor openings using trigonometry.
- **Modular Codebase**: Clean, professional C++ structure.

---

## 🛠️ The Hardware
This code is designed for an ESP32-based robot with the following configuration:

### Pinout (`config.h`)
| Component | Pin | Note |
|-----------|-----|------|
| **Front Sonar** | Trig: 14, Echo: 35 | Straight ahead |
| **Left Sonar (NW)** | Trig: 18, Echo: 13 | Angled 45° Left |
| **Right Sonar (NE)** | Trig: 16, Echo: 34 | Angled 45° Right |
| **Motor Left** | IN1: 23, IN2: 22 | PWM: 25 |
| **Motor Right** | IN3: 21, IN4: 19 | PWM: 26 |
| **Start Button** | BOOT Button (GPIO 0) | Built-in |

---

## 🧠 The Brain (How it Works)

### 1. The Diagonal Advantage
Most bots use side sensors (90°). Ours uses diagonal sensors (45°). 
- **Why?** 
    - A side sensor only sees the wall *right now*.
    - A diagonal sensor effectively "looks ahead," minimizing the need for derivative (D) term spikes in PID.
    - Result: Smoother driving!

### 2. Priority Logic (`WallFollower.ino`)
The loop follows a strict hierarchy of survival:
1.  **Goal Check**: "Am I in a big open space?" -> Stop and Celebrate.
2.  **Stuck Check**: "Have I not moved for 3 cycles?" -> Back up and Re-evaluate.
3.  **Gap Check**: "Is there a left turn?" -> Take it (Left-Hand Rule).
4.  **Collision Avoidance**: "Is there a wall in front?" -> Turn away.
5.  **PID Cruise**: "Just keep swimming..." -> Maintain center path.

### 3. The Code Structure
We've organized the code into professional modules:
- `config.h`: The "Control Center" for all tuning knobs.
- `motors.cpp`: The driver layer (literally).
- `sensors.cpp`: Eyes of the operation.
- `pid.cpp`: The mathematical heart.

---

## 🚀 How to Run
1.  **Clone this repo.**
2.  Open `WallFollower.ino` in Arduino IDE.
3.  Install **NewPing** library if you haven't.
4.  **Upload** to your ESP32.
5.  Place bot in maze.
6.  Press the **BOOT** button to start!

---

## 🔧 Tuning
Want to make it faster? Tweaks inside `config.h`:
- `BASE_SPEED`: Increase for speed (default 130).
- `PID_KP`: Increase if it responds too slowly to walls.
- `PID_KD`: Increase if it wobbles.

---
*Happy Mazing!* 🏁
