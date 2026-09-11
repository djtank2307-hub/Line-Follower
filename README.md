

# Advanced PID Line Follower Robot

An optimized, autonomous line-following robot built with Arduino. This project utilizes a Proportional-Integral-Derivative (PID) control algorithm for smooth tracking, dynamic line-loss recovery, and sensor-based sharp 90-degree turn and smooth curve, execution without relying on blind delays.

![Robot Image]([Link to your uploaded photo, e.g., media/robot.jpg])

## 🚀 Features
* **PID Line Tracking:** Calculates error via a 5-channel IR array for proportional steering.
* **Active Braking & Reversing:** Motor driver logic allows negative speeds for tight cornering and anti-overshoot braking.
* **Smart Turn Execution:** Rapidly polls the center sensor during 90-degree pivots to stop perfectly on the new line.
* **Line-Loss Recovery:** Memorizes the last known direction and executes a sweep search if the track is completely lost.

## 🛠️ Bill of Materials (BOM)

| Component | Quantity | Notes |
| :--- | :--- | :--- |
| **Arduino UNO R4** | 1 | Or equivalent microcontroller |
| **L298N Motor Driver** | 1 | Dual H-Bridge |
| **5-Channel IR Sensor** | 1 | For line detection |
| **2WD Chassis Kit** | 1 | Includes DC motors and wheels |
| **Power Supply** | 1 | [Enter your battery type, e.g., 7.4V LiPo] |

## 🔌 Pin Mapping

**Motor Driver (L298N)**
* `ENA` -> Pin 9 (PWM)
* `IN1` -> Pin 4
* `IN2` -> Pin 5
* `IN3` -> Pin 2
* `IN4` -> Pin 3
* `ENB` -> Pin 10 (PWM)

**Sensor Array**
* `S1` (Far Left) -> A0
* `S2` (Left) -> A1
* `S3` (Center) -> A2
* `S4` (Right) -> A3
* `S5` (Far Right) -> A4

## ⚙️ PID Tuning Guide
If you replicate this build, you may need to adjust the variables in the code based on your chassis weight and battery voltage:
* **`Kp` (Proportional):** Controls how aggressively the robot steers towards the line. Increase if the bot ignores curves; decrease if it violently wobbles.
* **`Kd` (Derivative):** Acts as a dampener. Increase this to smooth out the wobbling caused by `Kp`. 
* **`Ki` (Integral):** Accumulates past errors to help with long, sweeping curves. Keep this value very small (e.g., `0.002`) to prevent integral windup.

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
