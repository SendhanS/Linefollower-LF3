# LF3 – Line Follower Robot (Modular PCB + Custom Chassis)

**LF3** is a compact line follower robot designed with a custom PCB carrier board, N20 motors, and a chassis optimized for wired sensor integration. The design is focused on modularity and ease of assembly, allowing sensors to be externally mounted and replaced without soldering to the board.

---

## 📌 Key Features

- **Microcontroller:** Arduino Nano (inserted via female headers)
- **Motor Driver:** L293D onboard
- **Motors:** N20 gear motors
- **Sensor Input:** External sensor modules via header wires (IR array or similar)
- **Chassis:** Custom design made in SolidWorks, with an outline provided for easy fabrication
- **Power Supply:** 7.4V or 2-cell Li-ion battery, not more than 12V

---

## 📁 Files Included

- `LF3.ino` – Main Arduino sketch for basic line following
- `Mchassis.sldprt` – SolidWorks part file of the chassis
- `assembly.sldasm` – Full assembly file
- `LF3` – EasyEDA project file for the PCB
- `schematic.png` – Electrical schematic overview
- `chassis_outline.pdf` – Printable chassis template
- `*.gbr` – Gerber files for PCB fabrication
- `*.jpg/.png` – Robot build and design reference images

---

## 📂 Purpose

This project was created as a practical robotics platform — simple enough for rapid prototyping, but structured to reflect real-world modularity. It’s well-suited for testing motor control, evaluating sensor placement strategies, and embedded programming using Arduino.

---

## 📝 License

This project is open-source under the MIT License.

---

## 👤 Author

Designed by **Sendhan S**  
Feel free to fork, modify, or share feedback.  
Connect with me on [LinkedIn](https://www.linkedin.com/in/sendhan-s-2483a3274/)
