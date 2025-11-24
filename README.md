# Development of a Robotic Companion Motion Control Algorithm Using Visual Feedback

This repository contains the material from my bachelor thesis **"Development of a Robotic Companion Motion Control Algorithm Using Visual Feedback"**, completed at the Department of Computer Science & Telecommunications, University of Thessaly (Lamia, Greece), March 2024.

The goal of this project is to design and implement a **human-following robotic companion** using a **Pioneer 3-AT** mobile robot, a **PlayStation Eye camera**, and **visual feedback**. The system detects a person in real time and controls the robot so that it follows the person while maintaining a safe, constant distance.

---

## 🧠 What the project does

- Uses a **PlayStation Eye** camera to capture live video.
- Applies **OpenCV** together with a **MobileNet SSD** object detection model to detect humans in the camera feed.
- Extracts the **bounding box** of the detected person and converts image features into motion commands using the **Image-Based Visual Servoing (IBVS)** algorithm.
- Sends linear and angular velocity commands to the **Pioneer 3-AT** via **ROSARIA / AriaCoda** in a ROS Noetic environment.
- Logs errors, logic flags and velocities and visualizes them with **matplotlib** and **rqt_graph** for analysis.

In practice, the robot becomes a simple **robotic companion** that is able to lock on to a person and follow them autonomously based only on camera data.

---

## 🔧 Technologies & Tools

- **Robot platform:** Pioneer 3-AT (Autonomous Mobile Robot)
- **Operating system:** Ubuntu 20.04 LTS
- **Middleware:** ROS Noetic Ninjemys
- **Robot interface:** ROSARIA + AriaCoda
- **Computer vision:** OpenCV 4.4.0
- **Object detection:** MobileNet SSD (Single Shot Detector)
- **Control:** Image-Based Visual Servoing (IBVS)
- **Programming language:** Python 3.8
- **Development environment:** VS Code
- **Simulation / additional tools:** MobileSim, rqt_graph, camera calibration tools

---

## 📂 Repository contents

Typical structure (may vary depending on how the code is organized):

- `thesis/` – PDF of the full thesis (Greek & English abstract)
- `src/` – ROS package with Python scripts:
  - `camera_publisher.py` – publishes camera frames as ROS image topics
  - `camera_subscriber.py` – detects persons using MobileNet SSD and publishes bounding box data + logic flag
  - `cameratest.py` – alternative subscriber using a KCF tracker for more robust person tracking
  - `controller.py` – implements IBVS and publishes velocity commands to the robot
  - `plots.py` – subscribes to error, logic values and velocities and generates plots
- `models/` – MobileNet SSD models and configuration files
- `calibration/` – camera calibration files (YAML) and auxiliary material

---

## 🚀 High-level workflow

1. **Start ROS master** and connect to the Pioneer 3-AT (or MobileSim).
2. **Publish camera frames** using `camera_publisher.py`.
3. **Detect the person** in the video stream with `camera_subscriber.py` or `cameratest.py`.
4. **Run the controller** (`controller.py`) to compute velocities via IBVS.
5. **Send commands** to the robot through ROSARIA and make the robot follow the person.
6. **Log & visualize** the error term, boolean detection flag and velocities using `plots.py` and ROS tools like `rqt_graph`.

---

## 📄 Thesis

The full thesis (in Greek, with an English abstract) is included in this repository as a PDF and describes in detail:

- The theoretical background of **autonomous mobile robots (AMRs)** and robotic companions
- The architecture and capabilities of the **Pioneer 3-AT**
- The **ROS** ecosystem and the **ROSARIA / AriaCoda** interface
- The use of **OpenCV** and **MobileNet SSD** for real-time person detection
- The **IBVS** control law, math, and implementation
- Experimental setup, results, plots and conclusions

---

## 📬 Contact

**Author:** Konstantinos Tavatzian  
**Email:** konstantinostavatzian@gmail.com  
**LinkedIn:** https://www.linkedin.com/in/konstantinos-tavatzian/

If you are interested in ROS, computer vision, or human-following robots, feel free to reach out!
