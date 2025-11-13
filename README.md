
# 🤖 Cooperative Manipulation with Two Franka Emika Panda Robots

This project implements a **dual-arm cooperative manipulation task** using two Franka Emika Panda robots in Swift/Robotics Toolbox (Python). The robots work together to grasp, stabilize, and manipulate a shared rigid rod using real-time kinematics and inverse kinematics (IK) updates.

---

## 🔍 Project Overview

The goal of this project is to simulate **two robots collaboratively holding and moving a rod** without breaking contact.  
- **Panda 1 (leader)** is controlled directly via Swift sliders.  
- **Panda 2 (follower)** automatically computes IK to maintain a fixed grasp point on the opposite end of the rod.  

The rod is “welded” to Panda 1’s gripper, meaning any movement of Panda 1 updates the rod pose. Panda 2 continuously solves IK to reach its fixed grasp frame relative to the rod.

This project demonstrates:
- Cooperative dual-arm manipulation  
- Forward & inverse kinematics  
- Relative transformations between object frames  
- Real-time robotics simulation using Swift  
- Maintaining rigid-body constraints across multiple manipulators  

---

## 🚀 Features

### **✔ Real-time dual-arm simulation**
Each robot is loaded with its own base transform and can be controlled independently.

### **✔ Leader–Follower Architecture**
- Panda 1 defines the rod’s pose each frame.  
- Panda 2 computes an IK target to stay attached to the far end of the rod.

### **✔ Constant Grasp Frames**
Two fixed grasp transforms (**G_right** and **G_left**) are defined relative to the rod frame.

### **✔ Smooth IK Behavior**
The follower robot solves IK each loop to remain aligned with the shared object.

### **✔ Full Swift Visualization**
The environment displays:
- Both robots  
- The cylindrical rod  
- All live pose updates  

---

## 🛠 Technologies Used

- **Python 3**
- **Robotics Toolbox for Python (rtb)**
- **Swift Simulator**
- **spatialmath / spatialgeometry**
- SE(3) transformations & IK solvers

---

## 📁 File Structure

```
a3_2Pandas.py     # Main script for dual-arm rod manipulation
rod_example.png   # (Optional) Reference image for rod alignment
README.md         # Project documentation
```

---

## ▶️ How to Run

1. Install dependencies:
   ```bash
   pip install roboticstoolbox-python spatialmath-python swift-sim spatialgeometry
   ```

2. Launch simulation:
   ```bash
   python3 a3_2Pandas.py
   ```

3. Use Swift GUI sliders to move **Panda 1**.  
   Panda 2 will automatically follow.

---

## 🧠 Key Concepts Demonstrated

### **Inverse Kinematics (IK)**
Panda 2 solves IK every timestep to keep its gripper aligned with its grasp point on the rod.

### **Forward Kinematics (FK)**
Panda 1’s FK determines the rod’s pose:
```
T_rod = T_tool_right @ inv(G_right)
```

### **Rigid Body Constraint**
Panda 2's grasp point:
```
T_goal_left = T_rod @ G_left
```

### **Frame Transformations**
Extensive use of SE(3) transforms for:
- robot bases  
- gripper tools  
- rod alignment  
- contact frames  

---

## 📸 Example Output

The simulation shows:
- Two Panda robots facing each other  
- A rod between their grippers  
- Panda 1 moving freely  
- Panda 2 smoothly adjusting to maintain grip  

(Insert your own screenshot/gif here!)

---

## 📚 Learning Outcomes

Through this project, you gain hands-on experience with:
- Multi-robot coordination  
- Object-centric motion control  
- Real-time IK updates  
- Spatial algebra & transformation pipelines  
- Robotics Toolbox + Swift integration  

---

## 🧩 Future Improvements

- Add collision checking  
- Implement load-sharing and force constraints  
- Add task-space trajectories for coordinated movement  
- Introduce grasp compliance / soft contacts  

---

## 👤 Author
**Laiba Khan**  
CSC376 – Robotics  
University of Toronto  

