# joint_jogger_gui
RViz2 panel to jog joints with **◄/►** buttons, publishing `std_msgs/msg/Float64MultiArray` **velocity** commands to a `ros2_control` **JointGroupVelocityController**.

---

## Features (new)
- **ON/OFF toggle button**  
  - Green = publishing enabled  
  - Red = publishing disabled  
- **Fixed-length command**: always publishes exactly **N** joint velocities (default **7**), as required by velocity group controllers.
- **Per-joint step**: each row has its own step. Defaults:  
  - `Joint_0..2` → **0.05**  
  - `Joint_3..N-1` → **0.2**
- **Triangle buttons** (◄/►) with **hold-to-jog**; release sets that joint’s velocity to **0.0**.
- **Configurable**: topic and publish rate (Hz) are editable from the panel.
- **Safe by default**: publishing is OFF when the panel starts, preventing interference with other controllers.

---

## ⚠️ Important Safety Disclaimer
This tool directly commands **robot joint velocities**, which can cause robot motion.

- Always verify that the robot workspace is **clear of people, tools, and obstacles**.  
- Start with the robot in a **safe position** and with **low step values**.  
- Especially on first use, apply extreme caution and observe the robot closely.  
- Ensure you have an accessible **hardware emergency stop (E-Stop)**.  
- Use this plugin **at your own risk**.  
- The author of this plugin take **no responsibility** for any damage, injury, or malfunction caused by its use.

By using this plugin, you acknowledge that **you are fully responsible** for ensuring safe operation of the robot and environment.

---

## Usage
1. Start your `ros2_control` **JointGroupVelocityController**.
2. Launch **RViz2**, then **Panels → Add New Panel → joint_jogger_gui/JointJoggerPanel**.
3. Set **Topic** (default: `/velocity_controller/commands`).
4. Set **Rate (Hz)** if needed (default: **50.0**).
5. Set **Joints** count **N** (default: **7**). Row names are cosmetic (`Joint_i`); the controller uses **array order only**.
6. (Optional) Adjust each row’s **Step** value.
7. Press the **ON/OFF toggle**:
   - **ON (green)**: panel publishes velocity commands at the configured rate  
   - **OFF (red)**: publishing stops and a final zero command is sent

8. **Hold** ◄ or ► to jog a joint; **release** to stop.  
   Use **Stop All** to immediately set all velocities to zero.

> The controller requires exactly **N** values in the array; this panel always publishes an array of length **N** when enabled.

---

## Message format
```text
std_msgs/msg/Float64MultiArray
# data[i] is the velocity for Joint_i
```

---

## Build (colcon)
```bash
colcon build --packages-select joint_jogger_gui --symlink-install
source install/setup.bash
rviz2
```
Then add the panel as described above.

---

## Important: proper panel removal
To properly delete the panel it is unfortunately not enough to click on the top-right red cross (that only hides it). You must:

1. Navigate to the **Panels** menu in RViz2.  
2. Select **Delete Panel**.  
3. Choose **JointJoggerPanel**.  
4. If there are multiple instances, repeat and delete all of them.

---
