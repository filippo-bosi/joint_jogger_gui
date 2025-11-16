# joint_jogger_gui
RViz2 panel to jog joints with **◄/►** buttons, publishing `std_msgs/msg/Float64MultiArray` **velocity** commands to a `ros2_control` **JointGroupVelocityController**.

---

## Features (new)
- **Fixed-length command**: always publishes exactly **N** joint velocities (default **7**), as required by velocity group controllers.
- **Per‑joint step**: each row has its own step. Defaults:
  - `Joint_0..2` → **0.05**
  - `Joint_3..N-1` → **0.2**
- **Triangle buttons** (◄/►) with **hold‑to‑jog**; release sets that joint’s velocity to **0.0**.
- **Configurable**: topic and publish rate (Hz) are editable from the panel.

---

## Usage
1. Start your `ros2_control` **JointGroupVelocityController**.
2. Launch **RViz2**, then **Panels → Add New Panel → joint_jogger_gui/JointJoggerPanel**.
3. Set **Topic** (default: `/velocity_controller/commands`).
4. Set **Rate (Hz)** if needed (default: **50.0**).
5. Set **Joints** count **N** (default: **7**). Row names are cosmetic (`Joint_i`); the controller uses **array order only**.
6. (Optional) Adjust each row’s **Step**.
7. **Hold** ◄ or ► to jog a joint; **release** to stop. Use **Stop All** to zero all velocities immediately.

> The controller will complain if it doesn’t receive exactly **N** values; this panel guarantees an array of length **N** every tick.

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

## Important: panel interference and proper removal

Sometimes this panel can create unwanted interference with other velocity control commands sent to the robot, because it keeps publishing velocity messages as long as it exists.  
To properly delete the panel it is unfortunately not enough to click on the top-right red cross (that only hides it). You must:

1. Go to the **Panels** menu in RViz2.  
2. Select **Delete Panel**.  
3. Choose **JointJoggerPanel**.  
4. If there are multiple instances, repeat and delete all of them.

Only after deleting all JointJoggerPanel instances will the panel stop publishing and no longer interfere with other controllers.

---
