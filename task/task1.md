Create a concise, implementation-ready migration plan to replace the current robot model in this workspace with the new model located in `/home/zmerrr/Documents/Yantravira/Yantravira-urdf1`.

### Constraints
- **Plan only:** Do not edit files, copy assets, or run builds yet.
- **Token-efficient discovery:** Never read binary/mesh files (`.stl`, `.dae`, `.obj`, `.png`) or scan `build/`, `install/`, `log/`, and `.git/`. Use targeted filename and text searches to inspect only directory layouts, `.urdf`/`.xacro` files, launch/spawn scripts, and YAML/config files.

### What to Audit
1. **New model (`/home/zmerrr/Documents/Yantravira/Yantravira-urdf1`):** Entry `.urdf`/`.xacro` file, package/mesh URI format (`package://` vs. relative paths), root/base link name, joint names and types, and any simulator/control tags or sensors.
2. **Current workspace:** All files referencing the existing robot model—description packages, launch files, simulator configs, RViz, controllers/`ros2_control`, kinematics/MoveIt, and hardcoded TF frame names.

### Required Output Format
Keep the plan tight and skip general advice. Include only:
1. **Model Diff Table:** Current robot vs. Yantra (`/home/zmerrr/Documents/Yantravira/Yantravira-urdf1`) across entry file, base frame, joint names, mesh URI style, and plugins/sensors.
2. **File Change Manifest:** Exact file paths to create, modify, or remove, with a 1-line change description for each.
3. **Risks & Mismatches:** Joint/link renames, broken mesh paths, missing transmission/control tags, or TF frame conflicts that need decisions before editing.
4. **Execution Order:** Numbered steps in dependency order.
5. **Verification Commands:** Exact commands to validate XML/kinematics, rebuild, and confirm the model spawns cleanly.
