## PROJECT OVERVIEW
The robot implementation file structure
```bash
├── krawl
│   ├── bullet_map.py
│   ├── camera.py
│   ├── gripper_logic.py
│   ├── krawl
│   │   ├── meshes
│   │   └── urdf
│   │       ├── inertial_macros.xacro
│   │       └── krawl.urdf
│   ├── lidar.py
│   ├── main.py
│   ├── non_classed.py
│   ├── pyproject.toml
│   ├── requirements.txt
│   ├── robot_povs.py
│   ├── sim_camera_lidar.png
│   ├── uv.lock
│   └── wheel_logic.py
└── README.md
```
in the structure above
the main.py is the entrance to the whole implementation. 
the wheel_logic.py implements movement of the robot in the simulation environment
the gripper_logic.py implements the movement of the robot's gripper
the lidar.py and bullet_map.py implement a lidar system for object detection
the camera.py and the robot_povs.py implement a visualization system for the robot

## GETTING STARTED 
To build the robot implementation, run

```bash
cd krawl
```
change directory to the krawl folder

```python
uv venv
```
to create a virtual environment to hold your site packages
```python
source .venv/bin/activate
```
activate your environment

```python
uv pip install requirements.txt
```
for environments using uv
or
```python
pip install requirements.txt
```
This installs the required site packages.

To launch the robot
```python
uv run main.py

```
