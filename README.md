drone_control

cd catkin_ws/src

git clone https://github.com/Goohuram/drone_control.git

cd ..

catkin_make


Programmer un PID pour régulerla hauteur du drone

rosrun drone_control takeoff.py

rosservice call /altitude_service "a: data: '10'"

Pour changer donner un nouveau waypoint

rosrun drone_control waypoint.py

rosservice call /waypoint_service "x:

data: '-5'

y:

data: '-5'

z:

data: '10'"


Planificationde mission

rosrun drone_control Final_project.py
