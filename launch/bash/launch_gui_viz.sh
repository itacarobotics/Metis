#!/bin/bash

# Open terminal 1 and run gepetto-gui
# gnome-terminal -- bash -c '/opt/openrobots/bin/gepetto-gui; exec bash'


# Open terminal 2, navigate to the specified directory, and launch ros2
gnome-terminal -- bash -c '
    export PATH=/opt/openrobots/bin:$PATH &&
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH &&
    export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH &&
    export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH &&
    export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH &&

    source /opt/ros/humble/setup.bash &&
    source /home/ostifede02/dr_ws/install/setup.bash &&
    
    cd dr_ws/launch &&
    ros2 launch deltarobot_app.launch.py; exec bash'

