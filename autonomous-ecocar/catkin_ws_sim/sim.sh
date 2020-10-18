#/bin/bash

source devel/setup.bash

# Setup perspective file
sed "s|/home/dynamo/autonomous-ecocar/catkin_ws_sim|$PWD|g" Ecocar_template.perspective > Ecocar.perspective

roscore &
sleep 0.5
rqt --perspective-file Ecocar.perspective
kill %1
