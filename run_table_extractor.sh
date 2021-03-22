#!/bin/bash

SESSION=TABLE_EXTRACTOR



tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:0 

tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "docker run -it --rm --net=host primitect_ros" C-m
tmux send-keys enter
tmux select-pane -t 0
tmux send-keys "export ROS_MASTER_URI=http://10.0.0.143:11311"
tmux send-keys enter
tmux send-keys "source /home/v4r/Markus_L/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch primitect_ros detect_objects.launch" C-m
tmux select-pane -t 1
tmux send-keys "hsrb_mode" 
tmux send-keys enter
tmux send-keys "source /home/v4r/Markus_L/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch --wait mongodb_store mongodb_store.launch db_path:=/home/v4r/Markus_L/mongo_database/table_store" C-m
tmux select-pane -t 2
tmux send-keys "hsrb_mode"
tmux send-keys enter
tmux send-keys "rviz" C-m
tmux select-pane -t 3
tmux send-keys "hsrb_mode"
tmux send-keys enter
tmux send-keys "source /home/v4r/Markus_L/devel/setup.bash"
tmux send-keys enter
tmux send-keys "cd /home/v4r/Markus_L/src/table_extractor/scripts/" C-m
tmux send-keys "python table_extractor.py"
# Attach to session
tmux -2 attach-session -t $SESSION


