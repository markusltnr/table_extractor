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
tmux send-keys "source /home/v4r/catkin_ws/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch --wait mongodb_store mongodb_store.launch db_path:=/home/v4r/mongo_db" C-m
tmux select-pane -t 1
tmux send-keys "rviz" C-m
tmux select-pane -t 2
tmux send-keys "source /home/v4r/catkin_ws/devel/setup.bash"
tmux send-keys enter
tmux send-keys "cd /home/v4r/catkin_ws/src/table_extractor/scripts/" C-m
tmux send-keys "python table_extractor_script.py"
tmux select-pane -t 3
tmux send-keys "source /home/v4r/catkin_ws/devel/setup.bash"
tmux send-keys enter
tmux send-keys "cd /home/v4r/catkin_ws/src/table_extractor/scripts/" C-m
tmux send-keys "python read_rosbag.py"
# Attach to session
tmux -2 attach-session -t $SESSION


