#!/usr/bin/env zsh

source /home/fw05/.zshrc

echo "Launching Application, please wait!"

gnome-terminal --geometry=40x40  \
--tab --title="roscore" -e "zsh -c \"pm2 restart 1;exec zsh\"" \
--tab --title="init" -e "zsh -c  \"sleep 2s;roslaunch fw_rev_04_init fw_robot.launch;exec zsh\"" \
--tab --title="cam" -e "zsh -c \"sleep 5s;roslaunch fw_rev_04_cam launch_d455_ekf_wheel_imu_jay.launch initial_reset:=false;exec zsh\"" \
--tab --title="slam" -e "zsh -c \"sleep 5s;roslaunch fw_rev_04_rtabmap fw_navigation_stereo_test2_jay_2dlidar.launch rviz:=true;exec zsh\"" \
--tab --title="nav" -e "zsh -c \"sleep 5s;roslaunch fw_rev_04_navigation fw_move_base_teb.launch;exec zsh\""
