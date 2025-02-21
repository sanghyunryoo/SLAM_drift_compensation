# SLAM_drift_compensation
## Drift compensation using IMU 
### base_footprint -> base_link update

ros2 build

1) build:
colcon build --symlink install

2) execute:
ros2 launch slam_tf_broadcaster slam_tf_broadcaster.launch.py use_sim_time:=True
