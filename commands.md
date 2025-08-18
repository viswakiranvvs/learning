# Commands

## Open isaac sim in streaming

```
isaacsim isaacsim.exp.full.streaming --no-window --streaming
```

## training

navigate to folder with isaaclab.sh file

```
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --headless --task=Isaac-Ant-v0 --video --max_iterations=20
```

## run python file

navigate to folder with isaaclab.sh file

```
./isaaclab.sh -p source/learning/learning/simulation/bipad.py --livestream=2
```

## Rtab

```
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera_rgb    depth_topic:=/camera_depth     camera_info_topic:=/camera_camera_info     frame_id:=Camera_OmniVision_OV9782_Color     approx_sync:=true     approx_sync_max_interval:=0.2     topic_queue_size:=30     sync_queue_size:=30 rtabmap_args:="--delete_db_on_start --Odom/Strategy=0"
```