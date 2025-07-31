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