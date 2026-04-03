## 手动启动

如果你想自己逐个开终端，按下面顺序运行。

### 终端 1：启动模拟器

```bash
cd /home/uestc/XZC/RMUA/IntelligentUAVChampionshipBase/simulator12.0.0.3
./run_simulator.sh 123
```

### 终端 2：启动队员控制器

```bash
cd /home/uestc/XZC/RMUA/IntelligentUAVChampionshipBase/basic_dev
sss
roslaunch controller pos_control_ego.launch 
```

### 终端 3：启动当前工程规划

```bash
cd /home/uestc/MTY/egoplanner_study
source devel/setup.bash
roslaunch ego_planner run_in_rmua.launch
```
rosservice call /airsim_node/drone_1/takeoff "{}"
## 运行中的关键话题

可以用下面几个话题确认链路是否正常：

```bash
rostopic echo -n 1 /rmua/odom
rostopic echo -n 1 /planning/pos_cmd
rostopic echo -n 1 /airsim_node/drone_1/vel_body_cmd
```

含义分别是：

1. `/rmua/odom`：规划使用的无人机状态
2. `/planning/pos_cmd`：EGO 输出的位置控制指令
3. `/airsim_node/drone_1/vel_body_cmd`：控制器发给模拟器的速度指令

