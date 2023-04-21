# Simple-LIO

## Done

- [x] 初始化零偏和P阵
- [x] IMU前向机械编排和噪声传播
- [x] 点云运动补偿

- [x] 迭代卡尔曼更新
  - [x] 构造观测矩阵
  - [x] 计算卡尔曼增益
  - [x] 计算误差状态
  - [x] 更新系统状态
- [x] 地图更新(使用ikd-tree)

## Todo

- [x] 使用加速度计初始化横滚角和俯仰角
- [ ] 优化重力向量

## Result

r3live数据集中`hku_main_building`的结果：

|     ![simple-lio](img/simple-lio.png)     |      ![fast-lio](img/fast-lio.png)       |
| :---------------------------------------: | :--------------------------------------: |
|                   this                    |                 fast-lio                 |
| 结束时位置：0.0722277 0.0584535 -0.676519 | 结束时位置：0.0838183 0.0721238 -1.56504 |

## How to use

1. 下载

```bash
git clone https://gitee.com/ghowoght/simple_lio
```

2. 更新依赖

```bash
cd simple_lio
git submodule update --init --recursive
```

3. 编译

```bash
catkin_make
```

4. 运行，需要自己准备数据集，输入为imu和点云数据，在config.yaml中配置话题名，点云格式为`livox_ros_driver::CustomMsg`

```bash
roslaunch simple_lio simple_lio.launch
```

​	**注：**也可以使用`sensor_msgs::PointCloud2`格式的点云，但是消息名固定为`/livox/lidar/sensor_pointcloud2`
