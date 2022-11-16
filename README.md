# Simple-LIO

## Done

- [x] 初始化零偏和P阵
- [x] IMU前向机械编排和噪声传播
- [x] 点云特征提取
- [x] 点云运动补偿

- [x] 迭代卡尔曼更新
  - [x] 构造观测矩阵(目前只使用了平面特征)
  - [x] 计算卡尔曼增益
  - [x] 计算误差状态
  - [x] 更新系统状态
- [x] 地图更新(目前这部分很简陋，也最耗时，其中降采样耗时占2/3)

## Todo

- [ ] 使用加速度计初始化横滚角和俯仰角
- [ ] 优化重力向量
- [x] 更改地图更新策略

## Result

| ![simple-lio](./img/simple-lio.png) | ![fast-lio](./img/fast-lio.png) |
| :---------------------------------: | :-----------------------------: |
|                this                 |            fast-lio             |

## How to use

下载

```bash
git clone https://gitee.com/ghowoght/simple_lio
```

更新依赖

```bash
cd simple_lio
git submodule update --init --recursive
```

编译

```bash
catkin_make
```

运行

```bash
roslaunch simple_lio simple_lio.launch
```
