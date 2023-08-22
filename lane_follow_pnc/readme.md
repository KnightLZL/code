运动规划算法
======
lane follow 场景下的pnc
--------

# 功能
实现EM Planner的参考线平滑，路径规划
![image](https://github.com/KnightLZL/huayu/blob/master/lane_follow_pnc/src/data/qp_path.jpg)

## 环境要求

ubuntu18.04

Cmake 3.10

Eigen3

### 使用步骤

cd lane_follow_pnc/

mkdir build/

cd build/

cmake ..

make

cd ..

cd bin/

./demo1

### 补充
demo为参考线平滑，demo1为path规划



