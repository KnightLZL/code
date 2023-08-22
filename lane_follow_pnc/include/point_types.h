/**
 * @file point_types.h
 * @author ZLLee
 * @brief  定义一些路径点信息
 * @version 0.1
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
 *
 */
//防止包含的头文件重复
#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace lane_follow_pnc
{
    // 全局坐标系下的路径点
    class PathPoint
    {
        public:
        double x;
        double y;
        double z;
        double heading; //航向角
        double kappa;  //曲率
        double s_;  //弧长
    };

    // 车辆定位信息
    class CarState :public PathPoint
    {
        public:
        double vx;  
        double vy;
        double v;

        double ax;
        double ay;
        double a;

        double t;  //时间戳
    };

    //增加fernet坐标系下的信息
    class FrenetPoint : public CarState
    {
        public:
        double s;
        double l;

        //对时间求导
        double s_d;
        double l_d;
        double s_d_d;
        double l_d_d;

        //对形状求导
        double l_ds;
        double l_d_ds;
        double l_d_d_ds;
        double ds;  //用以计算曲率

        //dp path中用以计算cost和记录晦朔路径的信息
        double dp_cost;   //记录该点到起点的代价
        int  dp_pre_row;  //该点最小cost的上一个点的行号
    };

    class FrenetPath
    {
        public:
        double cost;  //当前路径段障碍物的碰撞代价
        std::vector<FrenetPoint> frenet_path;
        int size_ = 0; //用以记录有效点的个数

        double max_speed;
        double max_acc;
        double max_kappa; //最大曲率限制
    };

} // namespace lane_follow_pnc

#endif