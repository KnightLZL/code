/**
 * @file collision_detection.h
 * @author ZLLee
 * @brief 碰撞检测
 * @version 0.1
 * @date 2023-07-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include "../point_types.h"
#include "../reference_line/reference_line.h"
#include "common.h"


namespace lane_follow_pnc
{
    class Obstacle
    {
        public:
        FrenetPoint point;
        double x_rad;           //x方向的障碍物半径
        double y_rad;           //y方向的障碍物半径
        std::vector<FrenetPoint> collision_box;   //碰撞的box，用八个点(四个角，加四个边中点)   
    };

    class CollisionDetection
    {
        public:

        double collision_distance; //设置的碰撞距离，认为距离小于这个就是碰撞上
        std::vector<Obstacle> detected_objects;
        std::vector<Obstacle> static_obstacle_list;
        std::vector<Obstacle> dynamic_obstacle_list;
        std::vector<PathPoint> ref_path; //参考线
        PathPoint host_match_point;   //此次车辆定位的匹配点位置
        int pre_match_index;    //前一次的匹配点的位置

        CollisionDetection() = default;     //采用默认的构造函数

        // 构造函数重载
        CollisionDetection(const std::vector<Obstacle> &detected_objects,
                            const double &collision_distance,
                            const std::vector<PathPoint> &ref_path,
                            PathPoint host_match_point,
                            const int pre_match_index);

        // 构造函数继续重载
        // CollisionDetection(const std::vector<Obstacle> &detected_objects,
        //             const double &collision_distance,
        //             const std::vector<PathPoint> &ref_path,
        //             const int host_match_index);

        // 对动态还是静态障碍物分类
        void obstacle_classfication(std::vector<Obstacle> &detected_objects);

        // 计算碰撞区域
        void cal_collision_box(Obstacle &object);

        // //检查是否碰撞并计算碰撞代价
        // bool check_collison(FrenetPath & path, 
        //                     const FrenetPoint &leader_point,
        //                     const bool &car_following);
    };
}//lane_follow_pnc

#endif