/**
 * @file reference_line.h
 * @author ZLLee
 * @brief  参考线头文件
 * @version 0.1
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef REFERENCELINE_H
#define ReFERENCELINE_H

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"
#include "../point_types.h"
#include "reference_line/cubic_spline.hpp"
#include "../common.h"


namespace lane_follow_pnc
{
class ReferenceLine
{

    public:
    double lookahead_dist;
    int match_index;
    //构造函数
    ReferenceLine(double lookahead_distance):lookahead_dist(lookahead_distance),match_index(0) {}
    //通过函数重载，添加另一个获得两个参数的参考线构造
    ReferenceLine(double lookahead_distance, 
                std::unordered_map<std::string,double> &referline_params);
    //寻找路径最远点
    int search_target_index(const double &cur_x, const double &cur_y,
                const std::vector<PathPoint> &waypoints,
                const double &lookahead_distance);
    //截取路径
    std::vector<PathPoint> local_path_truncation(const CarState &cur_pose,
                                                    const std::vector<PathPoint> &global_path,
                                                    const int &pre_match_index);
    
    // cubic spiline平滑
    std::vector<PathPoint> smoothing(Spline2D &ref_frenet,
                                        const std::vector<PathPoint> &local_path );
    
    //离散点平滑
    std::vector<PathPoint> discrete_smooth(const std::vector<PathPoint> &local_path);

    //离散点平滑的二次规划求解
    void discrete_points_osqp(std::vector<std::pair<double,double>> &path_point2d);
    
    // void cal_heading(std::vector<PathPoint> &waypoints);

    private:
    //平滑参数
    double ref_weight_smooth;       //平滑代价
    double ref_weight_path_length;  //紧凑代价
    double ref_weight_ref_deviation; //形状相似代价

    //二次规划的凸空间约束
    double x_lower_bound;
    double x_upper_bound;
    double y_lower_bound;
    double y_upper_bound;
};
} //namespace lane_follow_pnc

#endif