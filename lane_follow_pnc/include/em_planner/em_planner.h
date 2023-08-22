/**
 * @file em_planner.h
 * @author ZLLee
 * @brief 路径规划
 * @version 0.1
 * @date 2023-07-31
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef EM_PLANNER_H
#define EM_PLANNER_H

#include <sys/time.h>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>
#include <cmath>
#include <future>
#include <thread>

//参考线头文件
#include "../reference_line/reference_line.h"
// 三次样条曲线的参考线拟合
#include "../reference_line/cubic_spline.hpp"
//定义的点类型
#include "../point_types.h"
//五次多项式头文件
#include "../polynomial/quintc_polynomial.hpp"
//障碍物检测头文件
#include "../collision_detection/collision_detection.h" 

namespace lane_follow_pnc
{
    class EMPlanner
    {
    public:
        CollisionDetection collision_detection;  //检测到的障碍物信息
        double desired_speed = 8.0;          //目标车速信息

        EMPlanner() = default;

        EMPlanner(const CollisionDetection &collision_detection,
                  std::unordered_map<std::string, double> &dp_path_params,
                  std::unordered_map<std::string, double> &qp_path_params);

        /*********************DP Path相关************************/
        void dp_samping(const FrenetPoint &inital_point);
        double calc_dppath_cost(const FrenetPoint &start, const FrenetPoint &end);
        void calc_dp_path(const FrenetPoint &initial_point);
        void dp_path_interpolation(const std::vector<FrenetPoint> &dp_path);

        /**********************QP Path相关**************************/
        // 开辟凸空间
        void calc_convex_space(const std::vector<FrenetPoint> &dp_final_path);
        //计算障碍物到dp path上的匹配点
        int get_obstacle_index(const std::vector<FrenetPoint> &dp_final_path,
                               const double &ob_s);
        void calc_qp_path(const std::vector<FrenetPoint> &dp_final_path,
                          const Eigen::VectorXd &l_min,
                          const Eigen::VectorXd &l_max);

        /*****************************************************/
        void get_cartesian_paths(std::vector<FrenetPoint> &frenet_path, std::vector<FrenetPoint> &ref_path);

        int get_cartesian_paths(std::vector<FrenetPoint> &frenet_path, Spline2D &ref_frenet);

        FrenetPath planning(std::vector<FrenetPoint> &ref_frenet, 
                            const FrenetPoint &inital_point);

        FrenetPath planning(Spline2D &ref_frenet, const FrenetPoint &initial_point);

    // private:
        double path_ds = 0.1;  //路径规划采样点间隔,对dp规划后路径的增密间隔
        
        /*****************DP path相关*****************/
        double dp_sample_l;    //dp横向采样间隔
        double dp_sample_s;    //dp纵向采样间隔

        int dp_sample_rows;    //dp采样的行数
        int dp_sample_cols;    //dp采样的列数

        double dp_cost_collision; //碰撞代价
        double dp_cost_dl;
        double dp_cost_ddl;
        double dp_cost_dddl;
        double dp_cost_ref;
        std::vector<std::vector<FrenetPoint>> dp_sample_path;  //dp采样的路径
        std::vector<FrenetPoint> dp_path;  //dp规划后的路径
        std::vector<FrenetPoint> dp_final_path;  //插值增密后的dp_path

        /***********************QP Path相关***************************/
        //QP path的边界
        Eigen::VectorXd l_min;
        Eigen::VectorXd l_max;

        //QP path的cost权重
        double qp_cost_l;
        double qp_cost_dl;
        double qp_cost_ddl;
        double qp_cost_dddl;
        double qp_cost_ref;
        double qp_cost_end_l;
        double qp_cost_end_dl;
        double qp_cost_end_ddl;

        std::vector<FrenetPoint>  qp_path;   //二次规划求解后的path
    };

} //lane_follow_pnc

#endif  