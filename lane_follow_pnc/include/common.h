/**
 * @file common.h
 * @author ZLLee
 * @brief   planning模块中通用的计算函数
 * @version 0.1
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
 *
 */



#include <iostream>
// 定义了操作字符串的类，可以方便的进行字符串和其他数据类型的转换
#include <sstream>
// 包含了格式化输入输出的一些工具
#include <iomanip>
#include <vector>
#include <algorithm>
// 提供了对浮点数特性和操作的定义  
#include <cfloat>
#include <cmath>
#include <Eigen/Eigen>
#include "point_types.h"
#include "OsqpEigen/OsqpEigen.h"


namespace lane_follow_pnc
{
    /*辅助函数*/

    //计算两点间的距离
    double calDistance(double x1,double y1,double x2, double y2);

    //寻找匹配点
    int searchMatchIndex(const double &cur_x, const double &cur_y,
                            const std::vector<PathPoint> &way_points,
                            const int &pre_match_index );

    // 匹配点到投影点
    PathPoint match2Projection(const CarState &cur_pose,  
                                    const PathPoint &match_point);

    // 计算航向角和曲率
    void calHeading(std::vector<PathPoint> &waypoints);

    //笛卡尔坐标系转fernet坐标系
    FrenetPoint Cartesain2Frenet( const CarState &global_point,
                                const PathPoint &projection_point);

    FrenetPoint calcFrenet(const CarState &global_point,
                                const std::vector<PathPoint> &ref_path,
                                const int pre_match_index,
                                PathPoint & host_point);

    //归一化角度
    double NormalizeAngle(const double angle);

    //计算frenet坐标系的s，确保每次的frenet坐标系的原点都是车辆定位点的匹配点 
    void  cal_s( FrenetPoint &point,  PathPoint & host_point);

    // frenet转cartesian
    void Frenet2Cartesian( const double rs, const double rx, const double ry, const double rtheta,
                                const double rkappa, const double rdkappa,
                                const std::array<double, 3>& s_condition,
                                const std::array<double, 3>& d_condition, double* const ptr_x,
                                double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
                                double* const ptr_v, double* const ptr_a); 

}//lane_follow_pnc

