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
                                const std::vector<PathPoint> &ref_path);

}//lane_follow_pnc

