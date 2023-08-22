/**
 * @file collision_detection.cpp
 * @author ZLLee
 * @brief  
 * @version 0.1
 * @date 2023-07-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../../include/collision_detection/collision_detection.h"
using namespace std;

namespace lane_follow_pnc
{

    // 障碍物检测的构造函数
    CollisionDetection::CollisionDetection(const std::vector<Obstacle> &detected_objects,
                                           const double &collision_distance,
                                           const std::vector<PathPoint> &ref_path,
                                           PathPoint host_match_point,
                                           const int pre_match_index)
    {
        this->detected_objects = detected_objects;
        this->collision_distance = collision_distance;
        this->ref_path = ref_path;
        this->host_match_point = host_match_point;
        this->pre_match_index = pre_match_index;

        // 保证静态和动态障碍物列表是空的
        static_obstacle_list.clear();
        dynamic_obstacle_list.clear();
        //障碍物分类传入的是检测到的障碍物
        obstacle_classfication(this->detected_objects);
        cout<<"障碍物分类成功"<<endl;
    }            


    /**
     * @brief 实现障碍物分类，区分动态和静态
    **/
    // 这里不用const，因为子中的函数 cal_collision_box(obstacle)会修改etected_objects
    void CollisionDetection::obstacle_classfication(std::vector<Obstacle> &detected_objects)
    {
        // 从容器中遍历每个对象
        for(Obstacle &obstacle : detected_objects) //从detected_objects中每次获取变量
        {
            //计算碰撞区域
            cal_collision_box(obstacle);
            cout<<"获取碰撞区域成功"<<endl;

            if(obstacle.point.v > 0.2)
            {
                //将动态障碍物加到动态障碍物列表中
                cout<<"动态障碍物已加到动态障碍物列表中"<<endl;
                dynamic_obstacle_list.push_back(obstacle);
            }
            else
            {
                //将静态障碍物投影到sl图中，并进行存储
                obstacle.point = calcFrenet(obstacle.point, ref_path,pre_match_index, host_match_point);
                cout<<"The static obstacle s and l :"<<obstacle.point.s<<"\t"<<obstacle.point.l<<endl;
                cout<<"The static obstacle x and y :"<<obstacle.point.x<<"\t"<<obstacle.point.y<<endl;

                for(auto &box_point : obstacle.collision_box)
                {
                    //将障碍物的八个点都投影到sl图上
                    box_point = calcFrenet(box_point,ref_path,pre_match_index, host_match_point);
                }

                static_obstacle_list.push_back(obstacle);
            }
        }

    }


    /**
     * @brief 这里是通过矩阵旋转，将障碍物的中心坐标，变为八个点的BOX
    */
    // 获取碰撞的box，采用八个点表示形状,不加const是因为要对obstacle添加collision_box
    void CollisionDetection::cal_collision_box(Obstacle &obstacle)
    {
        vector<FrenetPoint> collision_box(8);
        double x = obstacle.point.x;
        double y = obstacle.point.y;
        double heading = obstacle.point.heading;
        double x_rad = obstacle.x_rad;
        double y_rad = obstacle.y_rad;

        // 获取BOx边上8个点的坐标矩阵
        Eigen::MatrixXd position_matrix(8, 2), translation_matrix(8, 2), rotation_matrix(2, 2);

        // 位置矩阵
        position_matrix <<x, y,
                        x, y,
                        x, y,
                        x, y,
                        x, y,
                        x, y,
                        x, y,
                        x, y;

        // 定义一个非正交矩阵作为平移变换矩阵
        translation_matrix << -x_rad, -y_rad,
                              -x_rad, 0,
                              -x_rad, y_rad,
                              0, y_rad,
                              x_rad, y_rad,
                              x_rad, 0,
                              x_rad, -y_rad,
                              0, -y_rad;

        // 定义一组正交矩阵作为旋转矩阵
        rotation_matrix<< cos(heading), sin(heading),
                          -sin(heading), cos(heading);

        // 计算变换后的八个点坐标
        position_matrix = translation_matrix * rotation_matrix + position_matrix;

        for(int i =0; i<position_matrix.rows();++i)
        {
            // 包含的一些障碍物信息,静态用的
            collision_box[i].x = position_matrix(i,0);
            collision_box[i].y = position_matrix(i, 1);

            // 其他量保持和中心点一样，动态障碍物用来计算的一些信息
            collision_box[i].heading = obstacle.point.heading;
            collision_box[i].z = obstacle.point.z;
            collision_box[i].v = obstacle.point.v;
            collision_box[i].vx = obstacle.point.vx;
            collision_box[i].vy = obstacle.point.vy;
            collision_box[i].ax = obstacle.point.ax;
            collision_box[i].ay = obstacle.point.ay;
            collision_box[i].a = obstacle.point.a;

            // 将障碍物box信息push进去
            obstacle.collision_box.push_back(collision_box[i]);
        }
    }


    // // 碰撞检测并计算碰撞代价
    // /**
    //  * @brief 这里或许可以不用考虑跟车和前车的位置，就简单的假设我是一个车和静态障碍物
    // */
    // // path不加const是因为要对path中的cost修改
    // bool CollisionDetection::check_collison(FrenetPath &path, 
    //                         const FrenetPoint &leader_point,
    //                         cons0t bool &car_following)
    // {
    //     // 遍历每个障碍物
    //     for(Obstacle obstacle :detected_objects)
    //     {
    //         // 遍历每一个点
    //         for(auto box_point : obstacle.collision_box)
    //         {
    //             // 遍历路径上的每一个点
    //             for(int i=0; i < path.size_; ++i)
    //             {
    //                 double dist = calDistance(path.frenet_path[i].x, path.frenet_path[i].y,
    //                                 box_point.x, box_point.y);
                    
    //                 // 计算碰撞的cost（不计算跟车目标的碰撞cost）
    //                 if(dist < 3.5 && !(car_following)&&
    //                 calDistance(obstacle.point.x, obstacle.point.y, leader_point.x, leader_point.y) < 2.0)
    //                 {
    //                     path.cost += 3.0 / dist;
    //                 }

    //                 if(dist <= collision_distance)
    //                 {
    //                     return false;
    //                 }
    //             }
    //         }
    //     }
    //     return true;
    // }

} //lane_follow_pnc

