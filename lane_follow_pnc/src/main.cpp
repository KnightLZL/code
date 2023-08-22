#include <iostream>
#include <fstream>
#include <iomanip>
#include "../include/reference_line/reference_line.h"
#include "../include/point_types.h"
#include "../include/collision_detection/collision_detection.h"
#include "../include/em_planner/em_planner.h"
using namespace std;
using namespace lane_follow_pnc;
#define N 2264

int main()
{
    // 1.获取参考线信息
     // 读取文件
    // ifstream fin("../src/data/CubeTown.txt",ios::in);
    ifstream fin("../src/data/CubeTown1.txt",ios::in);

    if(!fin.is_open())
    {
        cout<<"open error"<<endl;
        exit(0);
    }

    int i =0;
    vector<double> v1;
    while(!fin.eof())
    {
        double temp;
        fin>>temp;
        v1.push_back(temp);
        i++;
    }

    int n = v1.size();
    cout<<"导航信息的坐标点个数为"<<n/2<<endl;
    vector<double> host_x;
    vector<double> host_y;

    for(size_t i =0; i<v1.size(); i++)
    {
        // cout<<v1[i]<<endl;
        if( i % 2 == 0)
        {
            host_x.push_back(v1[i]);
        }
        else
        {
            host_y.push_back(v1[i]);
        }
    }

    //2.定义参考线平滑参数
    double path_length = 50.0;

    std::unordered_map<std::string, double> reference_params = {{"ref_weight_smooth", 50.0},
                    {"ref_weight_path_length", 10.0}, {"ref_weight_offset", 20.0},
                     {"x_lower_bound", -0.1}, {"x_upper_bound", 0.1},
                     {"y_lower_bound", -0.1}, {"y_upper_bound", 0.1}};
    
    // cout<<reference_params["x_lower_bound"]<<"\t"<<reference_params["x_upper_bound"]<<endl;
    // cout<<reference_params["y_lower_bound"]<<"\t"<<reference_params["y_upper_bound"]<<endl;

    // 创建参考线对象
    lane_follow_pnc::ReferenceLine reference_line(path_length, reference_params);
    // 定义匹配点坐标
    int pre_match_index = 0;

    int start_index = pre_match_index;
    // 2.11从全局路径获取局部路径做参考线平滑
    CarState cur_pose;
    int host_index = 0;
    cur_pose.x = host_x[host_index];
    cur_pose.y = host_y[host_index];
    
    std::vector<PathPoint> global_path;
    for (size_t i =0; i<host_x.size()-1; ++i)
    {
        PathPoint temp;
        temp.x = host_x[i];
        temp.y = host_y[i];
        temp.s_ = i;
        global_path.push_back(temp);
        // cout<<global_path[i].x<<" "<<global_path[i].y<<" "<<global_path[i].heading<<endl;
    }

    std::vector<PathPoint> local_path = reference_line.local_path_truncation(cur_pose, global_path, start_index);
    // for(int i =0; i<local_path.size(); ++i)
    // {
    //     cout<<local_path[i].x<<" "<<local_path[i].y<<" "<<local_path[i].heading<<endl;
    // }
    cout<<"上一次的匹配点下标为："<<pre_match_index<<endl;

    if(local_path.size() > 1)
    {
        // 2.2离散点平滑
        std::vector<PathPoint> ref_path = reference_line.discrete_smooth(local_path);

        // 写入数据
        ofstream outfile;
        outfile.open("../src/data/smooth.txt", ios::trunc);
        if(!outfile.is_open())
        {
            cout<<"smooth open error"<<endl;
        }
        // 写入数据
        for(size_t i=0; i < ref_path.size(); i++)
        {
            outfile<<ref_path[i].x<<"\t"<<ref_path[i].y<<endl;
        }
        cout<<"the size of ref_path is: "<<ref_path.size()<<endl;
        cout<<"参考线数据写入成功"<<endl;
        outfile.close();    

        // // 输出查看下平滑后的参考线信息
        // for(size_t i=0; i<ref_path.size(); ++i)
        // {
        //     cout<<ref_path[i].x<<"\t"<<ref_path[i].y<<"\t"<<ref_path[i].heading<<"\t"<<ref_path[i].kappa<<endl;
        // }   
        cout<<"参考线平滑完毕"<<endl;
    

        // 3.设置障碍物,
        // 创建障碍物对象，用以存储所有的障碍物
        vector<Obstacle> obs;

        // 添加障碍物
        // 设置基本位置和大小信息
        Obstacle obs1;
        obs1.point.x = 15.0;
        obs1.point.y = 2;
        obs1.x_rad = 1;
        obs1.y_rad = 1;

        Obstacle obs2;
        obs2.point.x = 10.0;
        obs2.point.y = 3;
        obs2.x_rad = 1;
        obs2.y_rad = 1;
        obs2.point.v = 2;

        obs.push_back(obs1);
        // obs.push_back(obs2);


        // 4.进行障碍物碰撞检测
        double distance_check = 0.5;
        // 调用函数的时候只需要传入实参就行，不需要加入类型
        /**
         * 传入参数是障碍物、碰撞检测阈值、平滑后的参考线、车辆的匹配点、前一次的参考线匹配点
        */
        lane_follow_pnc::CollisionDetection collision_detection(obs, distance_check, local_path,global_path[host_index],pre_match_index);

        // 输出动静态障碍物的分类后信息
        cout<<"输出分类后的障碍物信息"<<endl;

        cout<<"静态障碍物的坐标是: "<<"x="<<collision_detection.static_obstacle_list.back().point.x<<"\t"
                                <<"y="<<collision_detection.static_obstacle_list.back().point.y<<endl;

        // cout<<"动态障碍物的坐标是: "<<"x="<<collision_detection.dynamic_obstacle_list.back().point.x<<"\t"
        //                         <<"y="<<collision_detection.dynamic_obstacle_list.back().point.y<<endl;

        
        // 输出静态障碍物的box
        cout<<"输出静态障碍物八个点组成的box"<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[0].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[0].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[0].s<<"\t"
        // <<collision_detection.static_obstacle_list.back().collision_box[0].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[1].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[1].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[1].s<<"\t"
        // <<collision_detection.static_obstacle_list.back().collision_box[1].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[2].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[2].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[2].s<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[2].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[3].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[3].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[3].s<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[3].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[4].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[4].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[4].s<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[4].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[5].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[5].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[5].s<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[5].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[6].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[6].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[6].s<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[6].l<<endl;

        // cout<<collision_detection.static_obstacle_list.back().collision_box[7].x<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[7].y<<endl;
        // cout<<collision_detection.static_obstacle_list.back().collision_box[7].s<<"\t"
        //     <<collision_detection.static_obstacle_list.back().collision_box[7].l<<endl;

        cout<<"障碍物检测完毕"<<endl;

        // 5.进行dp决策
        // 创建EM Planner
        // EMPlanner(const CollisionDetection &collision_detection,
        //               std::unordered_map<std::string, double> &dp_path_params,
        //               std::unordered_map<std::string, double> &qp_path_params);
            //     this->dp_sample_l = dp_path_params["dp_sample_l"];
            // this->dp_sample_s = dp_path_params["dp_sample_s"];
            // this->dp_sample_rows = static_cast<int>(dp_path_params["dp_sample_rows"]);
            // this->dp_sample_cols = static_cast<int>(dp_path_params["dp_sample_cols"]);
            
            // this->dp_cost_collision = dp_path_params["dp_cost_collision"];
            // this->dp_cost_dl = dp_path_params["dp_cost_dl"];
            // this->dp_cost_ddl = dp_path_params["dp_cost_ddl"];
            // this->dp_cost_dddl = dp_path_params["dp_cost_dddl"];
            // this->dp_cost_ref = dp_path_params["dp_cost_ref"];

            // // QP path params
            // this->qp_cost_l = qp_path_params["qp_cost_l"];
            // this->qp_cost_dl = qp_path_params["qp_cost_dl"];
            // this->qp_cost_ddl = qp_path_params["qp_cost_ddl"];
            // this->qp_cost_dddl = qp_path_params["qp_cost_dddl"];
            // this->qp_cost_ref = qp_path_params["qp_cost_ref"];
            // this->qp_cost_end_l = qp_path_params["qp_cost_end_l"];
            // this->qp_cost_end_dl = qp_path_params["qp_cost_end_dl"];
            // this->qp_cost_end_ddl = qp_path_params["qp_cost_end_ddl"];
        std::unordered_map<std::string, double> dp_path_params = {{"dp_sample_l", 1.0},
                        {"dp_sample_s", 5.0}, {"dp_sample_rows", 5},
                        {"dp_sample_cols", 10}, {"dp_cost_collision", 10000000},
                        {"dp_cost_dl", 15}, {"dp_cost_ddl", 10},
                        {"dp_cost_dddl", 10}, {"dp_cost_ref", 100000}};
        std::unordered_map<std::string, double> qp_path_params = {{"qp_cost_l", 15},
                        {"qp_cost_dl", 1500}, {"qp_cost_ddl", 10},
                        {"qp_cost_dddl", 1}, {"qp_cost_ref", 5},
                        {"qp_cost_end_l", 0}, {"qp_cost_end_dl", 0},
                        {"qp_cost_end_ddl", 0}};

        lane_follow_pnc::EMPlanner em_planner (collision_detection,dp_path_params, qp_path_params);
        FrenetPoint initial_point;
        initial_point.s = 0;
        initial_point.l = 0;
        // 采样
        em_planner.dp_samping(initial_point);
        cout<<"sampling successuflly "<<"the rows 、cols: "<<em_planner.dp_sample_path.size()<<"\t"<<em_planner.dp_sample_path[0].size()<<endl;
        // dp
        em_planner.calc_dp_path(initial_point);
        cout << "get dp_path successfully, the size is " << em_planner.dp_path.size() << endl;
        cout<<"dp规划的path"<<endl;
        for(size_t i = 0; i < em_planner.dp_path.size(); ++i)
        {
            cout<<em_planner.dp_path[i].s<<"\t"<<em_planner.dp_path[i].l<<endl;
        }

        // 增密
        cout<<"dp增密后的path"<<endl;
        em_planner.dp_path_interpolation(em_planner.dp_path);
        
        for(size_t i = 0; i < em_planner.dp_final_path.size(); ++i)
        {
            cout<<em_planner.dp_final_path[i].s<<"\t"<<em_planner.dp_final_path[i].l<<endl;
        }

        // 生成凸空间
        em_planner.calc_convex_space(em_planner.dp_final_path);
        cout<<"get convex space successfully"<<endl;

        // QP 求解
        em_planner.calc_qp_path(em_planner.dp_final_path,em_planner.l_min,em_planner.l_max);
        cout<<"solve qp path successfully"<<endl;
        for(size_t i = 0; i < em_planner.qp_path.size(); ++i)
        {
            cout<<em_planner.qp_path[i].s<<"\t"<<em_planner.qp_path[i].l<<endl;
        }


    
    }
    // 更新匹配点
    pre_match_index = reference_line.match_index;
    return 0;

}


