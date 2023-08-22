/**
 * @file reference_line.cpp
 * @author ZLLee
 * @brief 进行参考线平滑
 * @version 0.1
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "../../include/reference_line/reference_line.h"


using namespace std;

namespace lane_follow_pnc
{
    //初始化参考线
    ReferenceLine::ReferenceLine(double lookahead_distance,
                                std::unordered_map<std::string, double> &referline_params)
    {
        lookahead_dist = lookahead_distance;
        match_index = 0;
        //二次规划的代价权重
        ref_weight_smooth = referline_params["ref_weight_smooth"]; //参考线平滑代价
        ref_weight_path_length = referline_params["ref_weight_path_length"]; //参考线长度代价(紧凑代价)
        ref_weight_ref_deviation = referline_params["ref_weight_offset"]; //参考线形状相似代价

        //二次规划的约束
        x_lower_bound = referline_params["x_lower_bound"];
        x_upper_bound = referline_params["x_upper_bound"];
        y_lower_bound = referline_params["y_lower_bound"];
        y_upper_bound = referline_params["y_upper_bound"];
    }


    //寻找最远路径下标点
    int  ReferenceLine::search_target_index(const double &cur_x, const double &cur_y,
                const std::vector<PathPoint> &path,
                const double &lookahead_distance)
    {
        double dist;
        for(size_t i = match_index; i <path.size(); ++i)
        {
            dist = calDistance(cur_x, cur_y, path[i].x, path[i].y);
            if (dist > lookahead_distance)
            {
                return i;
            }
        }
        return path.size() - 1; //路径太短而前瞻距离很长，而返回整个路径
    }


    // 截取路径
    std::vector<PathPoint> ReferenceLine::local_path_truncation(const CarState &cur_pose,
                                                    const std::vector<PathPoint> &global_path,
                                                    const int &pre_match_index)
    {
        // 获取匹配点编号
        this->match_index = searchMatchIndex(cur_pose.x, cur_pose.y, global_path, pre_match_index);
        cout<<"Macth point_index is "<<match_index<<endl;

        //获取最远路径编号
        int target_index = search_target_index(cur_pose.x, cur_pose.y,global_path, lookahead_dist);
        cout<<"Target point_index is "<<target_index<<endl;

        //通过最远编号返回截取的路径
        vector<PathPoint> target_path(global_path.begin() + this->match_index,
                                        global_path.begin() + target_index + 1);
        cout<<"Size of target_path is "<<target_path.size()<<endl;
        return target_path;
    }

    //使用三次样条插值来平滑
    std::vector<PathPoint> ReferenceLine::smoothing(Spline2D &ref_frenet,
                        const std::vector<PathPoint> &local_path )
    {
        std::vector<PathPoint> ref_path;
        ref_path.clear();
        for (double i = 0; i < ref_frenet.s.back(); i += 0.1)
        {
        std::array<double, 2> point_ = ref_frenet.calc_postion(i);
        PathPoint ref_point;
        ref_point.x = point_[0];
        ref_point.y = point_[1];
        ref_point.heading = ref_frenet.calc_heading(i);
        ref_point.kappa = ref_frenet.calc_kappa(i);
        ref_point.s_ = i;
        ref_path.push_back(ref_point);
        }
        cout<<"The size of ref_path is "<<ref_path.size()<<endl;
        return ref_path;
    }

    // 基于FEM的离散点平滑方法
    std::vector<PathPoint> ReferenceLine::discrete_smooth(const std::vector<PathPoint> &local_path)
    {
        std::vector<PathPoint> smoothed_path;
        std::vector<std::pair<double, double>> path_point2d;
        for(auto point : local_path)
        {
            path_point2d.push_back(std::make_pair(point.x, point.y));
        }

        // 使用二次规划求解
        discrete_points_osqp(path_point2d);
        for(auto point2d : path_point2d)
        {
            PathPoint p;
            p.x = point2d.first;
            p.y = point2d.second;
            smoothed_path.push_back(p);
        }

        //计算航向角和曲率
        calHeading(smoothed_path);
        return smoothed_path;
    }


    // 离散点的二次规划求解器
    void ReferenceLine::discrete_points_osqp(std::vector<std::pair<double,double>> &path_point2d)
    {
        int n = path_point2d.size();
        cout<<"参考线平滑的二次规划求解规模为："<<2*n<<endl;
        // cout<<n<<endl;

        // 初始化A1, A2, A3, f,lb, ub矩阵
        // 平滑代价系数矩阵， x' A1' A1 x,(n-2)
        Eigen::SparseMatrix<double> A1(2*n-4, 2*n);
        // Eigen::SparseMatrix<double> A1(2 * n, 2 * n);

        // 路径长度代价矩阵x' A2' A2 x
        Eigen::SparseMatrix<double> A2(2*n-2, 2*n);
        // Eigen::SparseMatrix<double> A2(2 * n, 2 * n);

        // 路径形状相似代价矩阵 x' A3' A3 x,是一个单位阵
        Eigen::SparseMatrix<double> A3(2*n, 2*n);

        // 定义二次规划的H阵
        Eigen::SparseMatrix<double> H(2*n, 2*n);

        // 定义二次规划中的f向量,也就是导航给出的路径点
        Eigen::VectorXd f = Eigen::VectorXd::Ones(2*n);

        // 定义求解器需要的线性约束矩阵A
        Eigen::SparseMatrix<double> A(2*n, 2*n);

        // 定义约束向量
        Eigen::VectorXd lb = Eigen::VectorXd::Ones(2*n);
        Eigen::VectorXd ub = Eigen::VectorXd::Ones(2*n);

        // 定义求解后返回的结果
        Eigen::VectorXd qp_solution = Eigen::VectorXd::Zero(2*n);

        A.setIdentity();

        // 给约束赋值
        for(int i =0; i<n; i++)
        {
            f[2*i] = path_point2d[i].first;
            f[2*i+1] = path_point2d[i].second;
            // cout<<f[2*i]<<"\t"<<f[2*i+1]<<endl;
            
            lb[2*i] =  path_point2d[i].first + x_lower_bound;
            lb[2*i + 1] = path_point2d[i].second + y_lower_bound;


            ub[2*i] =  path_point2d[i].first + x_upper_bound;
            ub[2*i+1] = path_point2d[i].second + y_upper_bound;     
        }

        // cout<<"输出约束向量"<<endl;
        // for(int i =0; i<2*n; i++)
        // {
        //     cout<<lb[i]<<"\t"<<f[i]<<"\t"<<ub[i]<<endl;
        // }

        // 给平滑代价矩阵A1赋值
        for(int j =0; j<n-2; j++)
        {
            A1.insert(2*j, 2*j) = 1;
            A1.insert(2*j, 2*j+2) = -2;
            A1.insert(2*j, 2*j+4) = 1;
            A1.insert(2*j+1, 2*j+1) = 1;
            A1.insert(2*j+1, 2*j+3) = -2;
            A1.insert(2*j+1, 2*j+ 5) = 1;
        }

        // 给长度代价矩阵A2赋值
        for(int k=0; k<n-1;++k)
        {
            A2.insert(2*k, 2*k) = 1;
            A2.insert(2*k, 2*k +2)=-1;
            A2.insert(2*k+1, 2*k+1)=1;
            A2.insert(2*k+1, 2*k+3) = -1;
        }

        // 给形状相似代价矩阵A3赋值,将A3变为单位阵
        A3.setIdentity();

        //计算矩阵H
        H = 2 * (ref_weight_smooth * A1.transpose()*A1 + 
                ref_weight_path_length*A2.transpose()*A2 +
                ref_weight_ref_deviation * A3.transpose()*A3);
        
        f = -2 * ref_weight_ref_deviation * f.transpose();
        // f = -2 * ref_weight_ref_deviation * f;

        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfConstraints(2*n);
        solver.data()->setNumberOfVariables(2*n);
        solver.data()->setHessianMatrix(H);
        solver.data()->setGradient(f);
        solver.data()->setLinearConstraintsMatrix(A);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);

        if(!solver.initSolver())
        {
            cout<<"QSQP init failed"<<endl;
            return;
        }

        if(!solver.solve())
        {
            cout<<"QSQP solve failed"<<endl;
        }

        //获取最后的优化量
        qp_solution = solver.getSolution();

        for(int i =0; i<n; ++i)
        {
            path_point2d[i].first = qp_solution(2*i);
            path_point2d[i].second = qp_solution(2*i+1);
        }
    }
    
}
