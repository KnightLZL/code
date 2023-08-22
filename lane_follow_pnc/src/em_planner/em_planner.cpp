/**
 * @file em_planner.cpp
 * @author ZLLee
 * @brief 
 * @version 0.1
 * @date 2023-07-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../../include/em_planner/em_planner.h"

using namespace std;

namespace lane_follow_pnc
{

    /**
     * @brief 创建新的EMPLanner 对象
    */
   EMPlanner::EMPlanner(const CollisionDetection &collision_detection,
                        std::unordered_map<std::string, double> &dp_path_params,
                        std::unordered_map<std::string, double> &qp_path_params)
    {
        this->collision_detection = collision_detection;

        // dp path params
        this->dp_sample_l = dp_path_params["dp_sample_l"];
        this->dp_sample_s = dp_path_params["dp_sample_s"];
        this->dp_sample_rows = static_cast<int>(dp_path_params["dp_sample_rows"]);
        this->dp_sample_cols = static_cast<int>(dp_path_params["dp_sample_cols"]);
        
        this->dp_cost_collision = dp_path_params["dp_cost_collision"];
        this->dp_cost_dl = dp_path_params["dp_cost_dl"];
        this->dp_cost_ddl = dp_path_params["dp_cost_ddl"];
        this->dp_cost_dddl = dp_path_params["dp_cost_dddl"];
        this->dp_cost_ref = dp_path_params["dp_cost_ref"];

        // QP path params
        this->qp_cost_l = qp_path_params["qp_cost_l"];
        this->qp_cost_dl = qp_path_params["qp_cost_dl"];
        this->qp_cost_ddl = qp_path_params["qp_cost_ddl"];
        this->qp_cost_dddl = qp_path_params["qp_cost_dddl"];
        this->qp_cost_ref = qp_path_params["qp_cost_ref"];
        this->qp_cost_end_l = qp_path_params["qp_cost_end_l"];
        this->qp_cost_end_dl = qp_path_params["qp_cost_end_dl"];
        this->qp_cost_end_ddl = qp_path_params["qp_cost_end_ddl"];
    }


    /**
     * @brief DP 路径采样
     *          0   1   2   3
                0   x   x   x   x
                1   x   x   x   x
           -----2-  x---x---x---x----------------reference_line
                3   x   x   x   x
                4   x   x   x   x

    */
   void EMPlanner::dp_samping(const FrenetPoint &initial_point)
   {
        int rows = this->dp_sample_rows;
        int cols = this->dp_sample_cols;
        dp_sample_path.resize(rows);

        for(int i=0; i<rows; ++i)
        {
            dp_sample_path[i].resize(cols);
            for(int j=0; j<cols; ++j)
            {
                // 进行dp采样
                dp_sample_path[i][j].s = initial_point.s + (j + 1) * this->dp_sample_s;
                // dp_sample_path[i][j].s = initial_point.s + j * this->dp_sample_s;
                dp_sample_path[i][j].l = initial_point.l + ((rows + 1) /2 - (i+1)) * this->dp_sample_l;
                dp_sample_path[i][j].l_ds = 0;
                dp_sample_path[i][j].l_d_ds = 0;
            }
        }
   }


   /**
    * @brief 计算两个点之间五次多项式连接的代价
   */
    double EMPlanner::calc_dppath_cost(const FrenetPoint &start, const FrenetPoint &end)
    {
        // 五次多项式连接两点
        QuinticPolynomial quintic_path(start.l, start.l_ds, start.l_d_ds,
                                        end.l, end.l_ds, end.l_d_ds,
                                        start.s, end.s);

        // 五次多项式曲线上离散插值取10个点，计算代价(车辆前行，s递增)
        FrenetPoint insert_point;  //增密的点不需要记录，我们只是为了离散的计算代价，将连续通过采样边离散
        double cost_smooth = 0;   //平滑代价
        double cost_ref = 0;      //参考线代价
        double cost_collision =0;   //平滑代价
        int insert_size = 10;   //离散取得点个数

        for(int i =0; i<insert_size; ++i)
        {
            //计算离散点的s
            insert_point.s = start.s + (i+1)*(end.s - start.s) / insert_size;
            //根据离散的s计算l等参数
            insert_point.l = quintic_path.calc_point(insert_point.s);
            insert_point.l_ds = quintic_path.calc_first_derivative(insert_point.s);
            insert_point.l_d_ds = quintic_path.calc_second_derivative(insert_point.s);
            insert_point.l_d_d_ds = quintic_path.calc_third_derivative(insert_point.s);

            // 计算平滑代价
            cost_smooth += this->dp_cost_dl * pow(insert_point.l_ds, 2) +
                            this->dp_cost_ddl * pow(insert_point.l_d_ds, 2)+
                            this->dp_cost_dddl * pow(insert_point.l_d_d_ds, 2);
            // 如果ddl或者dddl太大，则增大cost
            if(abs(insert_point.l_d_ds) > 0.5 || 
            abs(insert_point.l_d_d_ds) > 0.4)
            {
                cost_smooth +=1e6;
            }

            // 偏离参考线代价
            cost_ref += this->dp_cost_ref * pow(insert_point.l, 2); //因为中心线是参考线，所以l越大越偏离参考线
            
            // 碰撞代价
            for(auto &static_ob : collision_detection.static_obstacle_list)
            {
                double dist = calDistance(insert_point.s, insert_point.l, 
                                        static_ob.point.s, static_ob.point.l);

                if(dist <6.0 && dist > 4.0)
                {
                    cost_collision += 1000 / dist;
                }
                else if(dist < 4.0)
                {
                    cost_collision += this->dp_cost_collision / dist;
                }
                else
                {
                    cost_collision +=0;
                }
                // cout<<"碰撞代价计算完毕"<<endl;
            }

            // cout<<"五次多项式连接代价计算完毕"<<endl;
        }
        // cout<<"碰撞代价计算完毕"<<endl;
        // cout<<"五次多项式连接代价计算完毕"<<endl;
        return cost_smooth + cost_ref + cost_collision;  
    }


    /**
     * @brief  路径动态规划
    **/
   void EMPlanner::calc_dp_path( const FrenetPoint &initial_point)
   {
        int rows = this->dp_sample_rows;
        int cols = this->dp_sample_cols;

        /**************计算dp cost  dp的主函数*****************/
        // 计算起点到第一列的cost
        for(int i = 0; i<rows; ++i)
        {
            dp_sample_path[i][0].dp_cost = calc_dppath_cost(initial_point, dp_sample_path[i][0]);
        }

        // 计算从第二列开始的cost
        for(int j=1; j<cols; ++j)
        {
            for(int i=0; i<rows; ++i)
            {
                dp_sample_path[i][j].dp_cost = DBL_MAX; //每个位置的cost先初始化为最大

                for(int k=0; k<rows; ++k)
                {
                    //计算前一列每一行到当前点的cost
                    double cost = dp_sample_path[k][j-1].dp_cost + 
                                    calc_dppath_cost(dp_sample_path[k][j-1], dp_sample_path[i][j]);
                    if(cost <dp_sample_path[i][j].dp_cost)
                    {
                        dp_sample_path[i][j].dp_cost = cost;  //更新最小的cost
                        dp_sample_path[i][j].dp_pre_row = k;  //记录最优路径的前一列的行数
                    }   
                }
            }
        }

        /******************回溯获取最优路径*********************/
        dp_path.resize(cols + 1); //dp路径的大小为采样列数 + 起点
        dp_path[0] = initial_point;  //第一个点设置为起点
        dp_path[cols] = dp_sample_path[0][cols-1];  //最后一个点为最后一列中cost最小的
        for(int i=1; i<rows; ++i)
        {
            if(dp_sample_path[i][cols-1].dp_cost < dp_path[cols].dp_cost)
            {
                dp_path[cols] = dp_sample_path[i][cols-1]; //找到最后一列中cost最小的
            }
        }
        // 回溯
        for(int j= cols-1; j>0; --j)
        {
            int pre_row = dp_path[j+1].dp_pre_row;
            dp_path[j] = dp_sample_path[pre_row][j-1]; 
            //j-1是因为dp_sample_path是cols，而dp_path是cols+1,dp的j对应sample的j-1
        }     
    }


    /**
    * @brief  dp path 使用五次多项式插值
    */
    void EMPlanner::dp_path_interpolation(const std::vector<FrenetPoint> &dp_path)
    {
        dp_final_path.clear();
        for(size_t i=0; i<dp_path.size()-1; ++i)
        {
            // 五次多项式连接两点
            QuinticPolynomial quintic_path(dp_path[i].l, dp_path[i].l_ds, dp_path[i].l_d_ds,
                                           dp_path[i+1].l, dp_path[i+1].l_ds, dp_path[i+1].l_d_ds,
                                           dp_path[i].s, dp_path[i+1].s);
            // 这里使用<=,是因为在最坏的情况下，不用=是取不到最后一个点的
            for(double s=dp_path[i].s; s<=dp_path[i+1].s; s+=path_ds)
            {
                FrenetPoint insert_point;
                insert_point.s = s;
                insert_point.l = quintic_path.calc_point(s);
                insert_point.l_ds = quintic_path.calc_first_derivative(s);
                insert_point.l_d_ds = quintic_path.calc_second_derivative(s);
                insert_point.l_d_d_ds = quintic_path.calc_third_derivative(s);
                dp_final_path.push_back(insert_point);
            }
        }
    }


    /**
     * @brief 生成 qp path的求解约束,即生成凸空间
    */
   void EMPlanner::calc_convex_space(const std::vector<FrenetPoint> &dp_final_path)
   {
        //对边界进行初始化
        int size = dp_final_path.size();
        // 这里考虑的是一个车道，假设的车道比较大[-5, 5]，不考虑借道
        l_min = Eigen::VectorXd::Ones(size) * -5;
        l_max = Eigen::VectorXd::Ones(size) * 5;

        // 遍历静态障碍物
        for(auto &static_ob : collision_detection.static_obstacle_list)
        {
            // 初始化障碍物的边界,将边界都初始化为静态障碍物的中心点
            // 借鉴车辆坐标系方向，前为正，左为正，右手坐标系
            double obs_s_min = static_ob.point.s;   //后边界
            double obs_s_max = static_ob.point.s;   //前边界
            double obs_l_min = static_ob.point.l;   //右边界
            double obs_l_max = static_ob.point.l;   //左边界

            // 使用&直接引用容器中的元素，不会创建新的对象，进而提高算法效率
            // 因为一个障碍物的box有八个点，所以依次访问，将边界赋值
            for(auto &box_point :static_ob.collision_box)
            {
                obs_s_min = min(box_point.s, obs_s_min);
                obs_s_max = max(box_point.s, obs_s_max);

                obs_l_min = min(box_point.l, obs_l_min);
                obs_l_max = max(box_point.l, obs_l_max);
            }

            // 计算障碍物到dp path上的匹配点
            int start_index = get_obstacle_index(dp_final_path, obs_s_min);
            int end_index = get_obstacle_index(dp_final_path, obs_s_max);
            int mid_index = get_obstacle_index(dp_final_path, static_ob.point.s);

            //如果某个障碍物不在路径范围内，则跳过，去遍历下一个障碍物
            if(start_index == 0 ||end_index ==(size - 1))
            {
                continue;
            }
            double path_l = dp_final_path[mid_index].l;

            cout<<"生成凸空间时起始点和终点的下标为："<<start_index<<"\t"<<end_index<<endl;

            //根据dp生成的路径得知车辆是向左还是向右
            for(int i = start_index; i<=end_index; ++i)
            {
                //从左边过
                if(path_l > static_ob.point.l)
                {
                    l_min(i) = max(l_min(i), obs_l_max);
                }

                //从右边过path_l < static_ob.point.l
                else
                {
                    l_max(i) = min(l_max(i), obs_l_min);
                }
            }
        }
    }


    /**
     * @brief 计算障碍物到dp path上的匹配点
    */

    int EMPlanner::get_obstacle_index(const std::vector<FrenetPoint> &dp_final_path,
                               const double &ob_s)
    {
        // 障碍物不在边界内
        //障碍物在车辆的后面
        if(dp_final_path.front().s > ob_s)
        {
            return 0;
        }
        //障碍物离车太远
        else if(dp_final_path.back().s < ob_s)
        {
            return dp_final_path.size() - 1;
        }
        else
        {
            int index = 0;
            // 先找到最接近点的下标,就是dp path上s大于ob_s的第一个点
            for(size_t i =0; i<dp_final_path.size(); ++i)
            {
                if (dp_final_path[i].s > ob_s)
                {
                    index = i;
                    break;
                }
            }
            // 判断是index还是Index-1更近
            if(fabs(dp_final_path[index].s - ob_s) > fabs(dp_final_path[index-1].s - ob_s))
            {
                return index -1;
            }
            else
            {
                return index;
            }
        }
    }


    /**
     * @brief 计算二次规划后的路径
            0.5*x'Hx + f'*x = min
            subject to A*x <= b
                    Aeq*x = beq
                    lb <= x <= ub;
            其中
                x = [l1,l1_dl,l1_ddl,l2,l2_dl,l2_ddl ....ln,ln_dl,ln_ddl]^T 3n个参数

            QP求解条件
                qp_cost_l = 2;
                qp_cost_dl = 25000;
                qp_cost_ddl = 15;
                qp_cost_dddl = 15;
                qp_cost_ref = 15;
                qp_cost_end_l = 15;
                qp_cost_end_dl = 15;
                qp_cost_end_ddl = 15;
                dp_final_path dp结果
                l_min,l_max 凸边界
            QP求解结果
                qp_path_l
                qp_path_l_ds
                qp_path_l_d_ds
    */
    void EMPlanner::calc_qp_path(const std::vector<FrenetPoint> &dp_final_path,
                                 const Eigen::VectorXd &l_min,
                                 const Eigen::VectorXd &l_max)
    {
        int n = dp_final_path.size();
        double ds = path_ds;

        //对于路径规划只需要考虑车宽就行
        double width = 3.0; //车宽

        // 定义Hissen矩阵  H,H_L,H_dl,H_ddl,H_dddl，H_center,
        Eigen::SparseMatrix<double> H(3*n, 3*n);
        Eigen::SparseMatrix<double> H_L(3*n, 3*n);
        Eigen::SparseMatrix<double> H_DL(3*n,3*n);
        Eigen::SparseMatrix<double> H_DDL(3*n, 3*n);
        Eigen::SparseMatrix<double> H_DDDL(n-1, 3*n);

        Eigen::SparseMatrix<double> H_CENTER(3*n, 3*n);
        Eigen::SparseMatrix<double> H_L_END(3*n, 3*n);
        Eigen::SparseMatrix<double> H_DL_END(3*n, 3*n);
        Eigen::SparseMatrix<double> H_DDL_END(3*n, 3*n);

        // 定义线性项 f
        Eigen::VectorXd f = Eigen::VectorXd::Zero(3*n);

        // 定义等式约束  加加速度优化法
        // Eigen::SparseMatrix<double> Aeq(2*n-2, 3*n);
        Eigen::VectorXd beq = Eigen::VectorXd::Zero(2*n-2);

        // 定义不等式约束  考虑车辆几何形状
        // Eigen::SparseMatrix<double> A_lu(3*n, 3*n);
        // A_lu.setIdentity();

        //定义的不等式约束,并初始化
        Eigen::VectorXd lb = Eigen::VectorXd::Ones(3*n) * (-DBL_MAX);  //不等式最小边界
        Eigen::VectorXd ub = Eigen::VectorXd::Ones(3*n) * (DBL_MAX);   //不等式最大边界

        // 约束整合矩阵 lb_merge <= A_merge * x <= ub_merge
        Eigen::SparseMatrix<double> A_merge(2*n-2+3*n, 3*n);   //稀疏矩阵，保存时只保存非0的

        Eigen::VectorXd lb_merge(2*n-2+3*n);   //稠密向量，是连续存储的
        Eigen::VectorXd ub_merge(2*n-2+3*n);


        /***************计算相应的H矩阵****************/
        for(int i=0; i<n; ++i)
        {
            H_L.insert(3*i, 3*i) = 1;
            H_DL.insert(3*i+1, 3*i+1) = 1;
            H_DDL.insert(3*i+2, 3*i+2) = 1;
        }
        H_CENTER = H_L;

        // 自己写的是i <n,这里会越界
        for(int i = 0; i<n-1;++i)
        {
            int row = i;
            int col = 3*i;

            H_DDDL.insert(row, col+2) =-1;
            H_DDDL.insert(row, col+5) = 1;
        }
        
        // 这里自己赋值的是三个0
        H_L_END.insert(3*n-3, 3*n-3) = 1;
        H_DL_END.insert(3*n-2, 3*n-2) = 1;
        H_DDL_END.insert(3*n-1, 3*n-1) = 1;

        // 因为ds != 1，所以H_DDDL = delta（ddl）/ ds，需要考虑到ds
        H = this->qp_cost_l * (H_L.transpose() * H_L) +
            this->qp_cost_dl * (H_DL.transpose() * H_DL) +
            this->qp_cost_ddl * (H_DDL.transpose() * H_DDL) +
            this->qp_cost_dddl * (H_DDDL.transpose() * H_DDDL) / pow(ds, 2) +
            this->qp_cost_ref * (H_CENTER.transpose() * H_CENTER) +
            this->qp_cost_end_l * (H_L_END.transpose() * H_L_END) +
            this->qp_cost_end_dl * (H_DL_END.transpose() * H_DL_END) +
            this->qp_cost_end_ddl * (H_DDL_END.transpose() * H_DDL_END);
        H = 2 * H;

        // 求f,自己没有加f
        // for (int i = 0; i < n; i++)
        // {
        //     f(3 * i) = (-2) * this->qp_cost_ref * dp_final_path[i].l;
        //     // f(3 * i) = (-2) * this->qp_cost_ref * 0.5 * (l_min(i) + l_max(i));
        // }

        /*****************计算相应约束*******************/
        // 等式约束，加加速度优化法
        int row_index_start = 0;
        for(int i =0; i<n-1; ++i)
        {
            int row = row_index_start + 2*i;
            int col = 3*i;
            A_merge.insert(row, col) = 1;
            A_merge.insert(row, col+1) = ds;
            A_merge.insert(row, col+2) = pow(ds, 2) / 3;
            A_merge.insert(row, col+3) = -1;
            A_merge.insert(row, col+5) = pow(ds, 2) / 6;

            A_merge.insert(row+1, col+1) = 1;
            A_merge.insert(row+1, col+2) = ds / 2;
            A_merge.insert(row+1, col+4) = -1;
            A_merge.insert(row+1, col+5) = ds/2;
        }
        
        //不等式约束
        row_index_start = 2*n - 2;
        for(int i=0; i<n; ++i)
        {
            int row = row_index_start + 3*i;
            int col = 3*i;
            A_merge.insert(row, col) = 1;
            A_merge.insert(row+1, col+1) = 1;
            A_merge.insert(row+2, col+2) = 1;
        }

        //给规划起点添加的约束为当前点的位置，从而保证规划起点在qp中是不变的，
        lb(0) = dp_final_path[0].l;
        lb(1) = dp_final_path[0].l_ds;
        lb(2) = dp_final_path[0].l_d_ds;
        ub(0) = lb(0);
        ub(1) = lb(1);
        ub(2) = lb(2);

        //计算不等式约束，因为等式约束beq就是0
        for( int i =1; i<n; ++i)
        {
            lb(3*i) = l_min(i) + width /2;
            lb(3*i+1) = -2.0;
            lb(3*i+2) = -0.1;

            ub(3*i) = l_max(i) - width/2;
            ub(3*i+1) = 2.0;
            ub(3*i+2) = 0.1;
        }


        /******对等式和不等式的约束进行汇总*****/

        // 首先是等式约束
        lb_merge.block(0, 0, 2*n-2, 1) = beq;
        ub_merge.block(0, 0, 2*n-2, 1) = beq;

        //不等式约束
        lb_merge.block(2*n-2, 0, 3*n, 1) = lb;
        //在lb_merge矩阵的(2*n-2, 0)的位置插入一个(3*n, 1)大小的矩阵
        ub_merge.block(2*n-2, 0, 3*n, 1) = ub;


        /************进行二次规划求解***************/
        OsqpEigen::Solver solver;

        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);  //是否打印osqp的日志
        solver.data()->setNumberOfVariables(3*n); //优化变量的个数  A阵列数
        solver.data()->setNumberOfConstraints(2*n-2+3*n); //约束的个数  A阵行数
        solver.data()->setHessianMatrix(H);    //代价函数矩阵
        solver.data()->setGradient(f);      //线性项矩阵
        solver.data()->setLinearConstraintsMatrix(A_merge);  //线性约束矩阵
        solver.data()->setLowerBound(lb_merge);   //约束下界
        solver.data()->setUpperBound(ub_merge);  //y约束上界

        if (!solver.initSolver())
        {
            cout<<"QSOP init failed"<<endl;
            return;
        }
        if (!solver.solve())
        {
            cout<<"QSOP solve failed"<<endl;;
            return;
        }

        Eigen::VectorXd qp_solution(3*n);
        qp_solution = solver.getSolution();

        qp_path = dp_final_path;  //给qp_path先初始化，让s和l_d_d_ds和dp的一样
        for(int i=0; i<n; ++i)
        {
            qp_path[i].l = qp_solution(3*i);
            qp_path[i].l_ds = qp_solution(3*i+1);
            qp_path[i].l_d_ds = qp_solution(3*i+2);
        }

        cout<<"get QP path successfully"<<endl;
    }


    /***
     * @brief 将frenet轨迹转换为path轨迹
    */
    // 寻找自车定位点与参考线之间的匹配点下标
//    FrenetPoint  calc_projpoint( vector<FrenetPoint> &point, double s)
//    {
//         //首先寻找匹配点
//         int match_index = 1;
//         while(point[match_index].s < s)
//         {
//             match_index ++;
//         }

//         FrenetPoint match_point = point[match_index];
        
//         FrenetPoint projection_point = match_point;

//         //计算匹配点的切向量 
//         Eigen::Matrix<double, 2, 1> tor;
//         tor<<cos(match_point.heading), sin(match_point.heading);

//         // 匹配点至自车向量d
//         Eigen::Matrix<double, 2, 1> d;
//         d<<cur_pose.x - match_point.x, cur_pose.y - match_point.y;

//         //计算d在tor方向上的投影分量
//         double e_s = tor.transpose() * d;

//         // 求投影点
//         projection_point.x = match_point.x + e_s*cos(match_point.heading);
//         projection_point.y = match_point.y + e_s*sin(match_point.heading);

//         //求投影点的heading,认为投影点到匹配点的曲率不变
//         projection_point.heading = match_point.heading + e_s * match_point.kappa;
//         return projection_point;
//    }

   void EMPlanner::get_cartesian_paths(std::vector<FrenetPoint> &frenet_path, std::vector<FrenetPoint> &ref_path)
   {    
         //将frenet转为cartesian
        for (size_t i =0; i<frenet_path.size(); ++i)
        {
            //寻找匹配点
            int match_index = 1;
            while(ref_path[match_index].s < frenet_path[i].s)
            {
                match_index ++;
            }
            //得到匹配点
            FrenetPoint match_point = ref_path[match_index];
            //定义投影点
            FrenetPoint projection_point;
             //计算ds 
            double ds = frenet_path[i].s - match_point.s;
            // 求投影点的x,y
            projection_point.x = match_point.x + ds * cos(match_point.heading);
            projection_point.y = match_point.y + ds * sin(match_point.heading);
            //求投影点的heading,认为投影点到匹配点的曲率不变,近似认为dkappa=0
            projection_point.heading = match_point.heading + ds * match_point.kappa;
            projection_point.kappa = match_point.kappa;

            //求笛卡尔坐标系下的点
            //定义法向量
            // Eigen::Matrix<double,2, 1> nor;
            // nor[0][0] = -std::sin(projection_point.heading);
            // nor[1][0] = std::cos(projection_point.heading);
            // Eigen::Matrix<double, 2, 1> point;
            // point[0][0] = projection_point.x;
            // point[1][0] = projection_point.y;
            // point = point + frenet_path[i];
            frenet_path[i].x = projection_point.x + frenet_path[i].l * -std::sin(projection_point.heading);
            frenet_path[i].y = projection_point.y + frenet_path[i].l * std::cos(projection_point.heading);
            frenet_path[i].heading = projection_point.heading + 
                        std::atan(frenet_path[i].l_ds / (1 - projection_point.kappa * frenet_path[i].l));
            frenet_path[i].kappa =((frenet_path[i].l_d_ds + projection_point.kappa + frenet_path[i].l_ds * std::tan(frenet_path[i].heading- projection_point.heading)) *
                                  (std::cos(frenet_path[i].heading- projection_point.heading) / (1 -projection_point.kappa * frenet_path[i].l)) + projection_point.kappa) *
                                  std::cos(frenet_path[i].heading- projection_point.heading) / (1 -projection_point.kappa * frenet_path[i].l);
            // 设定期望速度(速度规划还未完成)
            frenet_path[i].v = desired_speed;
        }
    }

    // 规划任务函数
    FrenetPath EMPlanner::planning(std::vector<FrenetPoint> &ref_path, const FrenetPoint &initial_point)
    {
        FrenetPath final_path;   //定义的存储路径规划
        /*********DP Path*******/
        // dp采样
        dp_samping(initial_point);
        cout<<"sampling successuflly "<<"the rows 、cols: "<<dp_sample_path.size()<<"\t"<<dp_sample_path[0].size()<<endl;

        // dp path动态规划
        calc_dp_path(initial_point);
        cout << "get dp_path successfully, the size is " << dp_path.size() << endl;

        //dp path 插值增密
        dp_path_interpolation(dp_path);
        cout<<"get interpolation successfully, this size is "<<dp_final_path.size()<<endl;


        /************QP path***************/
        // 生成凸空间
        calc_convex_space(this->dp_final_path);
        cout<<"get convex space successfully"<<endl;

        // QP 求解
        calc_qp_path(this->dp_final_path,this->l_min,this->l_max);
        cout<<"solve qp path successfully"<<endl;

        // 转回cartesian坐标
        get_cartesian_paths(qp_path, ref_path);

        final_path.frenet_path = qp_path;

        return final_path;

    }



    // 使用三次样条曲线做frenet坐标系
    int EMPlanner::get_cartesian_paths(std::vector<FrenetPoint> &frenet_path, Spline2D &ref_frenet)
    {

        int size_ = 0;
        for (unsigned int i = 0; i < frenet_path.size(); ++i)
        {
            // 若轨迹比参考轨迹长，则及时截断
            if (frenet_path[i].s >= ref_frenet.s.back())
            {
                break;
            }
            size_++; // 有效点个数
            /*******************************求cartesian 坐标系中 x,y******************************************/
            // 投影点信息
            std::array<double, 2> poi = ref_frenet.calc_postion(frenet_path[i].s);
            double yawi = ref_frenet.calc_heading(frenet_path[i].s);
            double curi = ref_frenet.calc_kappa(frenet_path[i].s);
            double li = frenet_path[i].l;

            frenet_path[i].x = poi[0] + li * cos(yawi + M_PI / 2.0);
            frenet_path[i].y = poi[1] + li * sin(yawi + M_PI / 2.0);
            /*********************************求cartesian 坐标系中 yaw****************************************/
            frenet_path[i].heading = atan2(frenet_path[i].l_ds, (1 - curi * li)) + yawi;
            frenet_path[i].v = desired_speed;
        }
        return size_;
    }

      FrenetPath EMPlanner::planning(Spline2D &ref_frenet, const FrenetPoint &initial_point)
    {

        FrenetPath final_path;
        /***********************************DP path**************************************/
        // dp路径采样
        dp_samping(initial_point);


        // dp path动态规划
        calc_dp_path(initial_point);

        // dp path插值
        dp_path_interpolation(dp_path);

        /***********************************QP path**************************************/
        // 生成凸空间
        calc_convex_space(this->dp_final_path);

        // QP 求解
        calc_qp_path(this->dp_final_path, this->l_min, this->l_max);

        // 转回全局坐标系
        final_path.size_ = get_cartesian_paths(qp_path, ref_frenet);

        final_path.frenet_path = qp_path;

        return final_path;
    }




}//lane_follow_pnc