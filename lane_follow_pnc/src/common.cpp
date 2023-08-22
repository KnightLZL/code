/**
 * @file common.cpp
 * @author ZLLee
 * @brief 完成匹配点寻找，投影点计算、 坐标系转换
 * @version 0.1
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "../include/common.h"

#include <vector>

using namespace std;

namespace lane_follow_pnc
{
    // 计算两点间的距离
    double calDistance(double x1,double y1, double x2, double y2)
    {
        double dx = x2-x1;
        double dy = y2-y1;
        return std::sqrt(dx*dx +dy*dy);
    }

    // 寻找自车定位点与参考线之间的匹配点下标
    int searchMatchIndex(const double &cur_x, const double &cur_y,
                                const std::vector<PathPoint> &path,
                                const int &pre_match_index)
    {
        double dist;
        double min_dist = DBL_MAX;    //DBL_MAX为double类型的最大值
        int match_index = 0;
        for(size_t i =pre_match_index; i < path.size(); ++i)
        {
            dist = calDistance(path[i].x, path[i].y, cur_x, cur_y);
            if(dist <min_dist)
            {
                min_dist = dist;
                match_index = i;
            }
        }
        return match_index;
    }
    

    //通过匹配点求投影点
    PathPoint match2Projection(const CarState &cur_pose,
                                const PathPoint &match_point)
    {
        //投影点的其他值与匹配点想用，只求其x,y,heading
        PathPoint projection_point = match_point;

        //计算匹配点的切向量 
        Eigen::Matrix<double, 2, 1> tor;
        tor<<cos(match_point.heading), sin(match_point.heading);

        // 匹配点至自车向量d
        Eigen::Matrix<double, 2, 1> d;
        d<<cur_pose.x - match_point.x, cur_pose.y - match_point.y;

        //计算d在tor方向上的投影分量
        double e_s = tor.transpose() * d;

        // 求投影点
        projection_point.x = match_point.x + e_s*cos(match_point.heading);
        projection_point.y = match_point.y + e_s*sin(match_point.heading);

        //求投影点的heading,认为投影点到匹配点的曲率不变
        projection_point.heading = match_point.heading + e_s * match_point.kappa;
        return projection_point;
    }


    //计算航向角和曲率
    void calHeading(std::vector<PathPoint> &xy_points)
    {	
        // 定义中间变量
        std::vector<double> accumulated_s;	

		if (xy_points.size() < 2)
		{
            cout<<"the size of point is not engough"<<endl;
			return; //只有一个点，不能计算信息
		}
		std::vector<double> dxs;
		std::vector<double> dys;
		std::vector<double> y_over_s_first_derivatives;
		std::vector<double> x_over_s_first_derivatives;
		std::vector<double> y_over_s_second_derivatives;
		std::vector<double> x_over_s_second_derivatives;

		//为了方便计算heading和kappa，通过有限微信来计算dx和dy
		//首先计算dx和dy
		int points_size = xy_points.size();  //size_t会提高代码的平台适用性,一般可以用int
		for (int i = 0; i < points_size; ++i)
		{
			double x_delta = 0.0;
			double y_delta = 0.0;
			if (i == 0)  //第一个点，向后欧拉法
			{
				x_delta = (xy_points[i + 1].x - xy_points[i].x);
				y_delta = (xy_points[i + 1].y - xy_points[i].y);
			}
			else if (i == points_size - 1) //最后一个点。向后欧拉法
			{
				x_delta = (xy_points[i].x - xy_points[i - 1].x);
				y_delta = (xy_points[i].y - xy_points[i - 1].y);
			}
			else //中间的点，中点欧拉法
			{
				x_delta = 0.5*(xy_points[i + 1].x - xy_points[i - 1].x);
				y_delta = 0.5*(xy_points[i + 1].y - xy_points[i - 1].y);
			}
			dxs.push_back(x_delta);
			dys.push_back(y_delta);
		}

		//heading计算
		for (int i = 0; i < points_size; i++)
		{
			xy_points[i].heading = (std::atan2(dys[i], dxs[i]));  //atan2对象限敏感，可以根据输入得到象限，输出范围为[-pi,pi],而atan不敏感，输出的是[-pi/2,pi/2]
		}

		//为了计算kappa，对s进行线性插值,得到accumulated_s
		double distance = 0.0;
		accumulated_s.push_back(distance);
		double fx = xy_points[0].x;
		double fy = xy_points[0].y;
		double nx = 0.0;
		double ny = 0.0;
		for (int i = 1; i < points_size; ++i)
		{
			nx = xy_points[i].x;
			ny = xy_points[i].y;
			double end_segment_s =
				std::sqrt((fx - nx)*(fx - nx) + (fy - ny)*(fy - ny));
			accumulated_s.push_back(end_segment_s + distance);
			distance += end_segment_s;
			fx = nx;
			fy = ny;
		}

		//为了计算kappa，进行x和y的一阶差分,获得一阶差分
		for (int i = 0; i < points_size; ++i)
		{
			double xds = 0.0;
			double yds = 0.0;
			if (i == 0)
			{
				xds = (xy_points[i + 1].x - xy_points[i].x) /
					(accumulated_s.at(i+1) - accumulated_s.at(i));
				yds = (xy_points[i + 1].y - xy_points[i].y)/
					(accumulated_s.at(i+1) - accumulated_s.at(i));
			}
			else if (i == points_size - 1)
			{
				xds = (xy_points[i].x - xy_points[i - 1].x) /
					(accumulated_s.at(i) - accumulated_s.at(i-1));  //xy_points通过引用传入，直接使用[]索引进行获取；accumulated_s通过指针传入，需要通过at(i)函数获取索引为i的值；
				yds = (xy_points[i].y - xy_points[i - 1].y) /
					(accumulated_s.at(i) - accumulated_s.at(i-1));
			}
			else
			{
				xds = (xy_points[i + 1].x - xy_points[i - 1].x) /
					(accumulated_s.at(i + 1) - accumulated_s.at(i - 1));
				yds = (xy_points[i + 1].y - xy_points[i - 1].y) /
					(accumulated_s.at(i + 1) - accumulated_s.at(i - 1));
			}
			x_over_s_first_derivatives.push_back(xds);
			y_over_s_first_derivatives.push_back(yds);
		}

		//为了计算kappa，需要获得x和y的二阶微分
		for (int i = 0; i < points_size; ++i)
		{
			double xdds = 0.0;
			double ydds = 0.0;
			if (i == 0)
			{
				xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
					(accumulated_s.at(i + 1) - accumulated_s.at(i));
				ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
					(accumulated_s.at(i + 1) - accumulated_s.at(i));
			}
			else if (i == points_size - 1)
			{
				xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
					(accumulated_s.at(i) - accumulated_s.at(i - 1));
				ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
					(accumulated_s.at(i) - accumulated_s.at(i - 1));
			}
			else
			{
				xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
					(accumulated_s.at(i + 1) - accumulated_s.at(i - 1));
				ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
					(accumulated_s.at(i + 1) - accumulated_s.at(i - 1));
			}
			x_over_s_second_derivatives.push_back(xdds);
			y_over_s_second_derivatives.push_back(ydds);
		}

		//计算kappa
		for (int i = 0; i < points_size; ++i)
		{
			double xds = x_over_s_first_derivatives[i];
			double yds = y_over_s_first_derivatives[i];
			double xdds = x_over_s_second_derivatives[i];
			double ydds = y_over_s_second_derivatives[i];
			double kappa =
				(xds * ydds - yds * xdds) /
				(std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);  
			xy_points[i].kappa = kappa;
		}
		return ;
	}


    // Cartesian2Frenet
    FrenetPoint Cartesain2Frenet( const CarState &global_point,
                                const PathPoint &projection_point)
    {
        FrenetPoint frenet_point;
        //保留原信息
        frenet_point.x = global_point.x;
        frenet_point.y = global_point.y;
        frenet_point.z = global_point.z;
        frenet_point.heading = global_point.heading;
        frenet_point.kappa = global_point.kappa;

        frenet_point.vx = global_point.vx;
        frenet_point.vy = global_point.vy;
        frenet_point.v = global_point.v;
        frenet_point.ax = global_point.ax;
        frenet_point.ay = global_point.ay;
        frenet_point.a = global_point.a;
        frenet_point.t = global_point.t;

        double delta_theta = global_point.heading - projection_point.heading;
        // 计算s
        frenet_point.s = projection_point.s_;

        //计算l = （x-or - x_r_or） *n_or
        Eigen::Matrix<double, 2, 1> x_or;
        x_or << global_point.x, global_point.y;

        Eigen::Matrix<double, 2, 1> x_r_or;
        x_r_or <<projection_point.x, projection_point.y;

        // 计算法向量
        Eigen::Matrix<double, 2, 1> n_or;
        n_or <<-sin(projection_point.heading), cos(projection_point.heading); 

        //计算l
        frenet_point.l = (x_or - x_r_or).transpose() * n_or;

        // 计算s_d
        frenet_point.s_d = global_point.v * std::cos(delta_theta) / 
                            (1 - projection_point.heading * frenet_point.l);
        
        // 计算l_d
        frenet_point.l_d = global_point.v * std::sin(delta_theta);

        //计算l_ds
        if(fabs(frenet_point.s_d) < 1e-6)
        {
            frenet_point.l_ds = 0;
        }
        else
        {
            frenet_point.l_ds = frenet_point.l_d / frenet_point.s_d;
        }

        //计算l_d_d
        frenet_point.l_d_d = global_point.a * std::sin(delta_theta) - global_point.kappa * (1-global_point.heading) * frenet_point.s_d *frenet_point.s_d;

        //计算s_d_d
        Eigen::Matrix<double, 2, 1> t_or;
        t_or << cos(projection_point.heading), sin(projection_point.heading);

        Eigen::Matrix<double, 2, 1> a_or;
        a_or << global_point.ax, global_point.ay;
        //忽略dkr/ds,认为其等于0
        frenet_point.s_d_d = (a_or.transpose() * t_or + 2 * frenet_point.l_ds * projection_point.kappa * pow(frenet_point.s_d,2)) /
                            (1 - projection_point.kappa * frenet_point.l);
        
        //计算l_d_ds
        if(fabs(frenet_point.s_d) < 1e-6)
        {
            frenet_point.l_d_ds = 0;
        }
        else
        {
            frenet_point.l_d_ds = (frenet_point.l_d_d - frenet_point.l_ds * frenet_point.s_d_d) /
                                (pow(frenet_point.s_d, 2));
        }

        return frenet_point;
    }

    //计算frenet坐标
    // FrenetPoint calcFrenet(const CarState &global_point,
    //                             const std::vector<PathPoint> &ref_path)
    // {
    //     //计算匹配点下标
    //     int frenet_match_index = searchMatchIndex(global_point.x, global_point.y, ref_path, 0 );

    //     //通过匹配点求投影点
    //     PathPoint projection_point = match2Projection(global_point, ref_path[frenet_match_index]);

    //     //计算frenet坐标
    //     FrenetPoint frenet_point = Cartesain2Frenet(global_point, projection_point);

    //     return frenet_point;
    // }

    // 计算投影点坐标的s
    void  cal_s( FrenetPoint &point,  PathPoint & host_point)
    {
        point.s = point.s - host_point.s_;
    }


    // 计算frenet坐标重载
    FrenetPoint calcFrenet(const CarState &global_point,
                                const std::vector<PathPoint> &ref_path, const int pre_match_index,
                                PathPoint & host_point)
    {
        //计算匹配点下标
        int frenet_match_index = searchMatchIndex(global_point.x, global_point.y, ref_path, pre_match_index);
        // int frenet_match_index = match_point_index;

        //通过匹配点求投影点
        PathPoint projection_point = match2Projection(global_point, ref_path[frenet_match_index]);

        //计算frenet坐标
        FrenetPoint frenet_point = Cartesain2Frenet(global_point, projection_point);

        // 计算对应的s
        cal_s(frenet_point, host_point);


        return frenet_point;
    }


    //将角度值归一化到-pi , pi之间
    double NormalizeAngle(const double angle) 
    {
        //fmod取模操作
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);

        //如果为负数，让其变为正数
        if (a < 0.0) 
        {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }


    /*****************frenet 2 cartesian********************/
    void Frenet2Cartesian( const double rs, const double rx, const double ry, const double rtheta,
                                    const double rkappa, const double rdkappa,
                                    const std::array<double, 3>& s_condition,
                                    const std::array<double, 3>& d_condition, double* const ptr_x,
                                    double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
                                    double* const ptr_v, double* const ptr_a) 
    {

        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);

        *ptr_x = rx - sin_theta_r * d_condition[0];
        *ptr_y = ry + cos_theta_r * d_condition[0];

        const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

        const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
        const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
        const double cos_delta_theta = std::cos(delta_theta);

        *ptr_theta = NormalizeAngle(delta_theta + rtheta);

        const double kappa_r_d_prime =
            rdkappa * d_condition[0] + rkappa * d_condition[1];
        *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                        cos_delta_theta * cos_delta_theta) /
                        (one_minus_kappa_r_d) +
                    rkappa) *
                    cos_delta_theta / (one_minus_kappa_r_d);

        const double d_dot = d_condition[1] * s_condition[1];
        *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                                s_condition[1] * s_condition[1] +
                            d_dot * d_dot);

        const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

        *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
                s_condition[1] * s_condition[1] / cos_delta_theta *
                    (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
    }

} //namespace lane_follow_pnc
