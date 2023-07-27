#include <iostream>
#include <fstream>
#include <iomanip>
#include "../include/reference_line/reference_line.h"
#include "../include/point_types.h"
using namespace std;
using namespace lane_follow_pnc;
#define N 2264

int main() 
{
    // 获取参考线信息
     // 读取文件
    ifstream fin("../src/data/CubeTown.txt",ios::in);

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
    cout<<"size of v1 is "<<n<<endl;
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

    //定义参数
    double path_length = 50.0;

    std::unordered_map<std::string, double> reference_params = {{"ref_weight_smooth", 50.0},
                    {"ref_weight_path_length", 10.0}, {"ref_weight_offset", 20.0},
                     {"x_lower_bound", -0.1}, {"x_upper_bound", 0.1},
                     {"y_lower_bound", -0.1}, {"y_upper_bound", 0.1}};
    
    cout<<reference_params["x_lower_bound"]<<"\t"<<reference_params["x_upper_bound"]<<endl;
    cout<<reference_params["y_lower_bound"]<<"\t"<<reference_params["y_upper_bound"]<<endl;

    lane_follow_pnc::ReferenceLine reference_line(path_length, reference_params);
    // 定义匹配点坐标
    int pre_match_index = 0;


    int start_index = pre_match_index;

    // 1从全局路径获取局部路径做参考线平滑
    CarState cur_pose;
    cur_pose.x = host_x[0];
    cur_pose.y = host_y[0];
    
    std::vector<PathPoint> global_path;
    for (size_t i =0; i<host_x.size(); ++i)
    {
        PathPoint temp;
        temp.x = host_x[i];
        temp.y = host_y[i];
        global_path.push_back(temp);
        // cout<<global_path[i].x<<" "<<global_path[i].y<<" "<<global_path[i].heading<<endl;

    }

    std::vector<PathPoint> local_path = reference_line.local_path_truncation(cur_pose, global_path, start_index);
    // for(int i =0; i<local_path.size(); ++i)
    // {
    //     cout<<local_path[i].x<<" "<<local_path[i].y<<" "<<local_path[i].heading<<endl;
    // }

    pre_match_index = reference_line.match_index;

    if(local_path.size() > 1)
    {
        // 2.离散点平滑
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
        cout<<"the size of ref_path is"<<ref_path.size()<<endl;
        // for(int i=0; i<10; ++i)
        // {
        //     cout<<ref_path[i].x<<"\t"<<ref_path[i].y<<" "<<ref_path[i].heading<<endl;
        // }
        cout<<"写入成功"<<endl;

        outfile.close();

        return 0;        
    }
    
}
