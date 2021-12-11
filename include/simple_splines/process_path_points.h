#include <iostream>
#include <cfloat>
#include <ros/ros.h>

namespace process_pathpoints{

    void delete_point_at(std::vector<std::pair<double, double> > &processed_path_points_list, const int &idx){

        if(idx > (int)processed_path_points_list.size() - 2){

            ROS_ERROR("idx > (int)processed_path_points_list.size() - 2!\n");
            return;

        }

        processed_path_points_list.erase(processed_path_points_list.begin() + idx + 1);
        return;
    
    }

    void insert_point_at(std::vector<std::pair<double, double> >&processed_path_points_list, const int &idx, const double &dis_){
        
        
        std::pair<double, double> p1_ = processed_path_points_list[idx], p2_ = processed_path_points_list[idx + 1];

        double del_x_ = p2_.first - p1_.first; 
        double del_y_ = p2_.second - p1_.second;

        double theta_; 

        theta_ = (del_x_ == 0 ? acos(0) : atan2(del_y_, del_x_));


        double x_ ,  y_; 

        x_ = p1_.first + dis_ * cos(theta_);
        y_ = p1_.second + dis_ * sin(theta_);

        std::pair<double, double> p3_ = {x_ , y_ };
        
        processed_path_points_list.insert(processed_path_points_list.begin() + idx + 1, p3_);
    

    }


};