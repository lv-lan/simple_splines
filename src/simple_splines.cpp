#include <simple_splines/simple_splines.h>
#include <simple_splines/vis_functions.h>
#include <simple_splines/process_path_points.h>

/**
 * @brief Returns the value of the cofficients a_0, a_1, a_2 and a_3 for a spline patch between idx and idx + 1
 * 
 * @param points_ 
 * @param idx 
 * @return std::vector<double> 
 */

std::vector<double> SimpleSplines::generate_spline_patch_cofficients(const std::vector<std::pair<double,double> > &points_, int idx){


    if(idx >= (int)points_.size() - 1) {

        ROS_ERROR("idx >= (int)points_.size() - 1\n");
        return std::vector<double>{};

    }

    double P_0, P_1, P_2, P_3; 

    double P1_dash, P2_dash; 

    P_1 = points_[idx].second, P_2 = points_[idx+ 1].second;

    P_0  = (idx - 1 < 0 ? P_2 : points_[idx - 1].second);  //Setting the slope to 0 for the first waypoint in the path

    P_3 = (idx + 2 > (int)points_.size() - 1 ? P_1 : points_[idx + 2].second); //Setting the gradient to 0 for last waypoint in the path

    ROS_WARN("P_0: %f P_3: %f\n", P_0, P_3);

    P1_dash = (P_2 - P_0)/2.0;
    
    P2_dash = (P_3 - P_1)/2.0;

    double a_0, a_1, a_2, a_3 ; 

    a_0  = P_1;
    a_1 =  P1_dash;

    a_3 = P2_dash - 2 * P_2 + 2 * a_0 + a_1;
    a_2 = P_2 - (a_0 + a_1 + a_3);

    return {a_0, a_1, a_2, a_3};

}


/**
 * @brief Iterates over waypoint pairs and generates spline patches between them 
 * 
 * @param points_ 
 */
void SimpleSplines::generate_complete_spline(const std::vector<std::pair<double, double> >&waypoint_list_){

    ROS_INFO("Inside generate_spline_path_points function!\n");
    int sz_ = (int)waypoint_list_.size(); 

    ROS_WARN("sz_: %d\n", sz_);

    for(int i = 0; i < sz_ -1  ; i++) {

        std::vector<double> cofficients_ = generate_spline_patch_cofficients(waypoint_list_, i);

        generate_spline_patch(path_points_, cofficients_, waypoint_list_, i);        

    }


}

/**
 * @brief Generates spline patch between two given waypoints
 * 
 * @param v_ 
 * @param points_ 
 * @param idx 
 */
void SimpleSplines::generate_spline_patch(std::vector<std::pair<double, double> >&path_points_list_, std::vector<double> &cofficients_, const std::vector<std::pair<double, double> >&waypoint_list_,  int idx){

    if(idx >= (int)waypoint_list_.size() - 1) {

        ROS_ERROR("idx >= (int)points_.size() - 1\n");
        return;

    }

    ROS_INFO("Generating spline_patch_points!\n");

    double a_0, a_1, a_2, a_3; 
    a_0 = cofficients_[0]; 
    a_1 = cofficients_[1]; 
    a_2 = cofficients_[2]; 
    a_3 = cofficients_[3];

    ROS_WARN("a_0: %f a_1: %f a_2: %f a_3: %f\n", a_0, a_1, a_2, a_3);
    
    double step_sz = 0.1;
 
    std::pair<double, double> P_1 = waypoint_list_[idx], P_2 = waypoint_list_[idx+ 1];

    ROS_WARN("P1: (%f,%f) P2: (%f,%f)\n", P_1.first, P_1.second, P_2.first, P_2.second);

    double x0_ = P_1.first, x1_ = P_2.first;  

    for(double mu = 0 ; mu <= 1; mu += step_sz){

        double x_ = x0_ + mu * (x1_ - x0_);

        double y_ = a_3 * pow(mu, 3) + a_2 * pow(mu,2) + a_1 * pow(mu, 1) + a_0 ;

        ROS_INFO("x_: %f y_: %f\n", x_, y_);

        path_points_list_.push_back({x_, y_}); 
        
    }

}




void SimpleSplines::process_path_points(const std::vector<std::pair<double, double> > &path_points_list, std::vector<std::pair<double, double> > &processed_path_points_list){

    int sz_ = (int)path_points_list.size(); 

    if(sz_ < 2) {

        ROS_ERROR("sz_ < 2!\n");
        return;

    }

    processed_path_points_list = path_points_list;

    double dis_ = 0.25; 

    for(int i = 0 ;i < (int)processed_path_points_list.size() - 1; i++) {

        std::pair<double, double> p1_ = {processed_path_points_list[i].first, processed_path_points_list[i].second};
        std::pair<double, double> p2_ = {processed_path_points_list[i + 1].first, processed_path_points_list[i + 1].second};

        double cdis_ = sqrt(pow(p1_.first - p2_.first,2 ) + pow(p1_.second - p2_.second, 2));

        if(cdis_ > dis_) {

            process_pathpoints::insert_point_at(processed_path_points_list, i, dis_);

        }

        else if(cdis_ < dis_) {

            process_pathpoints::delete_point_at(processed_path_points_list, i);
            i--;

        }

    }

}


void SimpleSplines::clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg){

    ROS_INFO("Inside the clicked_pose_callback!\n");

    ROS_WARN("x_: %f y_: %f\n",msg->point.x, msg->point.y);

    ROS_WARN("x_: %f y_: %f\n", msg->point.x, msg->point.y);

    std::pair<double, double> pt_ = {msg->point.x, msg->point.y};

    vis_functions::publish_waypoint_(pt_, path_point_pub_, nh_, marker_id++);

    waypoints_.push_back(pt_);

    
    ROS_INFO("waypoints_.size(): %d\n", waypoints_.size());

    if(waypoints_.size() > 1) {

        path_points_.resize(0);
        processed_path_points_.resize(0);

        generate_complete_spline(waypoints_);
        vis_functions::publish_waypoint_array_(path_points_, path_point_array_pub_, nh_);
        
        
        ROS_WARN("path_points_.size(): %d\n", path_points_.size());
        ROS_WARN("BEFORE PROCESSING --- processed_path_points_.size(): %d\n", processed_path_points_.size());

        process_path_points(path_points_, processed_path_points_);

        ROS_WARN("AFTER PROCESSING --- processed_path_points_.size(): %d\n", processed_path_points_.size());
        vis_functions::publish_waypoint_array_(processed_path_points_, processed_path_point_array_pub_, nh_);
        
    }




}

SimpleSplines::SimpleSplines(ros::NodeHandle &nh) : nh_{nh}
{

    ROS_INFO("Inside the SimpleSplines constructor!\n");
    marker_id =0 ;
 
    point_sub_ = nh_.subscribe("clicked_point", 100, &SimpleSplines::clicked_pose_callback, this);
    path_point_pub_ = nh_.advertise<visualization_msgs::Marker>("path_points_", 1000, true);
    path_point_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("pathpoints_array", 1000, true);
    processed_path_point_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("processed_pathpoints_array", 1000, true);
    

}

int main(int argc, char **argv){

    ros::init(argc, argv, "simple_spline_node");
    ros::NodeHandle nh_{"simple_spline_nh"};

    SimpleSplines* spline_ = new SimpleSplines(nh_);
    //ROS_INFO("nh_: %c\n", nh_.getNamespace().c_str());

    ros::spin();

    return 0;
}