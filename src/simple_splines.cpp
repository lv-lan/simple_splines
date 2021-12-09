#include <simple_splines/simple_splines.h>
#include <simple_splines/vis_functions.h>
#include <std_msgs/Bool.h>

std::vector<double> SimpleSplines::generate_spline_patch_cofficients(const std::vector<std::pair<double,double> > &points_, int idx){

    if(idx < 1 || (idx + 1) > (int)points_.size() - 2) {

        ROS_ERROR("idx < 1 || idx > (int)points_.size() - 2\n");
        return std::vector<double>{};

    }

    
    double P_0, P_1, P_2, P_3; 

    double P1_dash, P2_dash; 

    P_0  = points_[idx- 1].second, P_1 = points_[idx].second, P_2 = points_[idx+ 1].second, P_3 = points_[idx + 2].second;
    
    P1_dash = (P_2 - P_0)/2.0;
    
    P2_dash = (P_3 - P_1)/2.0;

    //P = a_3 * mu^3 + a_2 * mu^2 + a_1 * mu  + a_0 

    double a_0, a_1, a_2, a_3 ; 

    a_0  = P_1;
    a_1 =  P1_dash;

    a_3 = P2_dash - 2 * P_2 + 2 * a_0 + a_1;
    a_2 = P_2 - (a_0 + a_1 + a_3);

    return {a_0, a_1, a_2, a_3};

}


void SimpleSplines::generate_spline_path_points(const std::vector<std::pair<double, double> >&points_){

    ROS_INFO("Inside generate_spline_path_points function!\n");
    int sz_ = (int)points_.size(); 

    if(sz_ < 4) {

        ROS_ERROR("sz< 4 -- returning!\n");
        return;

    }

    for(int i = 1; i <= sz_ - 3; i++) {

        std::vector<double> v_ = generate_spline_patch_cofficients(points_, i);

        generate_spline_patch_points(v_, points_, i);        

    }

    vis_functions::publish_waypoint_array_(waypoints_, waypoint_array_pub_, nh_);


}

void SimpleSplines::generate_spline_patch_points(const std::vector<double> &v_, const std::vector<std::pair<double, double> >&points_,  int idx){

    ROS_INFO("Generating spline_patch_points!\n");

    double a_0, a_1, a_2, a_3; 
    a_0 = v_[0]; 
    a_1 = v_[1]; 
    a_2 = v_[2]; 
    a_3 = v_[3];

    ROS_WARN("a_0: %f a_1: %f a_2: %f a_3: %f\n", a_0, a_1, a_2, a_3);

    double step_sz = 0.1;

    std::pair<double, double> P_1 = points_[idx], P_2 = points_[idx+ 1];

    ROS_WARN("P1: (%f,%f) P2: (%f,%f)\n", P_1.first, P_1.second, P_2.first, P_2.second);

    double x0_ = P_1.first, x1_ = P_2.first;  

    //std::vector<std::pair<double, double> > waypoints_;

    for(double mu = 0 ; mu <= 1; mu += step_sz){

        double x_ = x0_ + mu * (x1_ - x0_);

        //double y_ = a_3 * pow(x_, 3) + a_2 + pow(x_,2) + a_1 * pow(x_, 1) + a_0 ;

        double y_ = a_3 * pow(mu, 3) + a_2 * pow(mu,2) + a_1 * pow(mu, 1) + a_0 ;

        //double y_ = a_3 * pow(x_, 3) + a_2 + pow(x_,2) + a_1 * pow(x_, 1) + a_0 ;

        ROS_WARN("x_: %f y_: %f\n", x_, y_);

        waypoints_.push_back({x_, y_}); 

    }

    //ROS_INFO("waypoints_.size(): %d\n", waypoints_.size());

   // vis_functions::publish_waypoint_array_(waypoints_, waypoint_array_pub_, nh_);

}

//Subscriber Callbacks

void SimpleSplines::clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg){

    ROS_INFO("Inside the clicked_pose_callback!\n");

    ROS_WARN("x_: %f y_: %f\n",msg->point.x, msg->point.y);

    ROS_WARN("x_: %f y_: %f\n", msg->point.x, msg->point.y);

    std::pair<double, double> pt_ = {msg->point.x, msg->point.y};

    vis_functions::publish_waypoint_(pt_, waypoint_pub_, nh_, ++marker_id);

    path_points_.push_back(pt_);

    ROS_INFO("path_points.size(): %d\n", path_points_.size());

    if(path_points_.size() > 4) {

        generate_spline_path_points(path_points_);

    }

    /*if(gen_path_flag) {

        generate_spline_path_points(path_points_);
        gen_path_flag = 0 ;
    
    }*/

}

/*void SimpleSplines::gen_path_flag_callback(const std_msgs::Bool *msg) {

    if(msg->data == true) {

        gen_path_flag = true; 
    
    }
    
}*/

SimpleSplines::SimpleSplines(ros::NodeHandle &nh) : nh_{nh}
{

    ROS_INFO("Inside the SimpleSplines constructor!\n");
    marker_id =0 ;
    gen_path_flag = 0 ;

    point_sub_ = nh_.subscribe("clicked_point", 100, &SimpleSplines::clicked_pose_callback, this);
    //gen_path_flag_sub_ = nh_.subscribe("/gen_path_flag", 100, &SimpleSplines::gen_path_flag_callback, this);
    waypoint_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoints_array", 1000, true);
    waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoints_", 1000, true);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "simple_spline_node");
    ros::NodeHandle nh_{"simple_spline_nh"};

    SimpleSplines* spline_ = new SimpleSplines(nh_);
    //ROS_INFO("nh_: %c\n", nh_.getNamespace().c_str());

    ros::spin();

    return 0;
}