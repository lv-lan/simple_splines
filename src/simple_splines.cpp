#include <simple_splines/simple_splines.h>
#include <simple_splines/vis_functions.h>

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

    P_3 = (idx + 2 > (int)points_.size() - 1 ? P_2 : points_[idx + 2].second); //Setting the gradient to 0 for last waypoint in the path

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
void SimpleSplines::generate_spline(std::vector<std::pair<double, double> >&points_){

    ROS_INFO("Inside generate_spline_path_points function!\n");
    int sz_ = (int)points_.size(); 

    ROS_WARN("sz_: %d\n", sz_);

    for(int i = 0; i < sz_ -1  ; i++) {

        std::vector<double> v_ = generate_spline_patch_cofficients(points_, i);

        generate_spline_patch(v_, points_, i);        

    }


    ROS_WARN("waypoints_.size() --- Before processing: %d\n", waypoints_.size());
    vis_functions::publish_waypoint_array_(waypoints_, waypoint_array_pub_, nh_);
    
    process_path_points(waypoints_);
    ROS_WARN("waypoints_.size() --- After processing: %d\n", waypoints_.size());
    


}

/**
 * @brief Generates spline patch between two given waypoints
 * 
 * @param v_ 
 * @param points_ 
 * @param idx 
 */
void SimpleSplines::generate_spline_patch(const std::vector<double> &v_, const std::vector<std::pair<double, double> >&points_,  int idx){

    if(idx >= (int)points_.size() - 1) {

        ROS_ERROR("idx >= (int)points_.size() - 1\n");
        return;

    }

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

    for(double mu = 0 ; mu <= 1; mu += step_sz){

        double x_ = x0_ + mu * (x1_ - x0_);

        double y_ = a_3 * pow(mu, 3) + a_2 * pow(mu,2) + a_1 * pow(mu, 1) + a_0 ;

        ROS_INFO("x_: %f y_: %f\n", x_, y_);

        waypoints_.push_back({x_, y_}); 

    }

}

void SimpleSplines::insert_intermediate_point(std::vector<std::pair<double, double> > &waypoints_, const int idx, const double dis_) {

    if(idx == (int)waypoints_.size() - 1) {

        ROS_ERROR("idx == (int)waypoints.size() - 1!\n");
        return; 

    }

    double x1 = waypoints_[idx].first, x2 = waypoints_[idx + 1].first; 

    double y1 = waypoints_[idx].second , y2 = waypoints_[idx + 1].second;

    double theta_; 

    if(x1 == x2) {theta_ = acos(0);}

    else {theta_ = atan2((y2 - y1) ,(x2 - x1));}

    double x_, y_; 

    x_ = x1 + dis_ * cos(theta_);
    y_= y1 + dis_ * sin(theta_);

    std::pair<double, double> pt_{x_, y_};

    waypoints_.insert(waypoints_.begin() + idx, pt_);

}




void SimpleSplines::process_path_points(std::vector<std::pair<double, double> > &waypoints_){

    double dis_ = 0.1;

    int cnt_ = 0 ;

    int sz_ = (int)waypoints_.size(); 

    for(int i =0 ;i < sz_ - 1; i++) {

        std::pair<double, double> a_{waypoints_[i]}, b_{waypoints_[i + 1]};

        double grid_dis_ = sqrt(pow(a_.first - b_.first, 2) + pow(a_.second - b_.second, 2));

        if(grid_dis_  > dis_) {
            
            //ROS_WARN("prev_sz: %d\n", waypoints_.size());
            cnt_++;
            insert_intermediate_point(waypoints_, i, dis_);
            //ROS_WARN("new_sz: %d\n", waypoints_.size());
            
        }

    }

}

void SimpleSplines::clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg){

    ROS_INFO("Inside the clicked_pose_callback!\n");

    ROS_WARN("x_: %f y_: %f\n",msg->point.x, msg->point.y);

    ROS_WARN("x_: %f y_: %f\n", msg->point.x, msg->point.y);

    std::pair<double, double> pt_ = {msg->point.x, msg->point.y};

    vis_functions::publish_waypoint_(pt_, waypoint_pub_, nh_, marker_id++);

    path_points_.push_back(pt_);

    ROS_INFO("path_points.size(): %d\n", path_points_.size());

    if(path_points_.size() > 4) {

        generate_spline(path_points_);

    }

}

SimpleSplines::SimpleSplines(ros::NodeHandle &nh) : nh_{nh}
{

    ROS_INFO("Inside the SimpleSplines constructor!\n");
    marker_id =0 ;
 
    point_sub_ = nh_.subscribe("clicked_point", 100, &SimpleSplines::clicked_pose_callback, this);
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