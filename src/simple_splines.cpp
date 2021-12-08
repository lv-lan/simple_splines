#include <simple_splines/simple_splines.h>


void SimpleSplines::add_points_to_path_points(const geometry_msgs::PointStamped &point_){

    ROS_INFO("Old size: %d\n", path_points_.size());
    path_points_.push_back(point_);
    ROS_INFO("New size: %d\n", path_points_.size());
    

}

void SimpleSplines::process_path_points(){

    ROS_INFO("Inside the process_path_function!\n");

    ROS_INFO("End of the process_path_points function!\n");

}


void SimpleSplines::clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg){

    ROS_INFO("Inside the clicked_pose_callback!\n");

    //ROS_WARN("x_: %f y_: %f\n",msg->pose.position.x, msg->pose.position.y);

    ROS_WARN("x_: %f y_: %f\n", msg->point.x, msg->point.y);

    add_points_to_path_points(*msg);

    if((int)path_points_.size() >= 3) {

        process_path_points();

    }

}

SimpleSplines::SimpleSplines(ros::NodeHandle &nh) : nh_{nh}
{

    ROS_INFO("Inside the SimpleSplines constructor!\n");
    point_sub_ = nh_.subscribe("clicked_point", 100, &SimpleSplines::clicked_pose_callback, this);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "simple_spline_node");
    ros::NodeHandle nh_{"simple_spline_nh"};

    SimpleSplines* spline_ = new SimpleSplines(nh_);
    //ROS_INFO("nh_: %c\n", nh_.getNamespace().c_str());

    ros::spin();

    return 0;
}