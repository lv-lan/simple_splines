#ifndef SPLINE_HEADER
#define SPLINE_HEADER


#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>


class SimpleSplines
{

    public:
        SimpleSplines(ros::NodeHandle &nh);
        void clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg);
        void process_path_points();
        void add_points_to_path_points(const geometry_msgs::PointStamped &point_);


    private:
        ros::NodeHandle nh_;
        std::vector<geometry_msgs::PointStamped> path_points_;
        ros::Subscriber point_sub_;


};

#endif


