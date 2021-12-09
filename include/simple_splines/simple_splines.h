#ifndef SPLINE_HEADER
#define SPLINE_HEADER


#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>


class SimpleSplines
{

    public:
        SimpleSplines(ros::NodeHandle &nh);
        

    private:
        
        void process_path_points();
        std::vector<double> generate_spline_patch_cofficients(const std::vector<std::pair<double,double> > &points_, int idx);

        void generate_spline_path_points(const std::vector<std::pair<double, double> >&points_);
        void generate_spline_patch_points(const std::vector<double> &v_, const std::vector<std::pair<double, double> >&points_,  const int idx);  

        //void gen_path_flag_callback(const std_msgs::Bool *msg);
        void clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg);

        ros::NodeHandle nh_;
        std::vector<std::pair<double, double> > path_points_;
        ros::Subscriber point_sub_, gen_path_flag_sub_;
        ros::Publisher waypoint_pub_, waypoint_array_pub_;
        int marker_id ;
        int gen_path_flag;
        std::vector<std::pair<double, double> > waypoints_;


};

#endif


