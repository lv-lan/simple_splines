#ifndef SPLINE_HEADER
#define SPLINE_HEADER


#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

//#define SPACING 1.0

class SimpleSplines
{

    public:
        SimpleSplines(ros::NodeHandle &nh);
        

    private:
        
        std::vector<double> generate_spline_patch_cofficients(const std::vector<std::pair<double,double> > &points_, int idx);

        void generate_spline(std::vector<std::pair<double, double> >&points_);
        void generate_spline_patch(const std::vector<double> &v_, const std::vector<std::pair<double, double> >&points_,  const int idx);  

        void clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg);
        void process_path_points(std::vector<std::pair<double, double> > &waypoints_);
        void insert_intermediate_point(std::vector<std::pair<double, double> > &waypoints_, const int idx, const double dis_    ) ;
        void publish_point(const std::pair<double, double> pt_);
        
        ros::NodeHandle nh_;
        std::vector<std::pair<double, double> > path_points_;
        ros::Subscriber point_sub_, gen_path_flag_sub_;
        ros::Publisher waypoint_pub_, waypoint_array_pub_, waypoint_array_pub_modified_;
        int marker_id ;
        int gen_path_flag;
        std::vector<std::pair<double, double> > waypoints_;


};

#endif


