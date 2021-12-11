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

        void generate_complete_spline(const std::vector<std::pair<double, double> >&points_);
        void generate_spline_patch(std::vector<std::pair<double, double> >&path_points_list_, std::vector<double> &cofficients_, const std::vector<std::pair<double, double> >&waypoint_list_,  int idx);
        void clicked_pose_callback(const geometry_msgs::PointStampedConstPtr &msg);
        void publish_point(const std::pair<double, double> pt_);
        void process_path_points(const std::vector<std::pair<double, double> > &path_points_list, std::vector<std::pair<double, double> > &processed_path_points_list);
        void insert_point_at(std::vector<std::pair<double, double> >&processes_points_list, const int &idx, const double &dis_);
        void delete_point_at(std::vector<std::pair<double, double> > &processed_path_points_list, const int &idx);
        
        ros::NodeHandle nh_;
        std::vector<std::pair<double, double> > path_points_, processed_path_points_;
        ros::Subscriber point_sub_, gen_path_flag_sub_;
        ros::Publisher path_point_pub_, path_point_array_pub_, processed_path_point_array_pub_;
        int marker_id ;
        int gen_path_flag;
        std::vector<std::pair<double, double> > waypoints_;


};

#endif



