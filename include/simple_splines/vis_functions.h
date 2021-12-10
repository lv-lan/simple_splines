#ifndef VIS_FUNCTIONS
#define VIS_FUNCTIONS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace vis_functions{

  
    void publish_waypoint_(const std::pair<double, double> &pt_, ros::Publisher &waypoint_pub_, ros::NodeHandle &nh_, int marker_id){

            visualization_msgs::Marker marker;
            marker.header.frame_id = "bi_tf/map";
            marker.header.stamp = ros::Time();
            marker.ns = nh_.getNamespace();

            marker.id = marker_id;  
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = pt_.first;
            marker.pose.position.y = pt_.second;
            marker.pose.position.z = 0;
            
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            
            marker.color.a = 1.0; // Don't forget to set the alpha!
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            waypoint_pub_.publish( marker );

    }
    
    void publish_waypoint_array_(const std::vector<std::pair<double, double> >&waypoints_, ros::Publisher &waypoint_array_pub_, ros::NodeHandle &nh_){

            visualization_msgs::MarkerArray marker_array;

            int marker_id = 0 ;
            
            for(int i =0 ; i < (int)waypoints_.size() ; i++) {

                visualization_msgs::Marker marker;
                marker.header.frame_id = "bi_tf/map";
                marker.header.stamp = ros::Time();
                marker.ns = nh_.getNamespace();

                marker.id = ++marker_id;  
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                
                marker.pose.position.x = waypoints_[i].first;
                marker.pose.position.y = waypoints_[i].second;
                marker.pose.position.z = 0;
                
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                
                marker.color.a = 1.0; // Don't forget to set the alpha!
                
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                marker_array.markers.push_back(marker);
            }

            waypoint_array_pub_.publish( marker_array );


    }


   
    
};

#endif

