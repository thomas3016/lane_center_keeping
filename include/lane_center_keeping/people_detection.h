/*
 * people_detection.h
 *
 *  Created on: Oct 26, 2016
 *      Author: aicrobo
 */

#ifndef INCLUDE_LANE_CENTER_KEEPING_PEOPLE_DETECTION_H_
#define INCLUDE_LANE_CENTER_KEEPING_PEOPLE_DETECTION_H_

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<stdio.h>
#include<ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

namespace lane_center_keeping
{
  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;
  class PeopleDetection
  {
  public:
    PeopleDetection(ros::NodeHandle nh,ros::NodeHandle private_nh);
     ~ PeopleDetection();

  public:
    void processData(const VPointCloud::ConstPtr &scan);

    private:
      // Parameters that define the grids and the height threshold
        // Can be set via the parameter server
        int grid_dim_;
        double m_per_cell_;
        double height_diff_threshold_;
        bool full_clouds_;

        // Point clouds generated in processData
        VPointCloud obstacle_cloud_;
        VPointCloud clear_cloud_;

        visualization_msgs::Marker people_marker;
        // ROS topics
        ros::Subscriber velodyne_scan_;
        ros::Publisher obstacle_publisher_;
        ros::Publisher clear_publisher_;

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;
        ros::Publisher marker_pub_;
        ros::Publisher clusteredPub_;

        ros::Publisher object_pub_;

    public:
       struct OBSTACLE_OBJECT
       {
         Eigen::Vector3f center_position;
         float length=0;
         float width=0;
         float height=0;
         size_t num_of_points=0;


       };
private:
       std::vector<OBSTACLE_OBJECT> obstacle_object_;




  };
}

#endif /* INCLUDE_LANE_CENTER_KEEPING_PEOPLE_DETECTION_H_ */
