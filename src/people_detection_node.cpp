/*
 * people_detection_node.cpp
 *
 *  Created on: Oct 26, 2016
 *      Author: aicrobo
 */

/*

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<velodyne_pointcloud/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VCloud;
typedef pcl::PointCloud<VPoint>::ConstPtr VCloudConstPtr;
typedef pcl::PointCloud<VPoint>::Ptr VCloudPtr;

typedef pcl::PointXYZ TPoint;
typedef pcl::PointCloud<TPoint> TCloud;
typedef pcl::PointCloud<TPoint>::ConstPtr TCloudConstPtr;
typedef pcl::PointCloud<TPoint>::Ptr TCloudPtr;

// Parameters that define the grids and the height threshold
// Can be set via the parameter server
int grid_dim_=320;
double m_per_cell_=320;
double height_diff_threshold_=0.3;
TCloud::ConstPtr pcl_in_;

bool recv_flag=false;

void processData(const TCloud::ConstPtr &scan)
{
  pcl_in_=scan;
  recv_flag=true;
}
void constructFullClouds(TCloud&obstacle_cloud)
{
      float min[grid_dim_][grid_dim_];
      float max[grid_dim_][grid_dim_];
      bool init[grid_dim_][grid_dim_];
      memset(&init, 0, grid_dim_ * grid_dim_);

      size_t obs_count=0;
      size_t npoints = pcl_in_->points.size();
       obstacle_cloud.points.resize(npoints);


      // build height map
      for (unsigned i = 0; i < npoints; ++i)
      {
        int x = ((grid_dim_ / 2) + pcl_in_->points[i].x / m_per_cell_);
        int y = ((grid_dim_ / 2) + pcl_in_->points[i].y / m_per_cell_);
        if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_)
        {
          if (!init[x][y])
          {
            min[x][y] = pcl_in_->points[i].z;
            max[x][y] = pcl_in_->points[i].z;
            init[x][y] = true;
          }
          else
          {
            min[x][y] =std::min(min[x][y], pcl_in_->points[i].z);
            max[x][y] =std::max(max[x][y], pcl_in_->points[i].z);
          }
        }
      }
        // display points where map has height-difference > threshold
        for (unsigned i = 0; i < npoints; ++i)
        {
           int x = ((grid_dim_/2)+pcl_in_->points[i].x/m_per_cell_);
           int y = ((grid_dim_/2)+pcl_in_->points[i].y/m_per_cell_);
           if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y])
            {
               if(((max[x][y] - min[x][y] > height_diff_threshold_)&&(max[x][y] - min[x][y]<2.3)) )
               {
                  obstacle_cloud.points[obs_count].x = pcl_in_->points[i].x;
                  obstacle_cloud.points[obs_count].y = pcl_in_->points[i].y;
                  obstacle_cloud.points[obs_count].z = pcl_in_->points[i].z;
                  obs_count++;
                }
             }
         }
        obstacle_cloud.points.resize(obs_count);
    }

int main(int argc,char**argv)
{

   ros::init(argc,argv,"people_detection_node");
   ros::NodeHandle nh;
   ros::param::get("~GRID_DIM",grid_dim_);
   ros::param::get("~M_PER_CELL",m_per_cell_);
   ros::param::get("~HEIGHT_DIFF_THRESHOLD",height_diff_threshold_);


   ros::Subscriber people_detection = nh.subscribe("ground_cloud", 10,
                                                   processData,
                                    ros::TransportHints().tcpNoDelay(true));
   ros::Publisher marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker", 1, true);
   ros::Publisher obstacle_publisher_ = nh.advertise<TCloud>("velodyne_obstacles",1);
   ros::Rate loop_rate(10);

   while(ros::ok())
   {
     if(recv_flag)
     {
       //ROS_INFO("New msg");
       recv_flag=false;
       TCloudPtr obstacle_cloud_ptr(new TCloud);
       constructFullClouds(*obstacle_cloud_ptr);
       // pass along original time stamp and frame ID
       obstacle_cloud_ptr->header.stamp = pcl_in_->header.stamp;
       obstacle_cloud_ptr->header.frame_id =pcl_in_->header.frame_id;
       if (obstacle_publisher_.getNumSubscribers() > 0)
       obstacle_publisher_.publish(*obstacle_cloud_ptr);




     }
     ros::spinOnce();
     loop_rate.sleep();
   }
  return 0;
}

*/


#include <ros/ros.h>

#include "lane_center_keeping/people_detection.h"

/** Main entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "heightmap_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create height map class, which subscribes to velodyne_points
  lane_center_keeping::PeopleDetection hm(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}

























