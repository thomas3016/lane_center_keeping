/*
 * lane_centerline_detection.h
 *
 *  Created on: Sep 28, 2016
 *      Author: aicrobo
 */

#ifndef INCLUDE_LANE_CENTER_KEEPING_LANE_CENTERLINE_DETECTION_H_
#define INCLUDE_LANE_CENTER_KEEPING_LANE_CENTERLINE_DETECTION_H_

#include<string>
#include<iostream>
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include<velodyne_pointcloud/point_types.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>

#include<pcl/kdtree/kdtree.h>
#include<pcl/segmentation/extract_clusters.h>


#include <sensor_msgs/PointCloud2.h>


#include<tf/message_filter.h>
#include<tf/transform_listener.h>
#include <message_filters/subscriber.h>

#include<std_msgs/Bool.h>


// Include template implementations to transform a custom point cloud
#include<pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


#include<pcl/visualization/pcl_visualizer.h>

#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<eigen3/Eigen/src/Core/Matrix.h>
#include<lane_center_keeping/LaneKalmanFilter.h>

#include<lane_center_keeping/lane_kalman_filter.h>



typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VCloud;
typedef pcl::PointCloud<VPoint>::ConstPtr VCloudConstPtr;
typedef pcl::PointCloud<VPoint>::Ptr VCloudPtr;

typedef pcl::PointXYZ TPoint;
typedef pcl::PointCloud<TPoint> TCloud;
typedef pcl::PointCloud<TPoint>::ConstPtr TCloudConstPtr;
typedef pcl::PointCloud<TPoint>::Ptr TCloudPtr;

#define PI 3.14159265

namespace lane_center_keeping
{
  class LaneCenterlineDetection
  {
    public:
        LaneCenterlineDetection(ros::NodeHandle nh,ros::NodeHandle private_nh);
         ~LaneCenterlineDetection()
         {
           if(lkf!=NULL)
           {
             lkf->~LaneKalmanFilter();
             lkf=NULL;
           }
           if(lkfs!=NULL)
           {
             lkfs->~CurbEdgeKalmanFilter();
             lkfs=NULL;
           }
         }

    public:
        virtual void process_cityroad(const sensor_msgs::PointCloud2::ConstPtr&cloud);
        virtual bool curbFilter(const VCloud::ConstPtr&cloud_in,TCloud&clout_out,pcl::ModelCoefficients&coefficients);
        virtual void line_fit(const TCloud::ConstPtr&cloud,float &a,float &b,float&min_z);
        //virtual void curve_line_fit(const TCloud::ConstPtr&cloud,Eigen::Vector3f&coefficient,float &min_z);
        virtual float median3(const float a,const float b,const float c);
    public:
    //!Configuration parameters
        struct Params
        {
          //! Target frame ID
          //! - An empty value means to use the same frame ID as the input point cloud has...
          std::string frame_id;
          double min_range;
          //! The maximum radius/distance from the center [m]
          double max_range;
          //! Angular resolution [degrees]
          double angular_res;
          //! Radial resolution [m/cell]
          double radial_res;

          double ring_angular_res;

          //! Min-max height difference threshold[m]
          double max_height_diff;
          double min_height_threshold;
          double height_diff;
          Params() :
              frame_id(""),
              min_range(getDefaultMinRange()),
              max_range(getDefaultMaxRange()),
              angular_res(getDefaultAngularRes()),
              radial_res(getDefaultRadialRes()),
              ring_angular_res(getDefaultRingAngularRes()),
              max_height_diff(getDefaultMaxHeightDiff()),
              min_height_threshold(0.2),
              height_diff(0.3)
          {
          }
          //        static double getDefaultMinRange() { return 1.0; }
          static double getDefaultMinRange()
          {
            return 2.0;
          }
          static double getDefaultMaxRange()
          {
            return 50.0;
          }
          static double getDefaultAngularRes()
          {
            return 0.5;
          }
          static double getDefaultRadialRes()
          {
            return 0.25;
          }
          static double getDefaultRingAngularRes()
          {
              return 0.2;
          }
          static double getDefaultMaxHeightDiff()
          {
            return 0.2;
          }
        };


          //! Informations accumulated for each sampling/map bin
          struct PolarMapBin
          {
            //! Number of samples accumulated in the bin.
            unsigned n;
            //! Region index
            unsigned index;
            float vertice_z;
           float min_z,min_x,min_y;
           float max_z,max_x,max_y;
           float height_diff;
            //! Default constructor.
            PolarMapBin()
           :n(0)
           ,index(0)
           ,height_diff(0)
           ,min_x(std::numeric_limits<float>::max())
           ,min_y(std::numeric_limits<float>::max())
           ,min_z(std::numeric_limits<float>::max())
           ,max_x(std::numeric_limits<float>::min())
           ,max_y(std::numeric_limits<float>::min())
           ,max_z(std::numeric_limits<float>::min())
           ,vertice_z(std::numeric_limits<float>::max())
            {}
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          };

          struct RingMapBin
          {
            //! Average position.
            Eigen::Vector3d avg;
            double rad_avg;
            double rad_min;
            //! Helper values.
            Eigen::Vector3d sum;
            double rad_sum;
            //! Number of samples accumulated in the bin.
            unsigned n;

            double min_z;
            double max_z;
            //! Default constructor.
            RingMapBin() :
                n(0), avg(0, 0, 0), rad_avg(0), rad_min(std::numeric_limits<float>::max()), sum(0, 0, 0), rad_sum(0)
            {
            }
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          };

          //! Conversion to polar coordinates.
          void toPolarCoords(float x, float y, float& ang, float &mag)
          {
            static const float rad_to_deg = 180.0f / float(CV_PI);
            mag = std::sqrt(x*x + y*y);
            ang = std::atan2(y, x) * rad_to_deg;
          }
          //! Returns index of bin in the polar map by converting given polar coordinates [deg, m]
          void getPolarMapIndex(float ang, float mag, int& a, int& d)
          {
            a = int((ang + 180.001f) * inv_angular_res_) % num_of_angular_bins_;
            d = int(mag * inv_radial_res_) % num_of_radial_bins_;
          }
          //! Returns subscripted bin of the polar map.
          PolarMapBin& getPolarMapBin(int a, int d)
          {
            return polar_map_[d * num_of_angular_bins_ + a];
          }

          //! Returns index of bin in the polar map
          void getRingMapIndex(float ang, int ring, int& a, int& r)
          {
            a = int((ang + 180.001f) * inv_angular_rmap_res_) % num_of_angular_rmap_bins_;
            r = int(ring - min_ring_index_);
          }
          //! Returns subscripted bin of the polar histogram.
          RingMapBin& getRingMapBin(int a, int r)
          {
            return ring_map_[r * num_of_angular_rmap_bins_ + a];
          }

   private:
          ros::NodeHandle nh_, private_nh_;
          //! Parameters...
          //! Point cloud buffer to avoid reallocation on every message.
          VCloud pcl_in_;
          VCloud road_cloud_;

          ros::Publisher ground_pub_;
          ros::Publisher left_curb_pub_;
          ros::Publisher right_curb_pub_;
          ros::Publisher curb_info_pub_;

          ros::Publisher marker_pub_;
          visualization_msgs::Marker marker;

          ros::Subscriber raw_points_sub_;
          ros::Subscriber right_keeping_flag_sub_;
  private:
          //! Internal representation of a polar map
          typedef std::vector<PolarMapBin> tPolarMap;
          typedef std::vector<std::vector<float> >tPolarHeightMap;

          //! Map to avoid reallocation on every message
          tPolarMap polar_map_;
          tPolarHeightMap polar_height_map_;

          //! Internal representation of a polar histogram
           typedef std::vector<RingMapBin> tRingMap;
           //! Polar histogram
           tRingMap ring_map_;

          //! Current size of the polar map.
          int num_of_angular_bins_, num_of_radial_bins_;
          float inv_angular_res_, inv_radial_res_;

           //! Current size of the polar histogram.
           int num_of_angular_rmap_bins_;
           float inv_angular_rmap_res_;
           int min_ring_index_, max_ring_index_;
  private:
           Params params_;
           bool first_flag_;
           float road_width_;
           float right_keeping_flag_;

           float anchor_position_;
           LaneKalmanFilter*lkf;
           CurbEdgeKalmanFilter*lkfs;
  };
} //namespace

#endif /* INCLUDE_LANE_CENTER_KEEPING_LANE_CENTERLINE_DETECTION_H_ */
