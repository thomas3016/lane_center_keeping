/*
 * people_detection.cpp
 *
 *  Created on: Oct 26, 2016
 *      Author: aicrobo
 */
#include "lane_center_keeping/people_detection.h"
#include<pcl/kdtree/kdtree.h>
#include<pcl/segmentation/extract_clusters.h>

#include<lane_center_keeping/obstacle_info_msg.h>

namespace lane_center_keeping
{

  PeopleDetection::PeopleDetection(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    // get parameters using private node handle
    priv_nh.param("cell_size", m_per_cell_, 0.3);
    priv_nh.param("full_clouds", full_clouds_, true);
    priv_nh.param("grid_dimensions", grid_dim_, 320);
    priv_nh.param("height_threshold", height_diff_threshold_, 0.25);

    // Set up publishers
    obstacle_publisher_ = nh.advertise<VPointCloud>("velodyne_obstacles",1);
    clear_publisher_ = nh.advertise<VPointCloud>("velodyne_clear",1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("marker_array", 1, true);
    object_pub_=nh.advertise<lane_center_keeping::obstacle_info_msg>("object",1,true);
    // subscribe to Velodyne data points
    velodyne_scan_ = nh.subscribe("ground_cloud", 1,
                                    &PeopleDetection::processData, this,
                                    ros::TransportHints().tcpNoDelay(true));
  }

  PeopleDetection::~PeopleDetection() {}

  /** point cloud input callback */
  void PeopleDetection::processData(const VPointCloud::ConstPtr &scan)
  {

   // if ((obstacle_publisher_.getNumSubscribers() == 0)
     //  && (clear_publisher_.getNumSubscribers() == 0))
     // return;

    // pass along original time stamp and frame ID
    obstacle_cloud_.header.stamp = scan->header.stamp;
    obstacle_cloud_.header.frame_id = scan->header.frame_id;

    // pass along original time stamp and frame ID
    clear_cloud_.header.stamp = scan->header.stamp;
    clear_cloud_.header.frame_id = scan->header.frame_id;

    // set the exact point cloud size -- the vectors should already have
    // enough space
    size_t npoints = scan->points.size();
    obstacle_cloud_.points.resize(npoints);
    //obstacle_cloud_.channels[0].values.resize(npoints);

    clear_cloud_.points.resize(npoints);
    //clear_cloud_.channels[0].values.resize(npoints);

    size_t obs_count=0;
    size_t empty_count=0;
    // either return full point cloud or a discretized version
    if (full_clouds_)
    {
    	 float min[grid_dim_][grid_dim_];
    	 float max[grid_dim_][grid_dim_];
    	 bool init[grid_dim_][grid_dim_];
    	 memset(&init, 0, grid_dim_*grid_dim_);

    	 for (unsigned i = 0; i < npoints; ++i)
    	 {
    	      int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    	      int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    	      if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_)
    	      {
    	        if (!init[x][y])
    	        {
    	          min[x][y] = scan->points[i].z;
    	          max[x][y] = scan->points[i].z;
    	          init[x][y] = true;
    	        } else
    	        {
    	          min[x][y] = std::min(min[x][y], scan->points[i].z);
    	          max[x][y] = std::max(max[x][y], scan->points[i].z);
    	        }
    	      }
    	    }
    	    // display points where map has height-difference > threshold

    	    for (unsigned i = 0; i < npoints; ++i)
    	    {
    	      int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    	      int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    	      if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y])
    	      {
    	        if ((max[x][y] - min[x][y] > height_diff_threshold_)&&(max[x][y] - min[x][y]<=3.5))
    	        {
    	          obstacle_cloud_.points[obs_count]=scan->points[i];
    	          obs_count++;
    	        }
    	        else
    	        {
    	          clear_cloud_.points[empty_count]=scan->points[i];
    	          empty_count++;
    	        }
    	      }
    	    }
    }
    
    else
    {
 
		float min[grid_dim_][grid_dim_];
		float max[grid_dim_][grid_dim_];
		float num_obs[grid_dim_][grid_dim_];
		float num_clear[grid_dim_][grid_dim_];
		bool init[grid_dim_][grid_dim_];

		//memset(&init, 0, grid_dim_*grid_dim_);

		for (int x = 0; x < grid_dim_; x++)
		{
			for (int y = 0; y < grid_dim_; y++)
			{
				init[x][y] = false;
				num_obs[x][y] = 0;
				num_clear[x][y] = 0;
			}
		}

		// build height map
		for (unsigned i = 0; i < npoints; ++i)
		{
			int x = ((grid_dim_ / 2) + scan->points[i].x / m_per_cell_);
			int y = ((grid_dim_ / 2) + scan->points[i].y / m_per_cell_);
			if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_)
			{
				if (!init[x][y])
				{
					min[x][y] = scan->points[i].z;
					max[x][y] = scan->points[i].z;
					num_obs[x][y] = 0;
					num_clear[x][y] = 0;
					init[x][y] = true;
				}
				else
				{
					min[x][y] = std::min(min[x][y], scan->points[i].z);
					max[x][y] = std::max(max[x][y], scan->points[i].z);
				}
			}
       }
    }

    obstacle_cloud_.points.resize(obs_count);
    clear_cloud_.points.resize(empty_count);


    obstacle_publisher_.publish(obstacle_cloud_);
    clear_publisher_.publish(clear_cloud_);
    if(obstacle_cloud_.points.size()<2)
    {

      lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
      object_msg->obstacle_property = 0;
      object_pub_.publish(object_msg);
      std::cout << "******************" << std::endl;
      return;
    }

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud(obstacle_cloud_.makeShared());
	pcl::EuclideanClusterExtraction < pcl::PointXYZI > ec;
	ec.setClusterTolerance(1);
	ec.setMinClusterSize(2);
	ec.setSearchMethod(tree);
	ec.setInputCloud(obstacle_cloud_.makeShared());
	ec.extract(cluster_indices);
	if(cluster_indices.size()==0)
	{
	     lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
	     object_msg->obstacle_property = 0;
	     object_pub_.publish(object_msg);
	     return;
	 }

	  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
	  {
		float min_z,max_z;
		Eigen::Vector3f sum(0.0, 0.0, 0.0);
		int count=0;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
		for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end(); pit++)
		{
			pcl::PointXYZI point = obstacle_cloud_.points[*pit];
			if(count==0)
			{
				min_z=point.z;
				max_z=point.z;
				count++;
			}
			else
			{

                min_z = std::min(min_z, point.z);
			    max_z = std::max(max_z, point.z);
			    count++;
			}
			//pcl::PointXYZI point = obstacle_cloud_.points[*pit];
			//min_z = std::min(min_z, point.z);
			//max_z = std::max(max_z, point.z);
			sum[0] = sum[0] + point.x;
			sum[1] = sum[1] + point.y;
			sum[2] = sum[2] + point.z;
			cloud_cluster->points.push_back(point);
		}
		size_t num_cluster_cloud = cloud_cluster->points.size();
		if(num_cluster_cloud==0)
	    continue;
		sum[0] = sum[0] / num_cluster_cloud;
		sum[1] = sum[1] / num_cluster_cloud;
		sum[2] = sum[2] / num_cluster_cloud;


		if (std::abs(sum[1]) > 1.8)
		{
			continue;
		}

		float diff_z = max_z - min_z;
		/*
		if ((diff_z > 2.1 ||diff_z<0.8|| max_z>0.2))
		{
			continue;
		}
		*/

       std::vector<int>count_point(5,0);
	   std::vector<float> max_xx(5);
	   std::vector<float> min_xx(5);
	   std::vector<float> max_yy(5);
	   std::vector<float> min_yy(5);
	   max_xx.clear();
	   min_xx.clear();
	   max_yy.clear();
	   min_yy.clear();

		float resolution = float(diff_z/5);
		for (size_t i = 0; i < cloud_cluster->points.size(); i++)
		{
			pcl::PointXYZI point = cloud_cluster->points[i];
			int index = int(((point.z-min_z)/resolution)+ 5)%5;
			if(count_point[index]==0)
			{
				min_xx[index]=point.x;
				max_xx[index]=point.x;
				min_yy[index]=point.y;
				max_yy[index]=point.y;
			    count_point[index]=count_point[index]+1;
			}
			else
			{
			   min_xx[index] = std::min(point.x, min_xx[index]);
		       max_xx[index] = std::max(point.x, max_xx[index]);
		       min_yy[index] = std::min(point.y, min_yy[index]);
		       max_yy[index] = std::max(point.y, max_yy[index]);
		       count_point[index]=count_point[index]+1;
			}

		}

		std::vector<float> diff_yy(5);
		std::vector<float> diff_xx(5);
		diff_xx.clear();
		diff_yy.clear();

		for (int i = 0; i < 5; i++)
		{
			if(count_point[i]!=0)
			{
				diff_yy.push_back(float(max_yy[i] - min_yy[i]));
				diff_xx.push_back(float(max_xx[i] - min_xx[i]));
			}
		}
		count_point.clear();
		std::sort(diff_yy.begin(), diff_yy.end());
		std::sort(diff_xx.begin(), diff_xx.end());

		std::cout << diff_yy[0] << "--" << diff_yy[1] << "--" << diff_yy[2]	<< "--" << diff_yy[3] << "--" << diff_yy[4] << std::endl;
        if(diff_yy.size()==0)
        {
        	continue;
        }
        int index=int(diff_yy.size()/2);
		OBSTACLE_OBJECT object;
		object.center_position = sum;
		object.height = diff_z;
		object.width = diff_yy[index];
		object.length = diff_xx[index];
		std::cout<<"object.width:"<<object.width<<"object.length"<<object.length<<"object.height"<<object.height<<std::endl;
		object.num_of_points = num_cluster_cloud;
		obstacle_object_.push_back(object);
		min_xx.clear();
		max_xx.clear();
		max_yy.clear();
		max_xx.clear();
		diff_yy.clear();
		diff_xx.clear();
		min_xx.swap(min_xx);
		max_xx.swap(max_xx);
		min_yy.swap(min_yy);
		max_yy.swap(max_yy);
		diff_yy.swap(diff_yy);
	 }

	  size_t num_of_obstacle=obstacle_object_.size();
	  std::cout<<"num_of_obstacle:"<<num_of_obstacle<<std::endl;
	  if(num_of_obstacle==0)
	  {
	      lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
	      object_msg->obstacle_property =0;
	      object_pub_.publish(object_msg);
	      return;
	   }

        OBSTACLE_OBJECT object;
        if (num_of_obstacle == 1)
        {
             object = obstacle_object_[0];
        }
        else
        {
             std::vector<float> index_x;
             index_x.clear();
             for (size_t i = 0; i < num_of_obstacle; i++)
              {
                  index_x.push_back(float(obstacle_object_[i].center_position[0]));
              }
              std::sort(index_x.begin(), index_x.end());
              object = obstacle_object_[index_x[0]];
              std::cout<<"object.height1:"<<object.height<<std::endl;
         }

         obstacle_object_.clear();
         obstacle_object_.swap(obstacle_object_);
         float position_x=object.center_position[0];
         std::cout<<"object.height"<<object.height<<std::endl;

         if(position_x>=20&&position_x<30&&object.num_of_points>10)
         {
              lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
               std::cout << "position_x>=20&&position_x<30" << std::endl;
               object_msg->position.x=object.center_position[0];
               object_msg->position.y=object.center_position[1];
               object_msg->position.z=object.center_position[2];
               object_msg->obstacle_property=0;
               object_pub_.publish(object_msg);

				return;
           }
         else if (position_x > 15 && position_x < 20 && object.num_of_points > 15)
         {
                  lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
                  std::cout << "position_x > 15 && position_x < 20" << std::endl;
                  object_msg->position.x = object.center_position[0];
                  object_msg->position.y = object.center_position[1];
                  object_msg->position.z = object.center_position[2];
                  object_msg->obstacle_property = 0;
                  object_pub_.publish(object_msg);
                  return;
         }
         else if(position_x>1.5&&position_x<=15)
         {

            float length=object.length;
            float width=object.width;
            std::cout<<"1"<<std::endl;
            std::cout<<"object.height"<<object.height<<std::endl;
            if(object.height>0.25&&object.height<0.8)
		    {
			      std::cout<<"2"<<std::endl;
				 if(std::abs(object.center_position[1])<1.8)
				 {
					lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
					std::cout << "######################position_x>1.5&&position_x<=15#######people############"<<width<< std::endl;
					object_msg->position.x=object.center_position[0];
					object_msg->position.y=object.center_position[1];
					object_msg->position.z=object.center_position[2];
					object_msg->obstacle_property=3;
					object_pub_.publish(object_msg);
					return;
				 }
		    }
		else if(object.height>=0.25&&object.height<0.8&&object.width>1&&object.width<=3.5)
		{
			std::cout<<"3"<<std::endl;
			if(std::abs(object.center_position[1])<1.2)
			{
				lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
				std::cout << "######################position_x>1.5&&position_x<=15#######people############"<<width<< std::endl;
				object_msg->position.x=object.center_position[0];
				object_msg->position.y=object.center_position[1];
				object_msg->position.z=object.center_position[2];
				object_msg->obstacle_property=3;
				object_pub_.publish(object_msg);
				return;
			}
		}

                //width=std::max(length,width);
            if(object.height<2.1&&object.height>1&&object.center_position[2]>=-1.5&&object.center_position[2]<-0.3)
            {
               if(width>0.15&&width<1.5&&std::abs(object.center_position[1])<1.8)
               {
                	  lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
                	  std::cout << "######################position_x>10&&position_x<=15#######people############"<<width<< std::endl;
                	  object_msg->position.x=object.center_position[0];
                	  object_msg->position.y=object.center_position[1];
                	  object_msg->position.z=object.center_position[2];
                	  object_msg->obstacle_property=1;
                	  object_pub_.publish(object_msg);
                	  return ;
               }
             }
            else if(object.height>=0.8&&object.height<3.5&&object.width>=1.5&&object.width<3.5&object.center_position[2]>-1.5)
            {
               if(std::abs(object.center_position[1])<1.4)
               {

                		lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
                		std::cout << "######################position_x>10&&position_x<=15#######people############"<<width<< std::endl;
                		object_msg->position.x=object.center_position[0];
                        object_msg->position.y=object.center_position[1];
                		object_msg->position.z=object.center_position[2];
                		object_msg->obstacle_property=2;
                		object_pub_.publish(object_msg);
                		return ;
                	}
                	if(std::abs(object.center_position[1])-object.width/2<0.9)
                	{
                		lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
                		std::cout << "######################position_x>10&&position_x<=15#######people############"<<width<< std::endl;
                		object_msg->position.x=object.center_position[0];
                	    object_msg->position.y=object.center_position[1];
                		object_msg->position.z=object.center_position[2];
                	    object_msg->obstacle_property=2;
                		object_pub_.publish(object_msg);
                		return ;
                	}

                }


             }
             else
             {
                  lane_center_keeping::obstacle_info_msgPtr object_msg(new lane_center_keeping::obstacle_info_msg);
                  object_msg->obstacle_property = 0;
                  object_pub_.publish(object_msg);
		          return;
              }
  }

} // namespace lane_center_keeping
