/*
 * parameters_list.h
 *
 *  Created on: Sep 28, 2016
 *      Author: aicrobo
 */

#ifndef INCLUDE_LANE_CENTER_KEEPING_PARAMETERS_LIST_H_
#define INCLUDE_LANE_CENTER_KEEPING_PARAMETERS_LIST_H_
#include<string>
namespace lane_center_keeping
{

    /**
     * laser scan - parameters
     */
    const std::string FRAME_ID_PARAM        = "frame_id";
    const std::string MIN_RANGE_PARAM       = "min_range";
    const std::string MAX_RANGE_PARAM       = "max_range";
    const std::string ANGULAR_RES_PARAM     = "angular_res";
    const std::string RADIAL_RES_PARAM      = "radial_res";
    const std::string RING_ANGULAR_RES_PARAM     = "ring_angular_res";
    const std::string MAX_HEIGHT_DIFF_PARAM = "max_height_diff";
    const std::string MIN_HEIGHT_THRESHOLD_PARAM = "min_height_threshold";
    const std::string HEIGHT_DIFF_PARAM = "height_diff";
    const std::string ROAD_WIDTH="road_width";
}



#endif /* INCLUDE_LANE_CENTER_KEEPING_PARAMETERS_LIST_H_ */


