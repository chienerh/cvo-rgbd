/* ----------------------------------------------------------------------------
 * Copyright 2019, Tzu-yuan Lin <tzuyuan@umich.edu>, Maani Ghaffari <maanigj@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   data_type.h
 *  @author Tzu-yuan Lin, Maani Ghaffari 
 *  @brief  Data type definition
 *  @date   September 18, 2019
 **/
#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <tbb/concurrent_vector.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define PYR_LEVELS 3
#define NUM_FEATURES 5
#define NUM_CLASS 2

namespace cvo{

typedef std::vector<Eigen::Vector3f> cloud_t;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_row;

struct camera_info{
    float scaling_factor;    // scaling factor for depth data
    float fx;  // focal length x
    float fy;  // focal length y
    float cx;  // optical center x
    float cy;  // optical center y
};

struct frame{

    int frame_id;

    int h;        // height of the image without downsampling
    int w;        // width of the image without downsampling

    cv::Mat image;
    cv::Mat image_hsv;
    cv::Mat intensity;
    cv::Mat depth;
    MatrixXf_row semantic_labels;

    Eigen::Vector3f* dI;    // flattened image gradient, (w*h,3). 0: magnitude, 1: dx, 2: dy
    Eigen::Vector3f* dI_pyr[PYR_LEVELS];  // pyramid for dI. dI_pyr[0] = dI
    float* abs_squared_grad[PYR_LEVELS];  // pyramid for absolute squared gradient (dx^2+dy^2)
    float avg_abs_squared_grad;
};

struct point_cloud{

    int num_points;

    cloud_t positions;  // points position. x,y,z
    Eigen::Matrix<float, Eigen::Dynamic, NUM_FEATURES> features;   // features are rgb dx dy

    Eigen::Matrix<float, Eigen::Dynamic, NUM_CLASS> labels;

    // for visualization
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_dso_cloud;
};



}

#endif // DATA_TYPE_H