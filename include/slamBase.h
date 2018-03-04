#pragma once

// C++ standard library
#include <vector>
using namespace std;

// PCL library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp> // this must be included after Eigen

// g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// local library
#include "parameterReader.h"

/****************************************************
**                   Data Type                     **
****************************************************/
// define point cloud data type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera intrinsic parameters
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

// frame structure
struct FRAME
{
    int frameID;
    cv::Mat rgb, depth; // rgb and depth image
    cv::Mat desp; // descriptor
    vector<cv::KeyPoint> kp; // keypoints
};

// results of PnP
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

// results of checking two frames
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

// parameters for checking keyframes
struct CHECK_KEYFRAME_PARAMETERS
{
    int min_inliers; // minimum number of inliers
    double max_norm; // maximum motion norm for new frame
    double max_norm_lp; // maximum motion norm for loop closing checking
    double keyframe_threshold; // minimum motion norm (avoid too close)
    string matcher; // BF, FLANN 
    double good_match_threshold; // threshold to consider a match as a good match
    int min_matches; // minimum good matches to consider 2 frames as matched
    CAMERA_INTRINSIC_PARAMETERS camera; // camera intrinsic
};

/**************************************************************
**                      Function                             **
**************************************************************/
// image2PointCloud: convert rgb+depth to point cloud
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS &camera);

// point2dTo3d: convert a 2D point (y,x,d) to a 3D point
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);

// computeKeyPointsAndDesp: extract keypoints and compute descriptors
void computeKeyPointsAndDesp(FRAME &frame, string detector, string descriptor);

// estimateMotion: compute motion between two frames
RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera,
                             string matcher, double good_match_threshold, int min_matches);

// cvMat2Eigen: convert rvec+tvec to transform matrix
Eigen::Isometry3d cvMat2Eigen(cv::Mat &rvec, cv::Mat &tvec);

// joinPointCloud: join new point cloud (from newFrame) to the original point cloud (not update)
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME &newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS &camera);

// filterPointCloud: use voxel grid and pass to filter point cloud 
void filterPointCloud(PointCloud::Ptr cloud, double gridsize);

// jointPointCloud2: join new point cloud to the original one in a updating fashion, with filtering
void joinPointCloud2(PointCloud::Ptr original, FRAME &newFrame, Eigen::Isometry3d T,
                     CAMERA_INTRINSIC_PARAMETERS &camera, double gridsize);

// normofTransform: compute norm for transform (rvec+tvec)
double normofTransform(cv::Mat rvec, cv::Mat tvec);

// checkKeyframes: for keyframe selection
CHECK_RESULT checkKeyframes(FRAME &f1, FRAME &f2, g2o::SparseOptimizer &opt, 
                            CHECK_KEYFRAME_PARAMETERS &k_params, bool is_loop=false);

// checkNearbyLoops: check nearby loop
void checkNearbyLoops(vector<FRAME> &frames, FRAME &currentFrame, g2o::SparseOptimizer &opt,
                      CHECK_KEYFRAME_PARAMETERS &k_params, int nearby_loops);

// checkRandomLoops: check random loop
void checkRandomLoops(vector<FRAME> &frames, FRAME &currentFrame, g2o::SparseOptimizer &opt,
                      CHECK_KEYFRAME_PARAMETERS &k_params, int random_loops);

// obtain camera intrinsic parameters from parameter reader
inline static CAMERA_INTRINSIC_PARAMETERS getCameraIntrinsic(ParameterReader &pd)
{
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof(pd.getData("camera.fx").c_str());
    camera.fy = atof(pd.getData("camera.fy").c_str());
    camera.cx = atof(pd.getData("camera.cx").c_str());
    camera.cy = atof(pd.getData("camera.cy").c_str());
    camera.scale = atof(pd.getData("camera.scale").c_str());

    return camera;
}
