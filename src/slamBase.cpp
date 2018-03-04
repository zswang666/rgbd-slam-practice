#include "slamBase.h"

// image2PointCloud
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS &camera)
{    
    // define pointer to point cloud
    PointCloud::Ptr cloud(new PointCloud);

    // iterate through depth image
    for(int y=0; y<depth.rows; y++)
    {
        for(int x=0; x<depth.cols; x++)
        {
            // get value at (y,x) in depth image
            ushort d = depth.ptr<ushort>(y)[x];
            // d may not exist, then skip
            if(d==0)
                continue;

            // project the point to 3D space
            PointT p;
            p.z = double(d) / camera.scale;
            p.x = (x-camera.cx) * p.z / camera.fx;
            p.y = (y-camera.cy) * p.z / camera.fy;

            // define color of projected point
            p.b = rgb.ptr<uchar>(y)[x*3];
            p.g = rgb.ptr<uchar>(y)[x*3+1];
            p.r = rgb.ptr<uchar>(y)[x*3+2];
            
            // add projected point to point cloud
            cloud->points.push_back(p);
        }
    }

    // set and save point cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

// point2dTo3d
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    // define 3D point
    cv::Point3f p;

    // conversion
    p.z = double(point.z) / camera.scale;
    p.x = (point.x-camera.cx) * p.z / camera.fx;
    p.y = (point.y-camera.cy) * p.z / camera.fy;

    return p;
}

void computeKeyPointsAndDesp(FRAME &frame, string detector, string descriptor)
{
    // define feature detector and descriptor extractor
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detector = cv::FeatureDetector::create(detector.c_str());
    _descriptor = cv::DescriptorExtractor::create(descriptor.c_str());
    if (!_detector || !_descriptor)
    {
        cerr << "Unknown detector or descriptor type!! " << detector << "," << descriptor << endl;
        return ;
    }

    // detect keypoints
    _detector->detect(frame.rgb, frame.kp);
    // compute descriptor
    _descriptor->compute(frame.rgb, frame.kp, frame.desp);

    return;
}

RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera,
                             string matcher, double good_match_threshold, int min_matches)
{
    vector<cv::DMatch> goodMatches;
    if(matcher == string("BF"))
    {
        // keypoints matching
        vector<cv::DMatch> matches;
        cv::BFMatcher _matcher;
        _matcher.match(frame1.desp, frame2.desp, matches);
        cout << "Find total " << matches.size() << " matches." << endl;

        // filter matching
        double minDis = 9999;
        for(size_t i=0; i<matches.size(); i++)
        {
            if(matches[i].distance < minDis)
                minDis = matches[i].distance;
        }
        if(minDis < 15) // minDis lower bound
            minDis = 15;
        for(size_t i=0; i<matches.size(); i++)
        {
            if(matches[i].distance < good_match_threshold*minDis)
                goodMatches.push_back(matches[i]);
        }
    }
    else if(matcher == string("FLANN"))
    {
        // keypoint matching using FLANN (Fast Library for Approximate Nearest Neighbors)
        // see http://www.coldvision.io/2016/06/27/object-detection-surf-knn-flann-opencv-3-x-cuda/
        // and https://github.com/opencv/opencv/issues/5937
        cv::FlannBasedMatcher _matcher(new cv::flann::LshIndexParams(20,10,2));
        vector<vector<cv::DMatch>> matches;
        _matcher.knnMatch(frame1.desp, frame2.desp, matches, 2); // find the best 2 matches for each descriptor

        for(size_t k=0; k<matches.size(); k++)
        {
            if( (matches[k][0].distance < good_match_threshold*(matches[k][1].distance)) &&
                ((int)matches[k].size() <= 2 && (int)matches[k].size() > 0))
            {
                goodMatches.push_back(matches[k][0]);
            }
        }
    }
    else
    {
        cerr << "No such matcher, " << matcher << endl;
        exit(0);
    }
    cout << "# of good matches = " << goodMatches.size() << endl;

    if(goodMatches.size() <= min_matches)
    {
        RESULT_OF_PNP result;
        result.inliers = -1;
        return result;
    }

    // get matched points
    vector<cv::Point3f> pts_obj; // matched 3D points of rgb1
    vector<cv::Point2f> pts_img; // matched 2D points of rgb2
    for(size_t i=0; i<goodMatches.size(); i++)
    {
        // in matches, query is rgb1 and train is rgb2
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.y)];
        if(d == 0)
            continue;
        pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));

        // convert 2D points with depth (u,v,d) to 3D points (x,y,z)
        cv::Point3f pt(p.x, p.y, d);
        cv::Point3f pd = point2dTo3d(pt, camera);
        pts_obj.push_back(pd);
    }

    // solve pnp
    cout << "solving PnP" << endl;
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

    // pack up results
    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen(cv::Mat &rvec, cv::Mat &tvec)
{
    // convert rvec to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // convert rotation matrix and tvec to transform matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0),
                                       tvec.at<double>(0,1),
                                       tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(0,1);
    T(2,3) = tvec.at<double>(0,2);

    return T;
}

// joinPointCloud
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME &newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    // convert frame to point cloud
    PointCloud::Ptr newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);

    // combine point cloud
    PointCloud::Ptr output(new PointCloud());
    pcl::transformPointCloud(*original, *output, T.matrix()); // project original cloud to new cloud coordinate
    *output += *newCloud; // integrate projected original cloud to new cloud

    return newCloud;
}

// filterPointCloud
void filterPointCloud(PointCloud::Ptr cloud, double gridsize)
{
    // filtering and sampling new cloud
    PointCloud::Ptr tmp(new PointCloud());
    static pcl::VoxelGrid<PointT> voxel; // voxel grid to adjust map resolution
    static pcl::PassThrough<PointT> pass; // z-direction filter for depth limitation of RGBD camera

    voxel.setLeafSize(gridsize, gridsize, gridsize);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 4.0); // eliminate points further than 4m

    voxel.setInputCloud(cloud);
    voxel.filter(*tmp);

    pass.setInputCloud(tmp);
    pass.filter(*cloud);
}

// joinPointCloud2
void joinPointCloud2(PointCloud::Ptr original, FRAME &newFrame, Eigen::Isometry3d T, 
                     CAMERA_INTRINSIC_PARAMETERS &camera, double gridsize)
{
    // convert frame to point cloud
    PointCloud::Ptr newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);

    // filtering and sampling new cloud
    filterPointCloud(newCloud, gridsize);

    // combine new cloud and original point cloud (update original)
    PointCloud::Ptr tmp(new PointCloud());
    pcl::transformPointCloud(*newCloud, *tmp, T.matrix());
    *original += *tmp;

    return;
}

// normofTransform
double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec))) + fabs(cv::norm(tvec));
}

// checkKeyframes
CHECK_RESULT checkKeyframes(FRAME &f1, FRAME &f2, g2o::SparseOptimizer &opt,
                            CHECK_KEYFRAME_PARAMETERS &k_params, bool is_loop)
{
    static g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    // compare two frames
    RESULT_OF_PNP result = estimateMotion(f1, f2, k_params.camera, k_params.matcher,
                                          k_params.good_match_threshold, k_params.min_matches);

    // check # of inliers
    if(result.inliers < k_params.min_inliers)
    {
        cout << "# of inliers: " << result.inliers << endl;
        return NOT_MATCHED;
    }

    // check whether motion is out-of-bound
    double norm = normofTransform(result.rvec, result.tvec);
    if(is_loop == false)
    {
        if(norm >= k_params.max_norm)
            return TOO_FAR_AWAY;
    }
    else
    {
        if(norm >= k_params.max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if(norm <= k_params.keyframe_threshold)
        return TOO_CLOSE;

    // add vertex to pose graph. view f2 as a new frame
    if(is_loop == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(f2.frameID);
        v->setEstimate(Eigen::Isometry3d::Identity());
        opt.addVertex(v);
    }

    // add edge to pose graph
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
    edge->vertices()[0] = opt.vertex(f1.frameID);
    edge->vertices()[1] = opt.vertex(f2.frameID);
    edge->setRobustKernel(robustKernel);
    // set information matrix
    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Identity();
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    edge->setInformation(information);
    // set measurement
    Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
    edge->setMeasurement(T.inverse());
    // add edge to pose graph
    opt.addEdge(edge);

    return KEYFRAME;
}

// checkNearbyLoops
void checkNearbyLoops(vector<FRAME> &frames, FRAME &currentFrame, g2o::SparseOptimizer &opt, 
                      CHECK_KEYFRAME_PARAMETERS &k_params, int nearby_loops)
{
    // check "nearby_loops"'th closest frames from current frame
    if(frames.size() > nearby_loops)
    {
        for(size_t i=frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes(frames[i], currentFrame, opt, k_params, true);
        }
    }
    else // no enough keyframes, only check existed ones
    {
        for(size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes(frames[i], currentFrame, opt, k_params, true);
        }
    }
}

// checkRandomLoops
void checkRandomLoops(vector<FRAME> &frames, FRAME &currentFrame, g2o::SparseOptimizer &opt,
                      CHECK_KEYFRAME_PARAMETERS &k_params, int random_loops)
{
    // set seed
    srand((unsigned int) time(NULL));

    // randomly compare with "random_loops" frames and check if there is a loop
    if(frames.size() > random_loops)
    {
        for(int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes(frames[index], currentFrame, opt, k_params, true);
        }
    }
    else // no enough keyframes, only check existed ones
    {
        for(int i=0; i<frames.size(); i++)
        {
            checkKeyframes(frames[i], currentFrame, opt, k_params, true);
        }
    }
}
