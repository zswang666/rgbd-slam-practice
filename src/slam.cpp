// C++ standard library
#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
using namespace std;

// local library
#include "print_colors.h"
#include "slamBase.h"

// g2o
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

// readFrame: read a rgb+depth image given paths
FRAME readFrame(int index, string rgbPath, string depthPath);

// getImagePaths: get all image paths from a directory and then sort them
vector<string> getImagePaths(const string dirName, const string imgExtension);

// numericSmallerThan: compare numeric value of two strings
bool numericSmallerThan(const string &lhs, const string &rhs);

// main function
int main(int argc, char **argv)
{
    // read paramters
    if(argc!=2)
    {
        cerr << "argc must be 2 to specify parameter definition file." << endl;
        return -1;
    }
    ParameterReader pd(argv[1]);

    // get image paths
    vector<string> rgbPaths, depthPaths;
    rgbPaths = getImagePaths(pd.getData("rgb_dir"), pd.getData("rgb_extension"));
    depthPaths = getImagePaths(pd.getData("depth_dir"), pd.getData("depth_extension"));

    // get indices
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());
    endIndex = min(min(endIndex, (int)rgbPaths.size()), (int)depthPaths.size());

    // container for all keyframes
    vector<FRAME> keyframes;

    // initialize
    cout << "Initializing..." << endl;
    int currentIndex = startIndex;
    //FRAME currentFrame = readFrame(currentIndex, pd);
    FRAME currentFrame = readFrame(currentIndex, rgbPaths[currentIndex], depthPaths[currentIndex]);//
    // get keypoints, descriptors and convert to point cloud
    string detector = pd.getData("detector");
    string descriptor = pd.getData("descriptor");
    CAMERA_INTRINSIC_PARAMETERS camera = getCameraIntrinsic(pd);
    computeKeyPointsAndDesp(currentFrame, detector, descriptor);
    PointCloud::Ptr cloud = image2PointCloud(currentFrame.rgb, currentFrame.depth, camera);

    // initialize solver
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    g2o::SparseOptimizer globalOptimizer;
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.setVerbose(false);

    // add the first vertex to pose graph
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(currentIndex);
    v->setEstimate(Eigen::Isometry3d::Identity());
    v->setFixed(true); // fixed coordinate of the first frame as reference
    globalOptimizer.addVertex(v);

    // add the first frame to keyframes
    keyframes.push_back(currentFrame);

    // start SLAM process
    double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");
    int nearby_loops = atoi(pd.getData("nearby_loops").c_str());
    int random_loops = atoi(pd.getData("random_loops").c_str());
    CHECK_KEYFRAME_PARAMETERS k_params;
    k_params.min_inliers = atoi(pd.getData("min_inliers").c_str());
    k_params.max_norm = atof(pd.getData("max_norm").c_str());
    k_params.max_norm_lp = atof(pd.getData("max_norm_lp").c_str());
    k_params.keyframe_threshold = keyframe_threshold;
    k_params.camera = camera;
    k_params.matcher = pd.getData("matcher");
    k_params.good_match_threshold = atoi(pd.getData("good_match_threshold").c_str());
    k_params.min_matches = atoi(pd.getData("min_matcher").c_str());
    for(currentIndex=startIndex+1; currentIndex<endIndex; currentIndex++)
    {
        cout << "Reading files" << currentIndex << endl;
        //FRAME currentFrame = readFrame(currentIndex, pd);
        FRAME currentFrame = readFrame(currentIndex, rgbPaths[currentIndex], depthPaths[currentIndex]);//
        computeKeyPointsAndDesp(currentFrame, detector, descriptor);
        CHECK_RESULT result = checkKeyframes(keyframes.back(), currentFrame, globalOptimizer, k_params);
        switch(result)
        {
            case NOT_MATCHED:
                cout << RED"Not enough inliers." << endl;
                break;
            case TOO_FAR_AWAY:
                cout << RED"Too far away, may be an error" << endl;
                break;
            case TOO_CLOSE:
                cout << RESET"Too close, not a keyframe" << endl;
                break;
            case KEYFRAME:
                cout << GREEN"This is a new keyframe" << endl;
                // check loop closure
                if(check_loop_closure)
                {
                    checkNearbyLoops(keyframes, currentFrame, globalOptimizer, k_params, nearby_loops);
                    checkRandomLoops(keyframes, currentFrame, globalOptimizer, k_params, random_loops);
                }
                keyframes.push_back(currentFrame);
                break;
            default:
                cout << RESET"Not supposed to be here@@" << endl;
                break;
        }
        cout << RESET"" << endl;
    }

    // optimize pose graph
    cout << RESET"optimizing pose graph, # of vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.save("./results/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100); // optimization iteration
    globalOptimizer.save("./results/result_after.g2o");
    cout << "optimization done." << endl;

    // combine point cloud
    cout << "combining the point cloud map..." << endl;
    double gridsize = atof(pd.getData("voxel_grid").c_str());
    for(size_t i=0; i<keyframes.size(); i++)
    {
        // get one frame from g2o pose graph
        g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(globalOptimizer.vertex(keyframes[i].frameID));
        Eigen::Isometry3d pose = vertex->estimate(); // estimate after pgo
        
        // add to cloud (update original point cloud)
        joinPointCloud2(cloud, keyframes[i], pose, camera, gridsize);
    }
    filterPointCloud(cloud, gridsize);

    // save point cloud
    pcl::io::savePCDFile("./results/result.pcd", *cloud);
    cout << "Final map is saved" << endl;
    
    globalOptimizer.clear();

    return 0;
}

FRAME readFrame(int index, string rgbPath, string depthPath)
{
    FRAME f;
    // read rgb
    f.rgb = cv::imread(rgbPath);
    // read depth
    f.depth = cv::imread(depthPath, -1);
    // frame index
    f.frameID = index;

    return f;
}

vector<string> getImagePaths(const string dirName, const string imgExtension)
{
    // container for image path
    vector<string> imgPaths;

    // read images with specified extension from directory
    DIR *dir;
    dir = opendir(dirName.c_str());
    struct dirent *ent;
    if(dir != NULL)
    {
        while((ent = readdir(dir)) != NULL)
        {
            string fileName = ent->d_name;
            // see https://stackoverflow.com/questions/51949/how-to-get-file-extension-from-string-in-c
            size_t extStrIndex = fileName.find_last_of(".");
            bool checkExt = fileName.substr(extStrIndex+1) == imgExtension;
            if(checkExt)
            {
                string rawName = fileName.substr(0, extStrIndex);
                imgPaths.push_back(rawName);
            }
        }
        closedir(dir);
    }
    else
    {
        cerr << "No such directory " << dirName << endl; 
        exit(0);
    }

    // sort image paths, from small to large
    sort(imgPaths.begin(), imgPaths.end(), numericSmallerThan);

    // add directory and extension
    stringstream ss;
    for(size_t i=0; i<imgPaths.size(); i++)
    {
        ss.clear();
        ss << dirName << imgPaths[i] << "." << imgExtension;
        string fileName;
        ss >> fileName;
        imgPaths[i] = fileName;
    }

    return imgPaths;
}

// see https://bytes.com/topic/c/answers/131442-sort-strings-based-their-numeric-values
bool numericSmallerThan(const string &lhs, const string &rhs)
{
    return atoi(lhs.c_str()) < atoi(rhs.c_str());
}
