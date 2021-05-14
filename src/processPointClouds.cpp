// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout <<" "<< cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true); //true代表保留立方体内的点
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion); //the resultant filtered point cloud dataset

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true); //去除车顶周围点云
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true); //删除索引内的点
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // 定义障碍物点云和平面点云
    typename pcl::PointCloud<PointT>::Ptr groundCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT> ());
    // 提取平面点云
    for (int index : inliers->indices){
        groundCloud->points.push_back(cloud->points[index]);
    }
    // 提取障碍物点云，设置输入点云，设置内点索引，设置提取外点，提取障碍物点云
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacleCloud);
    // 返回点云结果
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud,groundCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    
    // 创建分割对象
    pcl::SACSegmentation<PointT> seg;
    // 创建分割时所需的模型系数对象及存储内点的点索引集合对象inlier
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients (true);
    //必选配置，设置分割的模型类型，所用的随机参数估计方法，最大迭代次数，距离阈值，输入点云
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    // 引发分割实现，并存储分割结果到点集合inliers及存储平面模型的系数coefficients
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // 构建kd树
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);//设置近邻搜索的半径
    ec.setMinClusterSize(minSize);//设置一个聚类需要的最少点数目
    ec.setMaxClusterSize(maxSize);//设置一个聚类需要的最多点数目
    ec.setSearchMethod(tree);//设置点云搜索机制
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);//从点云中提取聚类，并将索引保存在clusterIndices中
    
    
    for(pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        
        for(int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = std::chrono::steady_clock::now();
//     // TODO:: Fill in this function to find inliers for the cloud.
//     pcl::PointIndices::Ptr inliersIndice (new pcl::PointIndices);
//     std::unordered_set<int> inliersResult;

//     while(maxIterations--){

//         std::unordered_set<int> inliers;

//         while(inliers.size() < 3){
//             inliers.insert(rand()%(cloud->points.size()));
//         }

//         float x1,y1,z1,x2,y2,z2,x3,y3,z3;
//         auto iter = inliers.begin();
//         x1 = cloud->points[*iter].x;
//         y1 = cloud->points[*iter].y;
//         z1 = cloud->points[*iter].z;
//         iter++;
//         x2 = cloud->points[*iter].x;
//         y2 = cloud->points[*iter].y;
//         z2 = cloud->points[*iter].z;
//         iter++;
//         x3 = cloud->points[*iter].x;
//         y3 = cloud->points[*iter].y;
//         z3 = cloud->points[*iter].z;

//         float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
//         float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
//         float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
//         float D = -(A*x1+B*y1+C*z1);

//         for(int index =0 ; index < cloud->points.size(); index++){
            
//             if(inliers.count(index) > 0){
//                 continue;
//             }

//             float x = cloud->points[index].x;
//             float y = cloud->points[index].y;
//             float z = cloud->points[index].z;
//             float distance = fabs(A*x+B*y+C*z+D)/sqrt(A*A+B*B+C*C);
            
//             if (distance <= distanceTol){
//                 inliers.insert(index);
//             }
//         }

//         if(inliers.size()> inliersResult.size()){
//             inliersResult = inliers;
//         }
//     }

//     for(int point : inliersResult)
//     {
//         inliersIndice->indices.push_back(point);
//     }

//     if (inliers->indices.size () == 0)
//     {
//         std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
//     }

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

//     return segResult;
// }

template <typename PointT>
void clusterHelper(int indice,typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>& cluster,std::vector<bool>& processed,KdTree* tree, float distanceTol)
{
    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(cloud->points[indice],distanceTol);
    for(int id : nearest)
    {
        if(!processed[id])
            clusterHelper(id,cloud,cluster,processed,tree,distanceTol);
    }
}

// template<typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {
//     // Time clustering process
//     auto startTime = std::chrono::steady_clock::now();

//     std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//     std::vector<std::vector<int>> clusters_indice;
// 	std::vector<bool> processed (cloud->points.size(), false);
//     // 构建kd树
//     KdTree* tree = new KdTree;
//     for (int i = 0; i < points.size(),i++)
//     {
//         tree->insert(point[i],i)
//     }
//     // 聚类,并提取索引
//     for(int i = 0; i < cloud->point.size();i++)
//     {
//         if(processed[i]){
//             continue;
//         }
//         std::vector<int> cluster;
//         clusterHelper(i,cloud,cluster,processed,tree,clusterTolerance);
//         clusters_indice.push_back(cluster);
//     }
//     // 提取聚类点云
//     for(std::vector<int> clusterIndex : clusters_indice)
//     {
//         typename plc::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
//         for(int i :clusterIndex)
//         {
//             cloudCluster->points.push_back(cloud->points[i])
//         }
//         cloudCluster->width = cloudCluster.size();
//         cloudCluster->height = 1;
//         cloudCluster->is_dense = true;
//         clusters.push_back(cloudCluster);
//     }

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

//     return clusters;
// }