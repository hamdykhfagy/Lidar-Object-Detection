// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);
    

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    
    std::vector<int> indices;
    
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    for (int index : indices)
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloudRegion);
    ex.setIndices(inliers);
    ex.setNegative(true);
    ex.filter(*cloudRegion);
    
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

        // TODO: Create two new point clouds, one cloud with obstacles and other
    // with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud{ new pcl::PointCloud<PointT> };
    typename pcl::PointCloud<PointT>::Ptr roadCloud{ new pcl::PointCloud<PointT> };

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*roadCloud);

    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(
        obstacleCloud, roadCloud);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    auto startTime{ std::chrono::steady_clock::now() };
    // TODO: Fill in this function
    while (maxIterations--)
    {
        std::unordered_set<int> inliersSet;
        while (inliersSet.size() < 3)
            inliersSet.insert(rand() % (cloud->points.size()));

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliersSet.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        
        //calculating the normal vector
        PointT normal;
        normal.x = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        normal.y = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        normal.z = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - y1);

        float a, b, c, d;
        a = normal.x;
        b = normal.y;
        c = normal.z;
        d = -(a * x1 + b * y1 + c * z1);

        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliersSet.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            float x4, y4, z4;
            x4 = point.x;
            y4 = point.y;
            z4 = point.z;

            float dist = fabs((a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c));

            if (dist <= distanceThreshold)
                inliersSet.insert(index);
        }

        if (inliersResult.size() < inliersSet.size())
            inliersResult = inliersSet;
    }
   
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices() };
    for (int index : inliersResult)
        inliers->indices.push_back(index);
    
    if (cloud->points.empty()) { std::cout << "Failed to segment points." << std::endl; }

    auto endTime{ std::chrono::steady_clock::now() };
    auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };

    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult{
        SeparateClouds(inliers, cloud)
    };
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree <PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);


    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
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



/************

my euclidean algorith

**************/

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    KdTree tree;
    
    for (int j = 0; j < cloud->points.size(); j++)
    {
        tree.insert(&(cloud->points[j]), j);
    }

    std::vector<bool> processed(cloud->points.size(), false);
    int i = 0;
    while (i < cloud->points.size())
    {
        if (processed[i])
        {
            i++;
            continue;
        }

        std::vector<int> clusterIDs;
        clusterHelper(i, cloud, clusterIDs, processed, tree, clusterTolerance);
        if ((clusterIDs.size() >= minSize) && (clusterIDs.size() <= maxSize))
        {
            //typename pcl::PointCloud<PointT>::Ptr cluster;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
            for (int index : clusterIDs)
                cloudCluster->points.push_back(cloud->points[index]);
            clusters.push_back(cloudCluster);
        }
        i++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr& cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree& tree, float distanceTol)
{
    processed[indice] = true;
    cluster.push_back(indice);
    std::vector<int> nearbyPoints = tree.search(&(cloud->points[indice]), distanceTol);
    for (int index : nearbyPoints)
        if (!processed[index])
            clusterHelper(index, cloud, cluster, processed, tree, distanceTol);

}


/********************************/

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
        PCL_ERROR ("Couldn't read file \n");
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