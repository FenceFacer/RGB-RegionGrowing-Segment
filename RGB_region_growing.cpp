#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std;
using namespace pcl;

int main()
{
    // Load PointCloud File
    // Create PointCloud Object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Load Local Pointcloud File(.txt)
    std::ifstream file("pointcloud//your_pointcloud.txt"); //Add your pointcloud here
    if (!file.is_open())
    {
        std::cerr << "Failed to open point cloud file." << std::endl;
        return -1;
    }

    std::string line;
    while (std::getline(file, line))
    {
        pcl::PointXYZRGB point;
        std::istringstream iss(line);
        if (!(iss >> point.x >> point.y >> point.z))
        {
            std::cerr << "Invalid point cloud data." << std::endl;
            return -1;
        }
        cloud->push_back(point);
    }
    file.close();
    cout << "File is readed!" << endl;

    // create a region-growing segment object
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> rg;
    rg.setInputCloud(cloud);
    rg.setMinClusterSize(1000);
    rg.setMaxClusterSize(1200000);
    rg.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
    rg.setNumberOfNeighbours(30);
    rg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); //Normal Smooth threshholds
    rg.setCurvatureThreshold(1.0);
    rg.setPointColorThreshold(10); // Point Color thresholds
    rg.setRegionColorThreshold(10); // Region Color thresholds

    // Set Seed indices
    std::vector<int> seed_indices;
    seed_indices.push_back(0);  // Set the first point as seed

    cout << "Segmenting..." << endl;
    // Segment
    std::vector<pcl::PointIndices> clusters;
    rg.extract(clusters);

    cout << "Outputing..." << endl;

    // output every cluster as a single document file
    for (int i = 0; i < clusters.size(); ++i)
    {
        std::string filename = "cluster//cluster_" + std::to_string(i + 1) + ".txt"; // output file path and name
        std::ofstream file(filename);

        // write corrdinate of every point in the cluster
        for (int j = 0; j < clusters[i].indices.size(); ++j)
        {
            int index = clusters[i].indices[j];
            pcl::PointXYZRGB point = cloud->points[index];
            file << point.x << " " << point.y << " " << point.z << endl;
        }

        file.close();
        std::cout << "Cluster " << i + 1 << " saved as " << filename << std::endl;
    }

    return 0;
}
