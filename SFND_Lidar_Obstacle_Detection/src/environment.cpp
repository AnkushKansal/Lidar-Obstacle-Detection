/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
	bool renderobstacle = false;
	bool renderplane = false;
	bool render_cluster = true;
	bool render_box = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor on heap.
    Lidar* LidarPtr = new Lidar(cars, 0);    
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = LidarPtr->scan();

    //renderRays(viewer, LidarPtr->position, pointcloud); //to generate rays

	if(renderScene)
		renderPointCloud(viewer, pointcloud, "Ankush Point Cloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> cloudpointProcessor;  

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = cloudpointProcessor.SegmentPlane(pointcloud, 100, 0.2);
    if(renderplane)
		renderPointCloud(viewer, segmentedCloud.first,"obstacleCloud",Color(1,0,0));
	if(renderobstacle)
		renderPointCloud(viewer, segmentedCloud.second,"planeCloud",Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = cloudpointProcessor.Clustering(segmentedCloud.first, 1.0, 3, 30);

	int clusterId = 0;
	std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

	//For each obstacle cluster we render
	for (auto cluster : cloudClusters)
	{
		if (render_cluster)
		{
			std::cout << "cluster size ";
			cloudpointProcessor.numPoints(cluster);
			renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
		}
		
		if (render_box)
		{
			Box box = cloudpointProcessor.BoundingBox(cluster);
			renderBox(viewer, box, clusterId, colors[clusterId], .2);
		}		
		++clusterId;
	}  
}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------


    bool renderobstacle = false;
    bool renderplane = true;
    bool render_cluster = true;
    bool render_box = true;
   
    // Experiment with the ? values and find what works best
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud;
    filteredCloud = pointProcessorI->FilterCloud(inputCloud, .2, Eigen::Vector4f(-10, -6.5, -3, 1), Eigen::Vector4f(30, 6.5, 10, 1));
    //renderPointCloud(viewer, filteredCloud, "filterCloud"); // downsampled data

    //Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    if (renderobstacle)
        renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud", Color(1, 0, 0));
    if (renderplane)
        renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(0, 1, 0));

    //Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedCloud.first, 0.5, 20, 1000);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    //For each obstacle cluster we render
    for (auto cluster : cloudClusters)
    {
        if (render_cluster)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[0]);
        }

        if (render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId, colors[2], .8);
        }
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	// ----------------------------------------------------
	// -----Open 3D viewer and display City Block     -----
	// ----------------------------------------------------


    bool renderobstacle = false;
    bool renderplane = true;
    bool render_cluster = true;
    bool render_box = true;

	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Experiment with the ? values and find what works best
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud;
    filteredCloud = pointProcessorI->FilterCloud(inputCloud, .3 , Eigen::Vector4f(-30, -6.5, -3, 1), Eigen::Vector4f(30, 6.5, 10, 1));
    //renderPointCloud(viewer, filteredCloud, "filterCloud"); // downsampled data

    //Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    if (renderobstacle)
        renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud", Color(1, 0, 0));
    if (renderplane)
        renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(0, 1, 0));

    //Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedCloud.first, 0.5, 20, 1000);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    //For each obstacle cluster we render
    for (auto cluster : cloudClusters)
    {
        if (render_cluster)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[0]);
        }

        if (render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId, colors[2], .8);
        }
        ++clusterId;
    }
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //simpleHighway(viewer);
    //cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    //stream the pcd file from the mentioned directory. streamPCD has a directory iterator.
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}