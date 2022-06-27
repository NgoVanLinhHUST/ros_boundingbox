/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
int countVehicle = 0;
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void rawPcdViewer(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    renderPointCloud(viewer, inputCloud, "inputCloud");
}
// City Block enviroment with  real PCD stream data
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //renderPointCloud(viewer,inputCloud,"inputCloud");

    //This is enable what kind of segmentation and cluster
    bool render_filterCloud = false; //enable render view filter cloud
    bool render_ransac = false;       //enable render obstacle
    bool render_clusters = true;    //enable render cluster
    bool render_box = true;         //enable bouding box in clustering




// Rotation if data difference 45*

    ///////////////////////
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 0 meters on the x axis.
  transform_2.translation() <<0.0, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis, rotation -pi/2 
  transform_2.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ())); //*******//***********// -0.7 if rotation 45* 

  // Print the transformation
//   printf ("\nMethod #2: using an Affine3f\n");
//   std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*inputCloud, *transformed_cloud, transform_2);


  //////////////////////////





  ////////////////////////

// void pcl::ConditionBase< PointT >::addCondition (Ptr  condition); 	
// build the condition
// build range condition range_cond  -10 < y < 3 ... remove another
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZI> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::GT, -15)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::LT, -2.5)));
//// build range condition range_cond2   3 < y < 10 ... remove other 
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond2 (new
        pcl::ConditionAnd<pcl::PointXYZI> ());
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::GT, 2.5)));
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZI> ("y", pcl::ComparisonOps::LT, 15)));

/// addCondition range_cond or range_cond2 = range_cond3 : -10 < y < 3 or 3 < y < 10 
pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond3 (new
                  pcl::ConditionOr<pcl::PointXYZI> ());
    range_cond3->addCondition(range_cond);
    range_cond3->addCondition(range_cond2);
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZI> condrem;

  condrem.setCondition (range_cond3);
  condrem.setInputCloud (transformed_cloud);
  condrem.setKeepOrganized (true);



      // apply filter
  condrem.filter (*transformed_cloud);
 //     cout << "hallooooo addcondition to remove___" << endl;
/////////////////////////////////////////////////////////////
  






    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(transformed_cloud, 0.8  , Eigen::Vector4f(-20., -13., -100., 1), Eigen::Vector4f(20., 13., 100., 1));
    if (render_filterCloud)
        renderPointCloud(viewer, filterCloud, "filterCloud");

    // Create point processor using Ransac Plane
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>{}; // must specify the typename on both sides (DON'T FORGET THE ONE AFTER `new`!)
    //SegmentPlane( cloud, maxIterations, distanceThreshold)
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlaneScratch(filterCloud, 100, 0.3);
    if (render_ransac)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }
    // change clusterTolerance / minSize , maxSize to change object cluster
    //// Create point processor using Ransac Clustering
    // Clustering(cloud, clusterTolerance, minSize, maxSize)
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->ClusteringScratch(segmentCloud.first, 1.0, 4, 200);  // 1 4 200
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "Cluster size";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId]);
            // if( numPoints(cluster) > 50)
            // {
            //     countVehicle++;
            //     std::cout << " ============== " << countVehicle << " ++++++++++++++";
            // }
        }
        if (render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
    if (render_clusters)
    {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }
}

// Simple Highway enviroment with simulation lidar
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    //generate poit cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //render lidar to see
    //renderRays(viewer, lidar-> position, inputCloud);

    //render point cloud
    renderPointCloud(viewer, inputCloud, "inputCloud");

    //This is enable what kind of segmentation and cluster
    bool render_ransac = false;  //enable render obstacle
    bool render_clusters = true; //enable render cluster
    bool render_box = true;      //enable bouding box in clustering

    // Create point processor using Ransac Plane
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>{}; // must specify the typename on both sides (DON'T FORGET THE ONE AFTER `new`!)
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.3);
    if (render_ransac)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }

    //// Create point processor using Ransac Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30); // 1.0 3 30
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "Cluster size";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId, Color(0, 0, 1));
        }
        ++clusterId;
    }
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);



    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(10.0);
}

int main(int argc, char **argv)
{ //Debug parameter addPcdRawViewer= true to open raw PCD
    bool addPcdRawViewer = true;

    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    //Add more viewer scence to debug
   
    pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("3D Viewer Raw PCD "));
    CameraAngle setAngle2 = TopDown;
    initCamera(setAngle2, viewer2);


    //Stream PCD function
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/MPTVKD");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    //Simple Highway function, uncomment for viewing
    //simpleHighway(viewer);

    //Real PCD with City Block, uncomment for statics viewing
    //cityBlock(viewer);

    while (!viewer->wasStopped())
    {
        //Stream PCD function
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        if (addPcdRawViewer)
        {
            viewer2->removeAllPointClouds();
            viewer2->removeAllShapes();
        }

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        cityBlock(viewer, pointProcessorI, inputCloudI);
        if (addPcdRawViewer)
        {
            rawPcdViewer(viewer2, inputCloudI);
        }
            

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
        //viewer->spinOnce();
    }
}