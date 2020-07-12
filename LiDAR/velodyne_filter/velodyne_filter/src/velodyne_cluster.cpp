#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_filter/PointArray.h"
#include <geometry_msgs/Point.h>

// PCL specific includes

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>


ros::Publisher pub;
ros::Publisher cluster_array_pub_;
ros::Publisher cluster_center_pub_;
ros::Publisher cluster_min_pub_;
ros::Publisher cluster_max_pub_;


typedef pcl::PointXYZ PointT;

bool print_fps_;

float x_axis_min_;
float x_axis_max_;

float y_axis_min_;
float y_axis_max_;

float z_axis_min_;
float z_axis_max_;

float z_axis_angle_;

int cluster_size_min_;
int cluster_size_max_;

uint32_t cluster_array_seq_ = 0;
uint32_t center_array_seq_ = 0;
uint32_t min_array_seq_ = 0;
uint32_t max_array_seq_ = 0;

std::string frame_id_;


int frames; clock_t start_time; bool reset = true;//fps



void roi_angle (const geometry_msgs::Point z_axis)
{
  z_axis_angle_ = z_axis.z * 3.14 / 180.0;
  ROS_INFO("z angle : %f", z_axis_angle_);
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);
 
  // Data containers used
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  
  pcl::VoxelGrid<PointT> vg;
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  vg.setInputCloud (cloud.makeShared());
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;



// crop box filter
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);

  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud(cloud_filtered);

  cropFilter.setMin(Eigen::Vector4f(x_axis_min_, y_axis_min_, z_axis_min_, 0));
  cropFilter.setMax(Eigen::Vector4f(x_axis_max_, y_axis_max_, z_axis_max_, 0));

  cropFilter.setTranslation(Eigen::Vector3f(0.0, 0.0, 0.0));
  cropFilter.setRotation(Eigen::Vector3f(0.0, 0.0, z_axis_angle_));
  cropFilter.filter(*cloud_filtered2);


  pcl::PointCloud<PointT>::Ptr cloud_ROI_alpha (new pcl::PointCloud<PointT>);

  // 오브젝트 생성 
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud_filtered2);            //입력 
  sor.setMeanK (50);                    //분석시 고려한 이웃 점 수(50개)
  sor.setStddevMulThresh (1.0);         //Outlier로 처리할 거리 정보 
  sor.filter (*cloud_ROI_alpha);         // 필터 적용 
  

    // Object for storing the plane model coefficients.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


  // 오프젝트 생성 Create the segmentation object.
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud (cloud_ROI_alpha);      //입력 
  seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
  seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
  seg.setMaxIterations (1000);               //최대 실행 수
  seg.setDistanceThreshold (0.01);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  //seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
  seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 


  //추정된 평면 파라미터 출력 (eg. ax + by + cz + d = 0 ).
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;


  pcl::PointCloud<PointT>::Ptr cloud_ROI (new pcl::PointCloud<PointT>);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_ROI_alpha);
  extract.setIndices (inliers);
  extract.setNegative (true);//false
  extract.filter (*cloud_ROI);


  // Creating the KdTree object for the search method of the extraction
 
  // 탐색을 위한 KdTree 오브젝트 생성
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_ROI); //KdTree 생성

  std::vector<pcl::PointIndices> cluster_indices; // 군집화된 결과물의 Index 저장
  // 군집화 오브젝트 생성
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.25); // 25cm
  ec.setMinClusterSize (100); // 최소 포인트 수
  ec.setMaxClusterSize (25000); // 최대 포인트 수
  ec.setSearchMethod (tree); // 위에서 정의한 탐색 방법 지정
  ec.setInputCloud (cloud_ROI); // 입력
  ec.extract (cluster_indices); // 군집화 적용

  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  std::vector<PointT> center_point;
  

  // 클러스터별 정보 수집, 출력, 저장
  std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > clusters;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        cluster->points.push_back(cloud_ROI->points[*pit]);


        pcl::PointXYZ pt = cloud_ROI->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);

            TotalCloud.push_back(pt2);
    }
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
    j++;
  }

  velodyne_filter::PointArray cluster_array;
  velodyne_filter::PointArray center_array;
  velodyne_filter::PointArray min_array;
  velodyne_filter::PointArray max_array;


  for(int i =0; i<clusters.size(); i++){
    //if(cluster_array_pub_.getNumSubscribers()>0){
      sensor_msgs::PointCloud2 ros_pc2_out;
      pcl::toROSMsg(*clusters[i], ros_pc2_out);
      cluster_array.clusters.push_back(ros_pc2_out);
    //}
    //if(cluster_center_pub_.getNumSubscribers() > 0){
      Eigen::Vector4f centroid;

      Eigen::Vector4f min_p;
      Eigen::Vector4f max_p;

      pcl::compute3DCentroid(*clusters[i], centroid);

      pcl::getMinMax3D(*clusters[i], min_p, max_p);
    

      geometry_msgs::Point center_point;
      center_point.x = centroid[0];
      center_point.y = centroid[1];
      center_point.z = centroid[2];

      geometry_msgs::Point min_point;
      min_point.x = min_p[0];
      min_point.y = min_p[1];
      min_point.z = min_p[2];

      geometry_msgs::Point max_point;
      max_point.x = max_p[0];
      max_point.y = max_p[1];
      max_point.z = max_p[2];



      ROS_INFO("===================================");

      ROS_INFO("cluster : %d", i); 
      ROS_INFO("center x position : %lf", center_point.x); 
      ROS_INFO("center y position : %lf", center_point.y); 
      ROS_INFO("center z position : %lf", center_point.z); 

      ROS_INFO("===================================");

      ROS_INFO("min x position : %lf", min_point.x); 
      ROS_INFO("min y position : %lf", min_point.y); 
      ROS_INFO("min z position : %lf", min_point.z); 

      ROS_INFO("===================================");

      ROS_INFO("max x position : %lf", max_point.x); 
      ROS_INFO("max y position : %lf", max_point.y); 
      ROS_INFO("max z position : %lf", max_point.z); 



      center_array.points.push_back(center_point);
      min_array.points.push_back(min_point);
      max_array.points.push_back(max_point);
    //}
  }


  if(cluster_array.clusters.size()) {
    cluster_array.header.seq = ++cluster_array_seq_;
    cluster_array.header.stamp = ros::Time::now();
    cluster_array.header.frame_id = frame_id_;
    cluster_array_pub_.publish(cluster_array);
  }

  if(center_array.points.size()) {
    center_array.header.seq = ++center_array_seq_;
    center_array.header.stamp = ros::Time::now();
    center_array.header.frame_id = frame_id_;
    cluster_center_pub_.publish(center_array);
  }
  
  if(min_array.points.size()) {
    min_array.header.seq = ++min_array_seq_;
    min_array.header.stamp = ros::Time::now();
    min_array.header.frame_id = frame_id_;
    cluster_min_pub_.publish(min_array);
  }

    if(max_array.points.size()) {
    max_array.header.seq = ++center_array_seq_;
    max_array.header.stamp = ros::Time::now();
    max_array.header.frame_id = frame_id_;
    cluster_max_pub_.publish(max_array);
  }
  
    // Convert To ROS data type 
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output);

  output.header.frame_id = "velodyne";   

  pub.publish(output); 



  if(print_fps_)if(++frames>10){std::cerr<<"[adaptive_clustering] fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}//fps


  ROS_INFO("published it."); 
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "velodyne_cluster");
 // ros::NodeHandle nh;



  ros::NodeHandle private_nh("~");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = private_nh.subscribe ("/velodyne_points", 1, cloud_cb);
  ros::Subscriber sub_angle = private_nh.subscribe ("/z_axis_angle", 1, roi_angle);


  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("pclplaneoutput", 1);
  pub = private_nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cluster", 1);

  cluster_array_pub_ = private_nh.advertise<velodyne_filter::PointArray>("clusters", 100);
  cluster_center_pub_ = private_nh.advertise<velodyne_filter::PointArray>("centers", 100); 
  cluster_min_pub_ = private_nh.advertise<velodyne_filter::PointArray>("min", 100); 
  cluster_max_pub_ = private_nh.advertise<velodyne_filter::PointArray>("max", 100); 
  // parameters
  private_nh.param<std::string>("frame_id", frame_id_, "velodyne");
  private_nh.param<bool>("print_fps", print_fps_, false);

  private_nh.param<float>("x_axis_min", x_axis_min_, -2.5);
  private_nh.param<float>("x_axis_max", x_axis_max_, 0.0);

  private_nh.param<float>("y_axis_min", y_axis_min_, 0.0);
  private_nh.param<float>("y_axis_max", y_axis_max_, 2.0);

  private_nh.param<float>("z_axis_min", z_axis_min_, -2.0);
  private_nh.param<float>("z_axis_max", z_axis_max_, 2.0);

  private_nh.param<float>("z_axis_angle_", z_axis_angle_, 0.0);


  private_nh.param<int>("cluster_size_min", cluster_size_min_, 50);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 20000);

  // Spin
  ros::spin ();
}
