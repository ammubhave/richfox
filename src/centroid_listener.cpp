#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "geometry_msgs/Point.h"
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include <pcl/point_types_conversion.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>

#include <visualization_msgs/Marker.h>
#include <cmath>

ros::Publisher pubCloud;
ros::Publisher pubCentroid;

ros::Publisher pubCloud2;
ros::Publisher pubCentroid2;

void PointCloudXYZHSVtoXYZRGB (pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out) {
   out.width = in.width;
   out.height = in.height;
   for (size_t i = 0; i < in.points.size (); i++) {
     pcl::PointXYZRGB p;
     pcl::PointXYZHSVtoXYZRGB (in.points[i], p);
     p.x = in.points[i].x;
     p.y = in.points[i].y;
     p.z = in.points[i].z;
     out.points.push_back (p);
   }
}



void pclCallback1(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud_raw;
  pcl::fromROSMsg(*input, cloud_raw);
  std::cout << cloud_raw.header.frame_id << std::endl;
  std::cout << input->header.frame_id << std::endl;

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloudXYZRGBtoXYZHSV(cloud_raw, *cloud_hsv);
 /* for (size_t i = 0; i < cloud_hsv->points.size (); ++i)
std::cerr << " " << cloud_hsv->points[i].h << " "
<< cloud_hsv->points[i].s << " "
<< cloud_hsv->points[i].v << std::endl;
*/
  pcl::PassThrough<pcl::PointXYZHSV> color_filter;

  color_filter.setInputCloud(cloud_hsv);
  color_filter.setFilterFieldName("h");
  //color_filter.setFilterLimits(82, 162);
  color_filter.setFilterLimits(100, 150);
  //color_filter.setFilterLimitsNegative(true);
  //color_filter.setFilterLimits(31, 44);
  color_filter.filter(*cloud_hsv);

  color_filter.setFilterFieldName("s");
// color_filter.setFilterLimits(0.2, 0.7);
  color_filter.setFilterLimits(0.3, 1);
  color_filter.setFilterLimitsNegative(false);
  //color_filter.setFilterLimits(0.34, 0.6);
  color_filter.filter(*cloud_hsv);

  color_filter.setFilterFieldName("v");
  color_filter.setFilterLimits(100, 255); //range from 0 to 255
 // color_filter.setFilterLimits(201, 360);
  color_filter.filter(*cloud_hsv);
 //cout << cloud_hsv->points.size () << endl;
  //ROS_INFO("cloud size: %d", cloud_hsv->points.size());

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_outlier_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZHSV> sor;
  sor.setInputCloud (cloud_hsv);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_outlier_filtered);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
  PointCloudXYZHSVtoXYZRGB(*cloud_outlier_filtered, *cloud_filtered_rgb);

// Code below placed in some callback function for subscribed point cloud cloud_input
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform("/base_link", input->header.frame_id, ros::Time(0), ros::Duration(10.0));
    // Look up transform between /base_link and point cloud frame
    // On PR2, point cloud frame is /head_mount_kinect_rgb_optical_frame
    listener.lookupTransform("/base_link", input->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Use pcl_ros to transform point cloud given TF transform
  // Both input and output clouds must be of type pcl::PointCloud<pcl::PointXYZRGB>
  // Some other point cloud types are supported, but NOT PointXYZHSV
  pcl_ros::transformPointCloud(*cloud_filtered_rgb, *cloud_transformed, transform);
  cloud_transformed->header.frame_id = "/base_link";
  Eigen::Vector4f centroid(0,0,0,0);
  int num;
  num = compute3DCentroid(*cloud_transformed, centroid);         
  //ROS_INFO("num used for centroid: %d", num);
  std::cout << centroid << std::endl;
  //ROS_INFO("cloud size: %d", cloud_transformed->points.size());
  geometry_msgs::Point pt;
  pt.x = centroid[0];
  pt.y = centroid[1];
  pt.z = centroid[2];

  pubCentroid.publish(pt);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_transformed, output);
  pubCloud.publish(output);
}
void pclCallback2(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud_raw;
  pcl::fromROSMsg(*input, cloud_raw);
  std::cout << cloud_raw.header.frame_id << std::endl;
  std::cout << input->header.frame_id << std::endl;

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloudXYZRGBtoXYZHSV(cloud_raw, *cloud_hsv);
 /* for (size_t i = 0; i < cloud_hsv->points.size (); ++i)
std::cerr << " " << cloud_hsv->points[i].h << " "
<< cloud_hsv->points[i].s << " "
<< cloud_hsv->points[i].v << std::endl;
*/
  pcl::PassThrough<pcl::PointXYZHSV> color_filter;

  color_filter.setInputCloud(cloud_hsv);
  color_filter.setFilterFieldName("h");
  //color_filter.setFilterLimits(82, 162);
  color_filter.setFilterLimits(200, 230);
  //color_filter.setFilterLimitsNegative(true);
  //color_filter.setFilterLimits(31, 44);
  color_filter.filter(*cloud_hsv);

  color_filter.setFilterFieldName("s");
// color_filter.setFilterLimits(0.2, 0.7);
  color_filter.setFilterLimits(0.3, 1);
  color_filter.setFilterLimitsNegative(false);
  //color_filter.setFilterLimits(0.34, 0.6);
  color_filter.filter(*cloud_hsv);

  color_filter.setFilterFieldName("v");
  color_filter.setFilterLimits(100, 255); //range from 0 to 255
 // color_filter.setFilterLimits(201, 360);
  color_filter.filter(*cloud_hsv);
 //cout << cloud_hsv->points.size () << endl;
  //ROS_INFO("cloud size: %d", cloud_hsv->points.size());

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_outlier_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZHSV> sor;
  sor.setInputCloud (cloud_hsv);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_outlier_filtered);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
  PointCloudXYZHSVtoXYZRGB(*cloud_outlier_filtered, *cloud_filtered_rgb);

// Code below placed in some callback function for subscribed point cloud cloud_input
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform("/base_link", input->header.frame_id, ros::Time(0), ros::Duration(10.0));
    // Look up transform between /base_link and point cloud frame
    // On PR2, point cloud frame is /head_mount_kinect_rgb_optical_frame
    listener.lookupTransform("/base_link", input->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Use pcl_ros to transform point cloud given TF transform
  // Both input and output clouds must be of type pcl::PointCloud<pcl::PointXYZRGB>
  // Some other point cloud types are supported, but NOT PointXYZHSV
  pcl_ros::transformPointCloud(*cloud_filtered_rgb, *cloud_transformed, transform);
  cloud_transformed->header.frame_id = "/base_link";
  Eigen::Vector4f centroid(0,0,0,0);
  int num;
  num = compute3DCentroid(*cloud_transformed, centroid);         
  //ROS_INFO("num used for centroid: %d", num);
  std::cout << centroid << std::endl;
  //ROS_INFO("cloud size: %d", cloud_transformed->points.size());
  geometry_msgs::Point pt;
  pt.x = centroid[0];
  pt.y = centroid[1];
  pt.z = centroid[2];

  pubCentroid2.publish(pt);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_transformed, output);
  pubCloud2.publish(output);
}

void pclCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pclCallback1(input);
  pclCallback2(input);
}


ros::Publisher marker_pub1;
void markerCallBack1(const geometry_msgs::Point::ConstPtr& centroid){
  visualization_msgs::Marker points;
  points.header.frame_id = "/base_link";
  points.header.stamp == ros::Time::now();
  points.ns = "points_and_lines";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = 0;

  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.08;
  points.scale.y = 0.08;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;
  points.points.push_back(*centroid);
  
  marker_pub1.publish(points);
}

ros::Publisher marker_pub2;
void markerCallBack2(const geometry_msgs::Point::ConstPtr& centroid){
  visualization_msgs::Marker points;
  points.header.frame_id = "/base_link";
  points.header.stamp == ros::Time::now();
  points.ns = "points_and_lines";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = 0;

  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.08;
  points.scale.y = 0.08;

  // Points are blue
  points.color.b = 1.0f;
  points.color.a = 1.0;
  points.points.push_back(*centroid);
  
  marker_pub2.publish(points);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_listener");
  ROS_INFO("Hello world!");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe< sensor_msgs::PointCloud2> ("/head_mount_kinect/depth_registered/points", 1, pclCallback);
  pubCentroid = nh.advertise<geometry_msgs::Point> ("centroid", 1);
  pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  pubCentroid2 = nh.advertise<geometry_msgs::Point> ("centroid2", 1);
  pubCloud2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);

  ros::Subscriber centroid = nh.subscribe<geometry_msgs::Point>("/centroid", 1000, markerCallBack1);
  ros::Subscriber centroid2 = nh.subscribe<geometry_msgs::Point>("/centroid2", 1000, markerCallBack2);
  marker_pub1 = nh.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
  marker_pub2 = nh.advertise<visualization_msgs::Marker>("visualization_marker2", 10);

  ros::spin();
  return 0;
}
