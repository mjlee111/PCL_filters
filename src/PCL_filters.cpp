#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>

void filter_callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg);
void passthrough_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
void voxel_grid_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg, float x_leaf, float y_leaf, float z_leaf);
void outliner_removal_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg, int meanK, float threshold);

void three_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg);

ros::Subscriber input_cloud_sub;

ros::Publisher passthrough_cloud_pub;
ros::Publisher voxel_cloud_pub;
ros::Publisher outliner_cloud_pub;
ros::Publisher whole_cloud_pub;

int main(int argc, char** argv){
  ros::init(argc, argv, "realsense_filtering_node");
  ros::NodeHandle n;

  input_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, filter_callback);
  passthrough_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("passthrough_cloud", 1);
  voxel_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("voxel_cloud", 1);
  outliner_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("outliner_cloud", 1);
  whole_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  ros::spin();

  return 0;
}


void filter_callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg){
  passthrough_filter(input_cloud_msg, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
  voxel_grid_filter(input_cloud_msg, 0.07, 0.07, 0.07);
  outliner_removal_filter(input_cloud_msg, 50, 1.0);
  three_filter(input_cloud_msg);
}

void passthrough_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud_msg, *input_cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(input_cloud);
  pass_filter.setFilterFieldName("x");
  pass_filter.setFilterLimits(x_min, x_max);
  pass_filter.setFilterFieldName("y");
  pass_filter.setFilterLimits(y_min, y_max);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(z_min, z_max);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter.filter(*filtered_cloud);

  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);

  passthrough_cloud_pub.publish(filtered_cloud_msg);
}

void voxel_grid_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg, float x_leaf, float y_leaf, float z_leaf){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud_msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(x_leaf, y_leaf, z_leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*cloud_out);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_out, cloud_msg);

  voxel_cloud_pub.publish(cloud_msg);
}

void outliner_removal_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg, int meanK, float threshold){
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud_msg, *input_cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliner_filter;
  outliner_filter.setInputCloud(input_cloud);
  outliner_filter.setMeanK(meanK);
  outliner_filter.setStddevMulThresh(threshold);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  outliner_filter.filter(*output_cloud);

  sensor_msgs::PointCloud2 output_cloud_msg;
  pcl::toROSMsg(*output_cloud, output_cloud_msg);
  output_cloud_msg.header = input_cloud_msg->header;

  outliner_cloud_pub.publish(output_cloud_msg);
}

void three_filter(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud_msg, *input_cloud_passthrough);

  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(input_cloud_passthrough);
  pass_filter.setFilterFieldName("x");
  pass_filter.setFilterLimits(0.0, 2.0);
  pass_filter.setFilterFieldName("y");
  pass_filter.setFilterLimits(0.0, 2.0);
  pass_filter.setFilterFieldName("z");
  pass_filter.setFilterLimits(0.0, 2.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter.filter(*filtered_cloud_1);

  //////////////////////////////////////////////////////////////////////////////////////////////////

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(filtered_cloud_1);
  voxel_filter.setLeafSize(0.07f, 0.07f, 0.07f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*filtered_cloud_2);

  //////////////////////////////////////////////////////////////////////////////////////////////////

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliner_filter;
  outliner_filter.setInputCloud(filtered_cloud_2);
  outliner_filter.setMeanK(50);
  outliner_filter.setStddevMulThresh(1.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  outliner_filter.filter(*output_cloud);

  sensor_msgs::PointCloud2 output_cloud_msg;
  pcl::toROSMsg(*output_cloud, output_cloud_msg);
  output_cloud_msg.header = input_cloud_msg->header;

  whole_cloud_pub.publish(output_cloud_msg);
}



