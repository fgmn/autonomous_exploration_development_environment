#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string waypoint_file_dir;
string boundary_file_dir;
double waypointXYRadius = 0.5;  // 在 XY 平面内判断是否到达一个航点的半径
double waypointZBound = 5.0;      // 在 Z 轴上判断是否到达航点的容忍范围
double waitTime = 0;            // 到达航点后等待的时间
double waitTimeStart = 0;       // 记录开始等待的时间
bool isWaiting = false;         // 是否正在等待
double frameRate = 5.0;         // 发布消息的频率
double speed = 1.0;             // 发布给机器人控制的速度
bool sendSpeed = true;          // 是否发送速度消息
bool sendBoundary = true;       // 是否发送边界消息
pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;

// reading waypoints from file function
void readWaypointFile()
{//打开指定的航点文件，解析 PLY 格式文件的 header（直到 "end_header" 为止），
//然后读取顶点（vertex）部分的点数，再依次读取每个点的 x, y, z 坐标，存储到 waypoints 点云中。

  FILE* waypoint_file = fopen(waypoint_file_dir.c_str(), "r");
  if (waypoint_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(waypoint_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(waypoint_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  waypoints->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(waypoint_file, "%f", &point.x);
    val2 = fscanf(waypoint_file, "%f", &point.y);
    val3 = fscanf(waypoint_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    waypoints->push_back(point);
  }

  fclose(waypoint_file);
}

// reading boundary from file function
void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(boundary_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(boundary_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  boundary->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(boundary_file, "%f", &point.x);
    val2 = fscanf(boundary_file, "%f", &point.y);
    val3 = fscanf(boundary_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    boundary->push_back(point);
  }

  fclose(boundary_file);
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();

  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointExample");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);
  nhPrivate.getParam("boundary_file_dir", boundary_file_dir);
  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);
  nhPrivate.getParam("waypointZBound", waypointZBound);
  nhPrivate.getParam("waitTime", waitTime);
  nhPrivate.getParam("frameRate", frameRate);
  nhPrivate.getParam("speed", speed);
  nhPrivate.getParam("sendSpeed", sendSpeed);
  nhPrivate.getParam("sendBoundary", sendBoundary);

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
  geometry_msgs::PointStamped waypointMsgs;
  waypointMsgs.header.frame_id = "map";

  ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32> ("/speed", 5);
  std_msgs::Float32 speedMsgs;

  ros::Publisher pubBoundary = nh.advertise<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5);
  geometry_msgs::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  // read waypoints from file
  readWaypointFile();

  // read boundary from file
  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  int wayPointID = 0;
  int waypointSize = waypoints->points.size();

  if (waypointSize == 0) {
    printf ("\nNo waypoint available, exit.\n\n");
    exit(1);
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    float disX = vehicleX - waypoints->points[wayPointID].x;
    float disY = vehicleY - waypoints->points[wayPointID].y;
    float disZ = vehicleZ - waypoints->points[wayPointID].z;

    // start waiting if the current waypoint is reached
    // 如果车辆接近当前航点（XY 距离小于 waypointXYRadius 且 Z 方向误差小于 waypointZBound）且未处于等待状态，
    // 则记录等待开始时间并将状态置为等待
    if (sqrt(disX * disX + disY * disY) < waypointXYRadius && fabs(disZ) < waypointZBound && !isWaiting) {
      waitTimeStart = curTime;
      isWaiting = true;
    }

    // move to the next waypoint after waiting is over
    if (isWaiting && waitTimeStart + waitTime < curTime && wayPointID < waypointSize - 1) {
      wayPointID++;
      isWaiting = false;
    }

    // publish waypoint, speed, and boundary messages at certain frame rate
    if (curTime - waypointTime > 1.0 / frameRate) {
      if (!isWaiting) {
        waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
        waypointMsgs.point.x = waypoints->points[wayPointID].x;
        waypointMsgs.point.y = waypoints->points[wayPointID].y;
        waypointMsgs.point.z = waypoints->points[wayPointID].z;
        pubWaypoint.publish(waypointMsgs);
      }

      if (sendSpeed) {
        speedMsgs.data = speed;
        pubSpeed.publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = ros::Time().fromSec(curTime);
        pubBoundary.publish(boundaryMsgs);
      }

      waypointTime = curTime;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
