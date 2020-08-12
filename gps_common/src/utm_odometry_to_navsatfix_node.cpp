/*
 * Translates nav_msgs/Odometry in UTM coordinates back into sensor_msgs/NavSat{Fix,Status}
 * Useful for visualizing UTM data on a map or comparing with raw GPS data
 * Added by Dheera Venkatraman (dheera@dheera.net)
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include "gps_common/GPSFix.h"
#include "gps_common/GPSStatus.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <stdio.h>

 //우리 UTM 좌표 52N임.
 //UTM은 미터단위라는 점..

using namespace gps_common;
using namespace std;

struct List {
	double x;
	double y;
	bool isArrive;
};

double testX = 346186.173608;
double testY = 4069617.6487;
double errorRange = 0.05;

static ros::Publisher fix_pub;
static ros::Publisher next_pub;
string frame_id, child_frame_id;
string zone_param;
double rot_cov;
ifstream readFile;
vector<List> GlobalUTMList;

void fileopen() {
	readFile.open("x_y_path2.txt");
	string line;
	getline(readFile, line);
	bool isX = true;
	while (getline(readFile, line)) {
		List list;
		size_t pre = 0, current;
		current = line.find(',');
		while (current != string::npos) {
			string substring = line.substr(pre, current - pre);
			list.x = stod(substring);
			pre = current + 1;
			current = line.find(',', pre);
			string substring2 = line.substr(pre, current - pre);
			cout.precision(13);
			list.y = stod(substring2);
		}
		list.isArrive = false;
		GlobalUTMList.push_back(list);
	}
	readFile.close();
}

List search(const nav_msgs::OdometryConstPtr& odom) {
	List nextList;
	int next;
	double minDistance = 999999999;
	for (auto i = GlobalUTMList.begin(); i != GlobalUTMList.end(); i++) {
		double manhatan = sqrt(pow((odom->pose.pose.position.x - i->x), 2) + pow((odom->pose.pose.position.y - i->y), 2));
		if (manhatan < minDistance && i->isArrive == false) {
			minDistance = manhatan;
			nextList.x = i->x;
			nextList.y = i->y;
		}
	}
	cout << nextList.x << endl;
	cout << nextList.y << endl;

	for (auto i = GlobalUTMList.begin(); i != GlobalUTMList.end(); i++) {
		if (fabs(i->x - nextList.x) < errorRange && fabs(i->y - nextList.y) < errorRange) {
			if (testX == i->x && testY == i->y) {	//isArrive?
				i->isArrive = true;
			}
		}
	}
	return nextList;
}

void callback(const nav_msgs::OdometryConstPtr& odom) {

	if (odom->header.stamp == ros::Time(0)) {
		return;
	}

	if (!fix_pub || !next_pub) {
		return;
	}

	double northing, easting, latitude, longitude;
	std::string zone;
	sensor_msgs::NavSatFix fix;

	northing = odom->pose.pose.position.y;
	easting = odom->pose.pose.position.x;

	if (zone_param.length() > 0) {
		// utm zone was supplied as a ROS parameter
		zone = zone_param;
		fix.header.frame_id = odom->header.frame_id;
	}
	else {
		// look for the utm zone in the frame_id
		std::size_t pos = odom->header.frame_id.find("/utm_");
		if (pos == std::string::npos) {
			ROS_WARN("UTM zone not found in frame_id");
			return;
		}
		zone = odom->header.frame_id.substr(pos + 5, 3);
		fix.header.frame_id = odom->header.frame_id.substr(0, pos);
	}

	ROS_INFO("zone: %s", zone.c_str());

	fix.header.stamp = odom->header.stamp;

	UTMtoLL(northing, easting, zone, latitude, longitude);

	fix.latitude = latitude;
	fix.longitude = longitude;
	fix.altitude = odom->pose.pose.position.z;

	fix.position_covariance[0] = odom->pose.covariance[0];
	fix.position_covariance[1] = odom->pose.covariance[1];
	fix.position_covariance[2] = odom->pose.covariance[2];
	fix.position_covariance[3] = odom->pose.covariance[6];
	fix.position_covariance[4] = odom->pose.covariance[7];
	fix.position_covariance[5] = odom->pose.covariance[8];
	fix.position_covariance[6] = odom->pose.covariance[12];
	fix.position_covariance[7] = odom->pose.covariance[13];
	fix.position_covariance[8] = odom->pose.covariance[14];

	fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

	fix_pub.publish(fix);

	List nextL;

	if (GlobalUTMList.back().isArrive != true) {
		nextL = search(odom);
	}
	sensor_msgs::NavSatFix next;

	double nextnorthing, nexteasting, nextlatitude, nextlongitude;

	nextnorthing = nextL.x;
	nexteasting = nextL.y;

	UTMtoLL(nextnorthing, nexteasting, zone, nextlatitude, nextlongitude);

	next.latitude = nextlatitude;
	next.longitude = nextlongitude;
	next.altitude = odom->pose.pose.position.z;

	next.position_covariance[0] = odom->pose.covariance[0];
	next.position_covariance[1] = odom->pose.covariance[1];
	next.position_covariance[2] = odom->pose.covariance[2];
	next.position_covariance[3] = odom->pose.covariance[6];
	next.position_covariance[4] = odom->pose.covariance[7];
	next.position_covariance[5] = odom->pose.covariance[8];
	next.position_covariance[6] = odom->pose.covariance[12];
	next.position_covariance[7] = odom->pose.covariance[13];
	next.position_covariance[8] = odom->pose.covariance[14];

	next.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

	next_pub.publish(next);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "utm_odometry_to_navsatfix_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_node("~");

	priv_node.param<std::string>("frame_id", frame_id, "");
	priv_node.param<std::string>("zone", zone_param, "");

	fix_pub = node.advertise<sensor_msgs::NavSatFix>("odom", 10);
	next_pub = node.advertise<sensor_msgs::NavSatFix>("nextodom", 10);

	ros::Subscriber odom_sub = node.subscribe("odom", 10, callback);

	ros::spin();
}