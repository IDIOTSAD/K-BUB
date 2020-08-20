#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <limits>

//ros �κ�
#include "jsoncpp.cpp"
#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <typeinfo>

#pragma comment(lib, "lib_json.lib")
#pragma warning(disable : 4996)

using namespace std;

typedef numeric_limits<double> dbl;

typedef struct nodeGEO
{
	string ID;
	string Lane_type;
	double x;
	double y;
	double z;
};

typedef struct linkGEO
{
	string ID;
	string Lane_type;
	string length;
	string R_Link; //is Right line exist?
	string L_Link; //is Left line exist?
	string FromND;
	string ToND;
	vector<double> x;
	vector<double> y;
	vector<double> z;
};

//GPS Data!
double CurX;
double CurY;
int Curidx;

//(For Destination) Path list
string pathX[100];
string pathY[100];
int pathQueueSize;

//JSON File
vector<nodeGEO> nodeGeometry;
vector<linkGEO> linkGeometry;

int parse()
{
	ifstream linejson("/home/administator/linkJSON.geojson", ifstream::binary);
	ifstream nodejson("/home/administator/nodeJSON.geojson", ifstream::binary);

	Json::Reader linereader, nodereader;
	Json::Value lineroot, noderoot;
	bool checkparsingRet_line = linereader.parse(linejson, lineroot);
	bool checkparsingRet_node = nodereader.parse(nodejson, noderoot);

	if (!checkparsingRet_line && !checkparsingRet_node)
	{
		std::cout << "Failed to parse Json : " << linereader.getFormattedErrorMessages();
		std::cout << "Failed to parse Json : " << nodereader.getFormattedErrorMessages();
		getchar();
		return 0;
	}
	printf("Parsing\n\n");

	//line vector save
	const Json::Value lineitems = lineroot["features"]; //type, geometry, properties
	int cnt = 0;
	for (auto i = lineitems.begin(); i != lineitems.end(); i++)
	{

		linkGEO g;
		Json::Value ID = (*i)["properties"]["ID"];
		Json::Value LID = (*i)["properties"]["L_LinkID"];
		Json::Value RID = (*i)["properties"]["R_LinkID"];
		Json::Value FND = (*i)["properties"]["FromNodeID"];
		Json::Value TND = (*i)["properties"]["ToNodeID"];
		cnt++;
		cout << cnt << "번째 "
			 << "Start! \n"
			 << endl;

		for (auto j = 0; j != (*i)["geometry"]["coordinates"][0].size(); j++)
		{
			Json::Value arryX = (*i)["geometry"]["coordinates"][0][j][0]; // 0 ~ 2
			Json::Value arryY = (*i)["geometry"]["coordinates"][0][j][1];
			Json::Value arryZ = (*i)["geometry"]["coordinates"][0][j][2];
			g.x.push_back(stod(arryX.asString()));
			g.y.push_back(stod(arryY.asString()));
			g.z.push_back(stod(arryZ.asString()));
		}

		g.ID = ID.asString();
		g.Lane_type = g.ID.substr(0, 2);
		g.FromND = FND.asString();
		g.ToND = TND.asString();
		g.L_Link = LID.asString();
		g.R_Link = RID.asString();

		cout << "type : " << g.Lane_type << ", " << g.ID << " : " << g.FromND << ", " << g.ToND << ", " << g.L_Link << ", " << g.R_Link << endl;

		linkGeometry.push_back(g);
	}
	//--------------end-------------------------

	//node vector save
	const Json::Value nodeitems = noderoot["features"]; //type, geometry, properties
	for (auto i = nodeitems.begin(); i != nodeitems.end(); i++)
	{
		nodeGEO g;
		Json::Value ID = (*i)["properties"]["ID"];
		Json::Value arryX = (*i)["geometry"]["coordinates"][0]; // 0 ~ 2
		Json::Value arryY = (*i)["geometry"]["coordinates"][1];
		Json::Value arryZ = (*i)["geometry"]["coordinates"][2];

		g.ID = ID.asString();
		g.Lane_type = g.ID.substr(0, 2);
		for (auto j = (*i)["geometry"]["coordinates"].begin(); j != (*i)["geometry"]["coordinates"].end(); j++)
		{
		}
		g.x = stod(arryX.asString());
		g.y = stod(arryY.asString());
		g.z = stod(arryZ.asString());

		cout << "type : " << g.Lane_type << ", " << g.ID << " : " << g.x << ", " << g.y << ", " << g.z << endl;

		nodeGeometry.push_back(g);
	}
	//--------------end------------------------

	printf("Parsing End\n");
}

void callback(const sensor_msgs::NavSatFix& fix)
{
	for (auto i = linkGeometry.begin(); i != linkGeometry.end(); i++)
	{
		printf("searching\n");
		for(int j = 0; j < i->x.size(); j++) {
			if(fix.latitude == i->x[j] && fix.longitude == i->y[j]) {
				printf("find!\n");
				getchar();
			}
		}
	}

}

// 자신과 가장 가까운 노드 찾기
// GPS가 고장났다가 복구되었을 때 or 초기 위치 설정 사용 or 목적지 설정하면 에이스타 할까말까?

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jsoncpp");
	ros::NodeHandle nh;
	parse();

	ros::Subscriber odom_sub = nh.subscribe("fix", 10, callback);
	ros::spin();
	
	return 0;
}
