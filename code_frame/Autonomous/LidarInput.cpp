#pragma once
#include <vector>
#include "Position.cpp"

using namespace std;

struct LidarInput
{
public:

	vector<pair<Position, Position>> obstacles;  // ��ֹ����� �ּ�/�ִ� XY ��ǥ �迭
};
