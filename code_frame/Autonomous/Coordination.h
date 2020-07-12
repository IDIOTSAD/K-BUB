#pragma once
#include <iostream>
#include <cmath>
#include <string>

using namespace std;

// ����/�浵 ��ǥ
class Coordination
{
public:

	float Latitude;  // ����
	float Longitude;  // �浵

	Coordination() {
		Latitude = 0.f;
		Longitude = 0.f;
	}
	Coordination(float _latitude, float _longitude)
		: Latitude(_latitude), Longitude(_longitude) { }

	// UNDONE: �Ÿ� ��� ��Ȯ�� ����
	static float GetDistance(Coordination& c1, Coordination& c2) {
		return sqrt(pow(c1.Latitude - c2.Latitude, 2) + pow(c1.Longitude - c2.Longitude, 2));
	}
	float GetDistance(Coordination& other) const {
		return sqrt(pow(Latitude - other.Latitude, 2) + pow(Longitude - other.Longitude, 2));
	}
};
