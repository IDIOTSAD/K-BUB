#pragma once
#include <queue>
#include "Coordination.h"
#include "InputManager.h"

// �۷ι� ��� ���� �� ��� ����
class GlobalPathManager
{
	// ���� ���� ��� �Ÿ� ����
	const float  ARRIVAL_TOLERANCE = 0.0001f;

	// �۷ι� ��� ��ǥ(��������Ʈ) ť. front�� �������� �̵��� ��ǥ
	queue<Coordination> waypoints;

	GlobalPathManager() {};
	GlobalPathManager(const GlobalPathManager& other) {};
	~GlobalPathManager() {};

public:

	// Singleton
	static GlobalPathManager* GetInstance() {
		static GlobalPathManager* instance = new GlobalPathManager();
		return instance;
	}

	// �������� �̵��� ��������Ʈ
	Coordination GetNextWaypoint() {
		return waypoints.front();
	}

	// ��������Ʈ�� �����ߴ��� Ȯ���ϰ� ó��
	// pos: ���� ��ǥ
	bool CheckArrival(Coordination& pos) {
		if (pos.GetDistance(waypoints.front()) < ARRIVAL_TOLERANCE) {
			ArriveWaypoint();
			return true;
		}
		return false;
	}

	// ��������Ʈ ����
	void ArriveWaypoint() {
		if (waypoints.size() == 0) return;
		waypoints.pop();
	}
};
