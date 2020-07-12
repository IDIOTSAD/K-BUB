#pragma once
#include "Coordination.h"
#include "StateArbiter.h"

// ���� �Է°� ���� �� ó��
class InputManager
{
	// Camera
	// TODO: ī�޶� �Է°� ����

	// LiDAR
	// TODO: LiDAR �Է°� ����

	// GPS
	Coordination coord;  // GPS�� ���� ��ǥ

	InputManager() {
		// Initialize
	}

	// ���� �Է°� ������Ʈ
	void updateRawInputs() {
		// TODO: updateRawInputs
	}

	// ī�޶� ��ȣ���� �ν��ߴ��� Ȯ��
	bool hasDetectedTrafficLights() {
		// TODO: hasDetectedTrafficLights
		return false;
	}

	// LiDAR�� ��ֹ��� �ν��ߴ��� Ȯ��
	bool hasDetectedObstacle() {
		// TODO: hasDetectedObstacle
		return false;
	}

public:

	// Singleton
	static InputManager* GetInstance() {
		static InputManager* instance = new InputManager();
		return instance;
	}

	// ���������� Ȯ�ε� GPS�� ���� ��ǥ
	Coordination GetCoord() const {
		return coord;
	}
};
