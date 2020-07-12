#pragma once
#include "Coordination.h"

// ���� ���. �켱���� ��������.
enum DRIVE_MODE {
	STOP,  // ����
	DRIVE,  // ������ ���� ���� (ī�޶�)
	SIGNAL,  // ���� ��ȣ ó�� (ī�޶�)
	AVOID,  // ���� ��ֹ� ���� (LiDAR)
};

// ������ ���� ��ȣ ����
class CarController
{
	float wheelSpeed;  // ���� �ӵ�
	float wheelAngle;  // ���� ���� (0 ���� / - ���� / + ����)

	CarController() {
		// Initialize
		wheelSpeed = 0.f;
		wheelAngle = 0.f;
	};
	CarController(const CarController& other) {};
	~CarController() {};

public:

	// Singleton
	static CarController* GetInstance() {
		static CarController* instance = new CarController();
		return instance;
	}

	float GetWheelSpeed() const {
		return wheelSpeed;
	}

	float GetWheelAngle() const {
		return wheelAngle;
	}
};
