#pragma once
#include "GCS.cpp"

// ������ ���� ��ȣ ����
class CarController
{
	float wheelSpeed;  // ���� �ӵ�
	float wheelAngle;  // ���� ���� (0 ���� / - ���� / + ����) 

	CarController()
	{
		// Initialize
		wheelSpeed = 0.f;
		wheelAngle = 0.f;
	};
	CarController(const CarController& other) {};
	~CarController() {};

public:

	// Singleton
	static CarController* Get()
	{
		static CarController* instance = new CarController();
		return instance;
	}

	float GetWheelSpeed() const
	{
		return wheelSpeed;
	}

	float GetWheelAngle() const
	{
		return wheelAngle;
	}
};
