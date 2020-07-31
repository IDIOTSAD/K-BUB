#pragma once
#include "GCS.cpp"
#include "CameraInput.cpp"
#include "LidarInput.cpp"
#include "StateArbiter.cpp"

// ����(ī�޶�, LiDAR, GPS) �Է°� ���� �� ó��
class InputManager
{
	// Camera
	CameraInput cameraInput;

	// LiDAR
	LidarInput lidarInput;

	// GPS
	GCS gcs;  // GPS�� ���� ��ǥ
	Position position;  // ���� XY ��ǥ

	InputManager()
	{
		// Initialize
	}

	// ���� �Է°� ������Ʈ
	void updateRawInputs()
	{
		// TODO: updateRawInputs
	}

	// ī�޶� ��ȣ���� �ν��ߴ��� Ȯ��
	bool hasDetectedTrafficLights()
	{
		// TODO: hasDetectedTrafficLights
		return false;
	}

	// LiDAR�� ��ֹ��� �ν��ߴ��� Ȯ��
	bool hasDetectedObstacle()
	{
		// TODO: hasDetectedObstacle
		return false;
	}

public:

	// Singleton
	static InputManager* GetInstance()
	{
		static InputManager* instance = new InputManager();
		return instance;
	}

	// ���������� Ȯ�ε� GPS�� ���� ��ǥ
	GCS GetCoord() const
	{
		return gcs;
	}
};
