#pragma once
#include "CarController.h"

// ���� ��� ����
static class StateArbiter
{
	static DRIVE_MODE driveMode;

public:

	DRIVE_MODE GetDriveMode() const {
		return driveMode;
	}

	// ���� ��� ����
	static DRIVE_MODE DetermineDriveMode(bool bSignal, bool bObstacle) {
		// UNDONE: ��ȣ��� ��ֹ��� ���ÿ� �νĵǴ� ���
		DRIVE_MODE driveMode = DRIVE_MODE::DRIVE;
		if (bObstacle) {  // ��ֹ� �ν�
			driveMode = DRIVE_MODE::AVOID;
		}
		else if (bSignal) {  // ��ȣ�� �ν�
			driveMode = DRIVE_MODE::SIGNAL;
		}
		return driveMode;
	}
};
