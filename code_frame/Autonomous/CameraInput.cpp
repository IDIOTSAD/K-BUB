#pragma once
#include <string>
#include "Position.cpp"

using namespace std;

struct CameraInput
{
public:

	string className;  // ������ Ŭ������
	float probability;  // Ŭ���� ��Ȯ�� Ȯ��
	Position LeftDown;  // �ٿ�� ���ϴ� ��ǥ
	Position RightUp;  // �ٿ�� ���� ��ǥ
};
