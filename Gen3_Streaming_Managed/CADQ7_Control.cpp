#include "CADQ7_Control.h"
// error C3699: '*': 'IServiceProvider' ���Ŀ� �� ���� ������ ����� �� �����ϴ�. 
// => #include �� using namespace�� ���� �������� ���� ����
using namespace System;

CADQ7_Control::CADQ7_Control()
{
}

CADQ7_Control::~CADQ7_Control()
{
}

int CADQ7_Control::main_loop(int extclk, int deci_rate)
{
	return ::main_loop(extclk, deci_rate);
}
