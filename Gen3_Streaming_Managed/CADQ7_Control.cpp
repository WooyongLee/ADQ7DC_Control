#include "CADQ7_Control.h"
// error C3699: '*': 'IServiceProvider' 형식에 이 간접 참조를 사용할 수 없습니다. 
// => #include 와 using namespace의 순서 변경으로 에러 수정
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
