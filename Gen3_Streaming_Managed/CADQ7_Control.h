#pragma once
#pragma comment(lib, "Gen3_Streaming_Native.lib") 
#define _CRT_SECURE_NO_WARNINGS

#include "..\Gen3_Streaming_Native\adq7_test.h"
using namespace System;

public ref class CADQ7_Control
{
public:
    CADQ7_Control();
    ~CADQ7_Control();

    int main_loop(int extclk, int deci_rate);
};