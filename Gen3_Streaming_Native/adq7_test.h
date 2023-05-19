#pragma once

#include "settings.h"
#include "ADQAPI.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <signal.h>
#include <inttypes.h>

#ifdef ADQ7DLL_EXPORTS
#define ADQ7DLL_DECLSPEC _declspec(dllexport)
#else
#define ADQ7DLL_DECLSPEC __declspec(dllimport)
#endif

//static int RegisterDump(void* adq_cu, int adq_num, unsigned int ul_target, unsigned int base_addr, unsigned int range);
//
//void sigint_handler(int dummy);
//
//static char label(int channel);
//
//static int write_record_to_file(int channel, int record, void* buf, size_t len);
//
//static void process_data(const struct ADQRecord* const record,
//    const struct ADQDataReadoutStatus* status, int channel,
//    int64_t bytes_received, enum ADQProductID_Enum pid);
//
//static void streaming(void* adq_cu, int adq_num, enum ADQProductID_Enum pid, int extclk, int deci_rate);
//
//static int configure(void* adq_cu, int adq_num, enum ADQProductID_Enum pid);


int main_loop(int extclk, int deci_rate);
