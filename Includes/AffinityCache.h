#pragma once

//#include </usr/include/mach/thread_policy.h>
//#include </System/Library/Frameworks/Kernel.framework/Versions/A/Headers/mach/thread_policy.h>
#include <mach/mach.h>
//#include <omp.h>
#include <assert.h>
#include <stdio.h>

void thread_func(int Newtag);
void AffinityCache(int parallel, int Nprocess, int CoeffAffinity);
