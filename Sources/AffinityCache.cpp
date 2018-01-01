#include "../Includes/AffinityCache.h"


void thread_func(int Newtag){
	thread_affinity_policy ap;
	ap.affinity_tag = Newtag; // non-null affinity tag
	
	int ret = ::thread_policy_set(
								  mach_thread_self(),
								  THREAD_AFFINITY_POLICY,
								  (integer_t*) &ap,
								  THREAD_AFFINITY_POLICY_COUNT
								  );
	assert(ret == 0);
}

void AffinityCache(int parallel, int Nprocess, int CoeffAffinity){
	if(parallel == 1){
		//Openmp
		//omp_set_num_threads(Nprocess);
		if(Nprocess == 2){
#pragma omp parallel
			{ 
#pragma omp sections 
				{
#pragma omp section	
					thread_func(CoeffAffinity);
#pragma omp section	
					thread_func(CoeffAffinity);
				}				
			}
		}
		else{
#pragma omp parallel
			{ 
#pragma omp sections 
				{
#pragma omp section	
					thread_func(CoeffAffinity);
#pragma omp section	
					thread_func(CoeffAffinity);
#pragma omp section	
					thread_func(CoeffAffinity);
#pragma omp section	
					thread_func(CoeffAffinity);
				}				
			}
		}
	}
	else{
		thread_func(CoeffAffinity);
	}		
}

