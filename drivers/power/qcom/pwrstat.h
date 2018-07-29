/*
 * include/linux/pwrstat.h - 
 */
#ifndef _LINUX_PWRSTAT_H_
#define _LINUX_PWRSTAT_H_


#include <linux/node.h>
#include <linux/compiler.h>
#include <linux/cpumask.h>

struct cpu_pstate_pwr {
	unsigned int freq;
	uint32_t power;
};
struct cpu_pwr_stats {
	int cpu;
	long temp;
	struct cpu_pstate_pwr *ptable;
	bool throttling;
	int len;
};



struct cpu_pwr_stats *get_cpu_pwr_stats(void);
void trigger_cpu_pwr_stats_calc(void);

#endif /* _LINUX_CPU_H_ */
