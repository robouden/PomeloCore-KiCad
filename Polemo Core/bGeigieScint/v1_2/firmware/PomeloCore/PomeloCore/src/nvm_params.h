#include <asf.h>

// HVDAC = volt * vdac_a + vdac_b

typedef struct pomelo_params
{
	float sipm_vMin;
	float sipm_vMax;
	float sipm_v0deg;
	float sipm_vTempComp;
	float vdac[2];		// HVDAC = vdac[0] + vdac[1] * Vdesired[V]
	float ecal[3];
	uint16_t threshold;
	uint8_t sys_outputs;
	bool sys_power;
	bool sys_coincidence;
	uint8_t sys_pulseChar;
	uint8_t initialized;
};