#include <asf.h>

typedef struct core_params
{
	uint8_t version;
	float vDac[2];		// HVDAC = vdac[0] + vdac[1] * Vdesired[V]
	float iMeas[3];
	uint16_t threshold;
	uint8_t sys_outputs;
	uint8_t sys_pulseChar;
	bool sys_power;
	bool sys_coincidence;
};

typedef struct core_params_old
{
	float vdac[2];		// HVDAC = vdac[0] + vdac[1] * Vdesired[V]
	uint16_t threshold;
	uint8_t sys_outputs;
	bool sys_power;
	bool sys_coincidence;
	uint8_t sys_pulseChar;
	uint8_t initialized;
};


typedef struct physics_params
{
	uint8_t version;
	float sipm_vMin;
	float sipm_vMax;
	float sipm_v0deg;
	float sipm_vTempComp;
	float ecal[3];
	float uSvph_constant;
	char detString[64];
	uint8_t tempType;		// 1: TMP116, 2: TMP451
};


typedef struct physics_params_old
{
	float sipm_vMin;
	float sipm_vMax;
	float sipm_v0deg;
	float sipm_vTempComp;
	float ecal[3];
	float uSvph_constant;
	char detString[64];
	uint8_t tempType;		// 1: TMP116, 2: TMP451
	uint8_t initialized;
};


