/**
  ******************************************************************************
  * @file    hrtimhandler_dcdc.c
  * @brief   handles HRTIM logic in the context of a DAB converter
  ******************************************************************************
  * @attention
  *	Created by Javier@evert.com 11/2/2025
  *
  ******************************************************************************
  */

#include "hrtim.h"
#include "hrtimhandler_dcdc.h"
//https://imperix.com/doc/implementation/dab-converter-control
//If the phase shift is kept in the range [-0.25, 0.25], the power transfer curve shown above is monotonic.
#define PHASE_DIFF_MAX 0.25
#define PHASE_DIFF_MIN -0.25

#define MAX_DUTY 1.0
#define MIN_DUTY 0.0

#define PWM1 HRTIM_OUTPUT_TA1
#define PWM2 HRTIM_OUTPUT_TA2
#define PWM3 HRTIM_OUTPUT_TB1
#define PWM4 HRTIM_OUTPUT_TB2
#define PWM5 HRTIM_OUTPUT_TD1
#define PWM6 HRTIM_OUTPUT_TD2
#define PWM7 HRTIM_OUTPUT_TC1
#define PWM8 HRTIM_OUTPUT_TC2

/*
 * PINOUT FOR EVERT DAB rev1.0 https://ike.365.altium.com/designs/69461553-9837-4579-B9AB-E5A32C741201?variant=[No+Variations]#design
000000000000000000000 PRIMARY SIDE 000000                                            0000000000000000000000000 SECONDARY SIDE 0000
0                 0                 0                                                0                 0                  0
0              0 PWM1            0 PWM3                                            PWM5 0            PWM7 0               0
0              0     0           0                                                0     0           0                     0
0      HRTIM_OUTPUT_TA1      HRTIM_OUTPUT_TB1                              HRTIM_OUTPUT_TD1      HRTIM_OUTPUT_TC1         0
0           0        00       0  00    00                                      0        00       0  00    00              0
0            000     0          00                                              000     0          00                     0
0              0000000            000000                                          0000000            000000               0
0                 0  0000000000000  0  0000000000000000000000000000  24:15  0000000  0                 0                  0
0                 0                 0                             00       0         0                 0                 00
0                 0                 0                               0 0 0 0          0                 0              0      0
0                 0                 0                               0 0 0 0          0                 0              0      0
  0               0                 0                               0 0 0 0          0                 0              0      0
   0              0                 0                               0 0 0 0          0                 0              0      0
   0              0                 0                               0 0 0 0          0                 0              0      0
  0               0                 0                               0 0 0 0          0                 0              0      0
0                 0                 0                               0 0 0 0          0                 0              0      0
0                 0                 0                               0 0 0 0          0                 0              0      0
0                 0                 0                             00       0         0                 0                 00
0                 0                 0000000000000000000000000000000         0000000000 000000000000000 0                  0
0              0 PWM2              PWM4                                            PWM6               PWM8                0
0             00     0          00     0                                   HRTIM_OUTPUT_TD2      HRTIM_OUTPUT_TC2         0
0      HRTIM_OUTPUT_TA2     HRTIM_OUTPUT_TB2                                    0        0        00  0    0              0
0           0000    000       00000   000                                      0000    000       00000   000              0
0                    0           0                                                0     0           0                     0
0              0000000           0000000                                          0000000           0000000               0
0                 0                 0                                                0                 0                  0
0000000000000000000000000000000000000                                                000000000000000000000000000000000000000000000
*/

//local variables
HRTIM_HandleTypeDef * p_hrtim;


//exposed functions contained in this .c
void HRTIM_dcdc_setup(HRTIM_HandleTypeDef * hrtim);

//local functions contained in this .c
float saturatefloat(float number, float max, float min);
uint32_t saturateuint32_t(uint32_t number, uint32_t max, uint32_t min);

void HRTIM_dcdc_setup(HRTIM_HandleTypeDef * hrtim){
	p_hrtim=hrtim;
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM1);  // Enable the generation of the waveform signal on the designated output
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM2);
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM3);
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM4);
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM5);
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM6);
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM7);
	  HAL_HRTIM_WaveformOutputStart(p_hrtim, PWM8);

	  HAL_HRTIM_WaveformCounterStart(p_hrtim, HRTIM_TIMERID_MASTER);  // Start the counter of the MASTER operating in waveform mode
	  HAL_HRTIM_WaveformCounterStart(p_hrtim, HRTIM_TIMERID_TIMER_A);
	  HAL_HRTIM_WaveformCounterStart(p_hrtim, HRTIM_TIMERID_TIMER_B);
	  HAL_HRTIM_WaveformCounterStart(p_hrtim, HRTIM_TIMERID_TIMER_C);
	  HAL_HRTIM_WaveformCounterStart(p_hrtim, HRTIM_TIMERID_TIMER_D);
}

float saturatefloat(float number, float max, float min){
	if(number>=max){
		return max;
	}else if (number<=min){
		return min;
	}else{
		return number;
	}
}

uint32_t saturateuint32_t(uint32_t number, uint32_t max, uint32_t min){
	if(number>=max){
		return max;
	}else if (number<=min){
		return min;
	}else{
		return number;
	}
}

void HRTIM_set_phase(float phase_diff){
	//full cnt range from master timer p_hrtim->Instance->sMasterRegs.MPER;
	//50% range from master timer p_hrtim->Instance->sMasterRegs.MPER/2;
	uint32_t reg_phaseA=(p_hrtim->Instance->sMasterRegs.MPER/2)*(1+saturatefloat(phase_diff,PHASE_DIFF_MAX,PHASE_DIFF_MIN)/2);
	uint32_t reg_phaseB=(p_hrtim->Instance->sMasterRegs.MPER/2)*(1-saturatefloat(phase_diff,PHASE_DIFF_MAX,PHASE_DIFF_MIN)/2);

	hhrtim1.Instance->sMasterRegs.MCMP1R=saturateuint32_t(reg_phaseA,p_hrtim->Instance->sMasterRegs.MPER-1,1);
	hhrtim1.Instance->sMasterRegs.MCMP2R=saturateuint32_t(reg_phaseB,p_hrtim->Instance->sMasterRegs.MPER-1,1);
}

void HRTIM_set_duty(float dutyPRI, float dutySEC){
	uint32_t reg_dutyPRI=p_hrtim->Instance->sTimerxRegs[0].PERxR * saturatefloat(dutyPRI,MAX_DUTY,MIN_DUTY);
	uint32_t reg_dutySEC=p_hrtim->Instance->sTimerxRegs[2].PERxR * saturatefloat(dutySEC,MAX_DUTY,MIN_DUTY);

	p_hrtim->Instance->sTimerxRegs[0].CMP1xR=saturateuint32_t(reg_dutyPRI,p_hrtim->Instance->sTimerxRegs[0].PERxR-1,1);
	p_hrtim->Instance->sTimerxRegs[1].CMP1xR=saturateuint32_t(reg_dutyPRI,p_hrtim->Instance->sTimerxRegs[1].PERxR-1,1);

	p_hrtim->Instance->sTimerxRegs[2].CMP1xR=saturateuint32_t(reg_dutySEC,p_hrtim->Instance->sTimerxRegs[2].PERxR-1,1);
	p_hrtim->Instance->sTimerxRegs[3].CMP1xR=saturateuint32_t(reg_dutySEC,p_hrtim->Instance->sTimerxRegs[3].PERxR-1,1);
}
