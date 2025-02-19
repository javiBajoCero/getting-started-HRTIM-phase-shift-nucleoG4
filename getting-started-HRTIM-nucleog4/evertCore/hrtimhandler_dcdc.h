/**
  ******************************************************************************
  * @file    hrtimhandler_dcdc.h
  * @brief   This file contains all the function prototypes for
  *          the hrtimhandler_dcdc.c file
  ******************************************************************************
  * @attention
  *	Created by Javier@evert.com 11/2/2025
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HRTIMHANDLER_DCDC_H__
#define __HRTIMHANDLER_DCDC_H__

typedef struct {
	float	main_phase_diff;			//delta
	float	primaryside_phase_diff;		//alpha1
	float	secondaryside_phase_diff;	//alpha2
	float	primaryside_duty;			//tau1
	float	secondaryside_duty;			//tau2
}DABworkSetpoint_struct;

void HRTIM_dcdc_setup(HRTIM_HandleTypeDef * hrtim);

void HRTIM_set_phases					( DABworkSetpoint_struct* setpoint);
void HRTIM_set_primaryandsecondary_duty	( DABworkSetpoint_struct* setpoint);

void HRTIM_setregisters_insideIRQ();

#endif /*__HRTIMHANDLER_DCDC_H__ */
