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

void HRTIM_dcdc_setup(HRTIM_HandleTypeDef * hrtim);
void HRTIM_set_phase(float phase_diff);
void HRTIM_set_duty(float dutyPRI, float dutySEC);
#endif /*__HRTIMHANDLER_DCDC_H__ */
