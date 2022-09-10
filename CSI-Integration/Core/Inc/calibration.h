/*
 * calibration.h
 *
 *  Created on: Jun 7, 2022
 *      Author: Benson
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "global.h"
#include "CAN_Driver.h"
#include "Sensor.h"

#define MODE_NORMAL     0
#define MODE_GET_CFG    1
#define MODE_SET_CFG    2
#define MODE_REQ_VOLTS  3

//#define TIMEOUT_CALIBRATION (5000 * F_TIMER2 / 1000) //unused?

double ADC_convert(uint16_t ADC_value);

void HostCANmsg(struct_CAN_Message msg);
void CSI_get_Config(void);
//void CSI_set_Config(struct_CAN_Message msg, uint16_t ADC_val,S08 device);
void CSI_set_Config(struct_CAN_Message msg, uint16_t ADC_val);
uint8_t get_CSI_Config_EEPROM(S08 Device, uint16_t dest_address);
void set_CSI_Config_EEPROM(S08 Device, uint8_t val, uint16_t dest_address);

#endif /* INC_CALIBRATION_H_ */
