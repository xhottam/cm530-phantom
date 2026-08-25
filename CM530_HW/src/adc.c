/*
 * adc.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "adc.h"
#include "system_init.h"
#include "mic.h"
#include "system_func.h"




EasyPort_t EasyEPort[12] = {
		{ PORT_SIG_MOT1P, PIN_SIG_MOT1P },
		{ PORT_SIG_MOT1M,PIN_SIG_MOT1M },
		{ PORT_SIG_MOT2P, PIN_SIG_MOT2P },
		{ PORT_SIG_MOT2M, PIN_SIG_MOT2M },
		{ PORT_SIG_MOT3P, PIN_SIG_MOT3P },
		{ PORT_SIG_MOT3M,PIN_SIG_MOT3M },
		{ PORT_SIG_MOT4P, PIN_SIG_MOT4P },
		{ PORT_SIG_MOT4M, PIN_SIG_MOT4M },
		{ PORT_SIG_MOT5P,PIN_SIG_MOT5P },
		{ PORT_SIG_MOT5M, PIN_SIG_MOT5M },
		{ PORT_SIG_MOT6P, PIN_SIG_MOT6P },
		{ PORT_SIG_MOT6M,PIN_SIG_MOT6M }
};



void SetEPort(EPortD_t pin, u8 state) {
	if (state)
		GPIO_SetBits(EasyEPort[pin].port, EasyEPort[pin].pin);
	else
		GPIO_ResetBits(EasyEPort[pin].port, EasyEPort[pin].pin);
}

#define ANALOG_RIGHT_BIT_SHIFT          0
//##############################################################################
u16 ReadAnalog(EPortA_t port) {
	if ((port == EPORT1A) || (port == EPORT4A)) {
		// Select EPORT1A and EPORT4A via multiplexer
		GPIO_ResetBits(PORT_ADC_SELECT0, PIN_ADC_SELECT0);
		GPIO_ResetBits(PORT_ADC_SELECT1, PIN_ADC_SELECT1);

		uDelay(5);

		if (port == EPORT1A) {
			// Start ADC1 Software Conversion
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			uDelay(5);
			return (u16) (ADC_GetConversionValue(ADC1))
					>> ANALOG_RIGHT_BIT_SHIFT;
		} else {
			// Start ADC2 Software Conversion
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);
			uDelay(5);
			return (u16) (ADC_GetConversionValue(ADC2))
					>> ANALOG_RIGHT_BIT_SHIFT;
		}
	} else if ((port == EPORT2A) || (port == EPORT5A)) {
		// Select EPORT2A and EPORT5A via multiplexer
		GPIO_SetBits(PORT_ADC_SELECT0, PIN_ADC_SELECT0);
		GPIO_ResetBits(PORT_ADC_SELECT1, PIN_ADC_SELECT1);

		uDelay(5);

		if (port == EPORT2A) {
			// Start ADC1 Software Conversion
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			uDelay(5);
			return (u16) (ADC_GetConversionValue(ADC1))
					>> ANALOG_RIGHT_BIT_SHIFT;
		} else {
			// Start ADC2 Software Conversion
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);
			uDelay(5);
			return (u16) (ADC_GetConversionValue(ADC2))
					>> ANALOG_RIGHT_BIT_SHIFT;
		}
	} else if ((port == EPORT3A) || (port == EPORT6A)) {
		// Select EPORT3A and EPORT6A via multiplexer
		GPIO_ResetBits(PORT_ADC_SELECT0, PIN_ADC_SELECT0);
		GPIO_SetBits(PORT_ADC_SELECT1, PIN_ADC_SELECT1);

		uDelay(5);

		if (port == EPORT3A) {
			// Start ADC1 Software Conversion
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			uDelay(5);
			return (u16) (ADC_GetConversionValue(ADC1))
					>> ANALOG_RIGHT_BIT_SHIFT;
		} else {
			// Start ADC2 Software Conversion
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);
			uDelay(5);
			return (u16) (ADC_GetConversionValue(ADC2))
					>> ANALOG_RIGHT_BIT_SHIFT;
		}
	} else if (port == VBUS) {
		u16 temp;

		// Set ADC1 to read SIG_VDD/VBUS on Channel 13
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1,
		ADC_SampleTime_239Cycles5);
		uDelay(5);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		uDelay(5);
		temp = (ADC_GetConversionValue(ADC1)) >> ANALOG_RIGHT_BIT_SHIFT;

		// Set ADC1 to read SIG_ADC0 (ADC1 multiplexer output) on Channel 10
		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1,
		ADC_SampleTime_239Cycles5);
		uDelay(5);

		return temp;
	}
	return 0x8000;
}

//##############################################################################
u16 ReadIR(EPortA_t port) {
	u16 temp;

	SetEPort((port * 2), 1);
	SetEPort((port * 2) + 1, 0);

	uDelay(25);
	temp = ReadAnalog(port);

	SetEPort((port * 2), 0);
	SetEPort((port * 2) + 1, 0);

	return temp;
}

//##############################################################################
void Battery_Monitor_Alarm(void) {
	u16 volt = ReadAnalog(VBUS) >> 4;
#ifdef DEBUG_PRINT_VOLTAGE
	//PrintString("\nBattery Voltage: ");
	//Printu32d(volt);
	//PrintString("e-1 [Volts]\n");
#endif

	// ALARM!!!
	if (volt < VBUS_LOW_LIMIT) {
		Buzzed(500, 100);
		Buzzed(500, 5000);
		Buzzed(500, 100);
		Buzzed(500, 5000);
#ifdef DEBUG_ADC
		PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		PrintString("Battery Voltage Critical");
		PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
#endif
	}
	return;
}
