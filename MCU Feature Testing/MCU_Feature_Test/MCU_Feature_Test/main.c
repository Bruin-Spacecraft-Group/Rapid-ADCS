/*
 * MCU Feature Test
 *
 * Created: 5/21/22
 * Author : UCLA RAPID ADCS
 */ 

#include <avr32/io.h>

#include "stdint.h"

#define POINTER(reg) *((uint32_t*) (reg))
#define sbi(port, bit) POINTER((port)) |= (1 << bit) // set bit
#define cbi(port, bit) POINTER((port)) = POINTER((port)) & ~(1 << bit) // clear bit
#define gbi(port, bit) ((POINTER((port)) & (1 << bit)) >> bit) // get bit
#define HIGH 0x1
#define LOW 0x0

//Peripheral Addresses
#define PORTA 0xffff1000
#define PM 0xfff0c00
#define TC 0xffff3800
#define PWM 0xffff3000
#define ADC 0xffff3c00
#define DAC 0xffff4000

//GPIO Offsets (read/write)
#define ENABLE 0x00
#define MUX0 0x10
#define MUX1 0x20
#define OUTPUT_DRIVER 0x40
#define OUTPUT 0x50
#define PIN_VAL 0x60
#define PULL_UP 0x70
//PM offsets
#define CKSEL 0x04
//TC offsets
#define CCR0 0x00
#define CMR0 0x04
#define CV0 0x10
#define SR0 0x20
//PWM offsets
#define ENA 0x04
#define CMR0 0x200
#define CPRD0 0x208
#define CDTY0 0x204
#define CUPD0 0x210
//ADC offsets
#define CR 0x00
#define MR 0x04
#define CHER 0x10
#define LCDR 0x20
#define CDR0 0x30
//DAC offsets
#define SDR 0x00
#define CONTROL 0x08 // should be CR, conflict with ADC offset naming

void setup(){
	//setup PBA clock
	sbi(PM + CKSEL, 23); // enable clock prescaler
	sbi(PM + CKSEL, 24); // PBA division = 4, 2khz clock result
	
	//setup TC
	sbi(TC + CCR0, 0); // enable clock
	sbi(TC + CMR0, 15); // enable waveform
	sbi(TC + CMR0, 0); // set source clock to SBA / 2, 1khz clock result
}

void delay(int ms){
	sbi(TC + CCR0, 2); // software trigger reset
	bool inLoop = true;
	while(inLoop){
		if(POINTER(TC + CV0) >= ms){
			inLoop = false;
		}
	}
}

uint32_t toDutyCycle(double DC){
	int totalBits = 524287;
	return((uint32_t) (DC * totalBits));
}

uint32_t toDAC(double voltage){
	double vRef = 3.3;
	int totalBits = 65535;
	return((uint32_t) ((voltage / vRef) * totalBits));
}

int main(void) {
	setup();
	
	// 0: GPIO output
	// 1: GPIO input
	// 2: PWM output
	// 3: ADC input
	// 4: DAC output
	int testFunction = 0;
	
	int gpioPin = 0;

	if(testFunction == 0){
		// enable and set to output for all gpio pins
		for(int a=0;a<28;a++){
			sbi(PORTA + ENABLE, a);
			sbi(PORTA + OUTPUT_DRIVER, a);
		}

		while(1){
			// GPIO run through
			if(gpioPin >= 28){
				gpioPin = 0;
			}
			sbi(PORTA + OUTPUT, a);
			cbi(PORTA + OUTPUT, (a + 27)%28);
			delay(500);
			gpioPin++;
		}	
	}
	else if(testFunction == 1){
		// enable and set to input for all gpio pins
		for(int a=0;a<28;a++){
			sbi(PORTA + ENABLE, a);
		}

		value = 0;
		while(1){
			// GPIO run through
			if(gpioPin >= 28){
				gpioPin = 0;
			}
			gbi(PORTA + PIN_VAL, gpioPin);
			// [output somewhere]
			delay(500);
			gpioPin++;
		}
	}
	else if(testFunction == 2){
		cbi(PORTA + ENABLE, 7); // disable GPIO
		cbi(PORTA + MUX0, 7);
		cbi(PORTA + MUX0, 7); // setup multiplex selector to PWM
		//[enable PWM clock in PM]
		// optional change channel input clock prescaler (currently 1)
		POINTER(PWM + CPRD0) = 0x7FFFF; // waveform period to max
		uint32_t totalBits = 524287;
		POINTER(PWM + CPRD0) = toDutyCycle(0.5); // set init duty cycle
		sbi(PWM + CMR0, 10); // set to updating duty cycle
		sbi(PWM + ENA, 0); // enable PWM
		while(1){
			sbi(PWM + CUPD0, toDutyCycle(0.75));
			delay(500);
			sbi(PWM + CUPD0, toDutyCycle(0.25));
			delay(500);
		}
	}
	else if(testFunction == 3){
		int output = 0;
		cbi(PORTA + ENABLE, 3); // ensure peripheral function control
		cbi(PORTA + MUX0, 3);
		cbi(PORTA + MUX1, 3); //ensure mux peripheral set to ADC
		sbi(ADC + MR, 26); // set sample hold time
		sbi(ADC + MR, 21); // set startup time
		sbi(ADC + CHER, 0); // enable channel
		while(1){
			delay(1000);
			sbi(ADC + CR, 1); // START trigger
			delay(100);
			output = POINTER(ADC + LCDR);
			// [output somewhere]
		}
	}
	else if(testFunction == 4){
		cbi(PORTA + ENABLE, 3); // ensure peripheral function control
		sbi(PORTA + MUX0, 3);
		sbi(PORTA + MUX1, 3); // set mux peripheral to DAC
		// [setup GCLK-ABDAC clock in generic clock register in PM, follow frequency specs by datasheet]
		sbi(DAC + CONTROL, 31); // enable DAC
		while(1){
			delay(2000);
			POINTER(DAC + SDR) = toDAC(2.5);
			delay(2000);
			POINTER(DAC + SDR) = toDAC(1.5);
		}
	}
}

