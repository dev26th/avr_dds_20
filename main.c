//*****************************************************************************
//
// File Name	: 'main.c'
// Title		: AVR DDS2 signal generator
// Author		: Scienceprog.com - Copyright (C) 2008
// Created		: 2008-03-09
// Revised		: 2008-03-09
// Version		: 2.0
// Target MCU	: Atmel AVR series ATmega16
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include "lcd_lib.h"

// define R2R port
#define R2RPORT PORTA
#define R2RDDR  DDRA

// define button port and dedicated pins (PD0-4)
#define BPORT   PORTD
#define BPIN    PIND
#define BDDR    DDRD
#define DOWN    0
#define LEFT    1
#define START   2
#define RIGHT   3
#define UP      4
#define BPORT2  PORTB
#define BPIN2   PINB
#define BDDR2   DDRB
#define OPT     2

// define Highs Speed (HS) signal output (PD5)
#define HSDDR   DDRD
#define HSPORT  PORTD
#define HS      5

// define eeprom addresses
#define EE_MENU_ENTRY  0
#define EE_FREQ_1      1
#define EE_FREQ_2      2
#define EE_FREQ_3      3
#define EE_FREQ_STEP_1 4
#define EE_FREQ_STEP_2 5
#define EE_HS_FREQ     6
#define EE_PWM_FREQ_1  7
#define EE_PWM_FREQ_2  8
#define EE_PWM_DUTY    9
#define EE_OFF_LEVEL   10
#define EE_INIT        E2END

#define RESOLUTION (16000000.0/9/65536/256)  // (16000000 Hz system clock)/(9 clocks per iteration)
                                             // /(2 bytes in fraction part => 65536 units)/(256 points per period)
#define FREQ_CAL      0.9999     // some calibration coeficient FIXME should be run-time configurable
#define MIN_FREQ      0.0        // minimum DDS frequency
#define MAX_FREQ      999999.999 // maximum DDS frequency
#define MIN_FREQ_STEP 0.001      // minimum DDS frequency step
#define MAX_FREQ_STEP 10000.0    // maximum DDS frequency step

void timer2Init(void);
void timer2Start(void);
void timer2Stop(void);
void timer1Start(uint8_t);
void timer1StartPwm(uint16_t);
void timer1Stop(void);
void static inline signalOut(const uint8_t *, uint8_t, uint8_t, uint8_t);
void static inline randomSignalOut(const uint8_t *);
void static inline sweepOut(const uint8_t *, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

// button processing
typedef void (ButtonHandlerFn_t)(void);
void processButton(void);
void buttonNop(void);
void menu_onUp(void);
void menu_onDown(void);
void menu_onOpt(void);
void optMenu_onUp(void);
void optMenu_onDown(void);
void optMenu_onOpt(void);
void signal_onLeft(void);
void signal_onRight(void);
void signal_onStart(void);
void freqStep_onLeft(void);
void freqStep_onRight(void);
void noise_onStart(void);
void hs_onLeft(void);
void hs_onRight(void);
void hs_onStart(void);
void pwm_onLeft(void);
void pwm_onRight(void);
void pwm_onStart(void);
void sweep_onStart(void);
void offLevel_onLeft(void);
void offLevel_onRight(void);

// menu processing
typedef void (MenuItemEnterHandlerFn_t)(void);
void signal_updateDisplay(void);
void noise_updateDisplay(void);
void freqStep_updateDisplay(void);
void hs_updateDisplay(void);
void pwm_updateDisplay(void);
void offLevel_updateDisplay(void);

// adjust LCDsendChar() function for strema
static int LCDsendstream(char c, FILE *stream);
// set output stream to LCD
static FILE lcd_str = FDEV_SETUP_STREAM(LCDsendstream, NULL, _FDEV_SETUP_WRITE);

struct ButtonHandlers {
	ButtonHandlerFn_t * onUp;
	ButtonHandlerFn_t * onDown;
	ButtonHandlerFn_t * onLeft;
	ButtonHandlerFn_t * onRight;
	ButtonHandlerFn_t * onStart;
	ButtonHandlerFn_t * onOpt;
};

struct MenuEntry {
	const char * title;
	uint8_t tag;
	MenuItemEnterHandlerFn_t * updateDisplay;
	struct ButtonHandlers buttonHandlers;
};

const char SINE_TITLE[]      PROGMEM = "      Sine      ";
const char SQUARE_TITLE[]    PROGMEM = "     Square     ";
const char TRIANGLE_TITLE[]  PROGMEM = "    Triangle    ";
const char SAW_TITLE[]       PROGMEM = "    SawTooth    ";
const char REV_SAW_TITLE[]   PROGMEM = "  Rev SawTooth  ";
const char ECG_TITLE[]       PROGMEM = "      ECG       ";
const char FREQ_STEP_TITLE[] PROGMEM = "    Freq Step   ";
const char NOISE_TITLE[]     PROGMEM = "     Noise      ";
const char HS_TITLE[]        PROGMEM = "   High Speed   ";
const char PWM_TITLE[]       PROGMEM = "    PWM (HS)    ";
const char SWEEP_TITLE[]     PROGMEM = "     Sweep      ";
const char OFF_LEVEL_TITLE[] PROGMEM = "   Off Level    ";

const struct MenuEntry MENU[] PROGMEM = {
	{
		SINE_TITLE,
		0,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			signal_onStart,
			menu_onOpt,
		}
	},
	{
		SQUARE_TITLE,
		1,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			signal_onStart,
			menu_onOpt,
		}
	},
	{
		TRIANGLE_TITLE,
		2,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			signal_onStart,
			menu_onOpt,
		}
	},
	{
		SAW_TITLE,
		3,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			signal_onStart,
			menu_onOpt,
		}
	},
	{
		REV_SAW_TITLE,
		4,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			signal_onStart,
			menu_onOpt,
		}
	},
	{
		ECG_TITLE,
		5,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			signal_onStart,
			menu_onOpt,
		}
	},
	{
		NOISE_TITLE,
		0,
		noise_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			buttonNop,
			buttonNop,
			noise_onStart,
			menu_onOpt,
		}
	},
	{
		HS_TITLE,
		0,
		hs_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			hs_onLeft,
			hs_onRight,
			hs_onStart,
			menu_onOpt,
		}
	},
	{
		PWM_TITLE,
		0,
		pwm_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			pwm_onLeft,
			pwm_onRight,
			pwm_onStart,
			menu_onOpt,
		}
	},
	{
		SWEEP_TITLE,
		0,
		signal_updateDisplay,
		{
			menu_onUp,
			menu_onDown,
			signal_onLeft,
			signal_onRight,
			sweep_onStart,
			menu_onOpt,
		}
	},
};
static const uint8_t MENU_SIZE = (sizeof(MENU)/sizeof(MENU[0]));

const struct MenuEntry OPT_MENU[] PROGMEM = {
	{
		FREQ_STEP_TITLE,
		0,
		freqStep_updateDisplay,
		{
			optMenu_onUp,
			optMenu_onDown,
			freqStep_onLeft,
			freqStep_onRight,
			optMenu_onOpt,
			optMenu_onOpt,
		}
	},
	{
		OFF_LEVEL_TITLE,
		0,
		offLevel_updateDisplay,
		{
			optMenu_onUp,
			optMenu_onDown,
			offLevel_onLeft,
			offLevel_onRight,
			optMenu_onOpt,
			optMenu_onOpt,
		}
	},
};
static const uint8_t OPT_MENU_SIZE = (sizeof(OPT_MENU)/sizeof(OPT_MENU[0]));

//various LCD strings
const char MNON[]  PROGMEM = "ON ";
const char MNOFF[] PROGMEM = "OFF";
const char RND[]   PROGMEM = "    Random";

struct signal {
	volatile double   freq;		// frequency value
	volatile bool     running;	// generator on/off
	volatile uint8_t  hsFreq;	// high speed frequency [1..8Mhz]
	volatile double   freqStep;	// frequency step value
	volatile uint16_t pwmFreq;	// PWM freq [61..62500Hz]
	volatile uint8_t  pwmDuty;
	volatile uint8_t  offLevel;     // output value when then generator if off
} SG;

//define signals
const uint8_t SINE_WAVE[] PROGMEM = {
	0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
	0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
	0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5,
	0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,
	0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc,
	0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3,
	0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83,
	0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
	0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
	0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a,
	0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08,
	0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
	0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
	0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c
};

const uint8_t SQUARE_WAVE[] PROGMEM = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
};

const uint8_t SAWTOOTH_WAVE[] PROGMEM = {
	0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,
	0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,
	0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,
	0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,
	0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d,0x5e,0x5f,
	0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,
	0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,
	0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,
	0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,
	0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
	0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,
	0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,
	0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,
	0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,
	0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
};

const uint8_t REV_SAWTOOTH_WAVE[] PROGMEM = {
	0xff,0xfe,0xfd,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,0xf6,0xf5,0xf4,0xf3,0xf2,0xf1,0xf0,
	0xef,0xee,0xed,0xec,0xeb,0xea,0xe9,0xe8,0xe7,0xe6,0xe5,0xe4,0xe3,0xe2,0xe1,0xe0,
	0xdf,0xde,0xdd,0xdc,0xdb,0xda,0xd9,0xd8,0xd7,0xd6,0xd5,0xd4,0xd3,0xd2,0xd1,0xd0,
	0xcf,0xce,0xcd,0xcc,0xcb,0xca,0xc9,0xc8,0xc7,0xc6,0xc5,0xc4,0xc3,0xc2,0xc1,0xc0,
	0xbf,0xbe,0xbd,0xbc,0xbb,0xba,0xb9,0xb8,0xb7,0xb6,0xb5,0xb4,0xb3,0xb2,0xb1,0xb0,
	0xaf,0xae,0xad,0xac,0xab,0xaa,0xa9,0xa8,0xa7,0xa6,0xa5,0xa4,0xa3,0xa2,0xa1,0xa0,
	0x9f,0x9e,0x9d,0x9c,0x9b,0x9a,0x99,0x98,0x97,0x96,0x95,0x94,0x93,0x92,0x91,0x90,
	0x8f,0x8e,0x8d,0x8c,0x8b,0x8a,0x89,0x88,0x87,0x86,0x85,0x84,0x83,0x82,0x81,0x80,
	0x7f,0x7e,0x7d,0x7c,0x7b,0x7a,0x79,0x78,0x77,0x76,0x75,0x74,0x73,0x72,0x71,0x70,
	0x6f,0x6e,0x6d,0x6c,0x6b,0x6a,0x69,0x68,0x67,0x66,0x65,0x64,0x63,0x62,0x61,0x60,
	0x5f,0x5e,0x5d,0x5c,0x5b,0x5a,0x59,0x58,0x57,0x56,0x55,0x54,0x53,0x52,0x51,0x50,
	0x4f,0x4e,0x4d,0x4c,0x4b,0x4a,0x49,0x48,0x47,0x46,0x45,0x44,0x43,0x42,0x41,0x40,
	0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,0x37,0x36,0x35,0x34,0x33,0x32,0x31,0x30,
	0x2f,0x2e,0x2d,0x2c,0x2b,0x2a,0x29,0x28,0x27,0x26,0x25,0x24,0x23,0x22,0x21,0x20,
	0x1f,0x1e,0x1d,0x1c,0x1b,0x1a,0x19,0x18,0x17,0x16,0x15,0x14,0x13,0x12,0x11,0x10,
	0x0f,0x0e,0x0d,0x0c,0x0b,0x0a,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,
};

const uint8_t TRIANGLE_WAVE[] PROGMEM = {
	0x00,0x02,0x04,0x06,0x08,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x16,0x18,0x1a,0x1c,0x1e,
	0x20,0x22,0x24,0x26,0x28,0x2a,0x2c,0x2e,0x30,0x32,0x34,0x36,0x38,0x3a,0x3c,0x3e,
	0x40,0x42,0x44,0x46,0x48,0x4a,0x4c,0x4e,0x50,0x52,0x54,0x56,0x58,0x5a,0x5c,0x5e,
	0x60,0x62,0x64,0x66,0x68,0x6a,0x6c,0x6e,0x70,0x72,0x74,0x76,0x78,0x7a,0x7c,0x7e,
	0x80,0x82,0x84,0x86,0x88,0x8a,0x8c,0x8e,0x90,0x92,0x94,0x96,0x98,0x9a,0x9c,0x9e,
	0xa0,0xa2,0xa4,0xa6,0xa8,0xaa,0xac,0xae,0xb0,0xb2,0xb4,0xb6,0xb8,0xba,0xbc,0xbe,
	0xc0,0xc2,0xc4,0xc6,0xc8,0xca,0xcc,0xce,0xd0,0xd2,0xd4,0xd6,0xd8,0xda,0xdc,0xde,
	0xe0,0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xee,0xf0,0xf2,0xf4,0xf6,0xf8,0xfa,0xfc,0xfe,
	0xff,0xfd,0xfb,0xf9,0xf7,0xf5,0xf3,0xf1,0xef,0xef,0xeb,0xe9,0xe7,0xe5,0xe3,0xe1,
	0xdf,0xdd,0xdb,0xd9,0xd7,0xd5,0xd3,0xd1,0xcf,0xcf,0xcb,0xc9,0xc7,0xc5,0xc3,0xc1,
	0xbf,0xbd,0xbb,0xb9,0xb7,0xb5,0xb3,0xb1,0xaf,0xaf,0xab,0xa9,0xa7,0xa5,0xa3,0xa1,
	0x9f,0x9d,0x9b,0x99,0x97,0x95,0x93,0x91,0x8f,0x8f,0x8b,0x89,0x87,0x85,0x83,0x81,
	0x7f,0x7d,0x7b,0x79,0x77,0x75,0x73,0x71,0x6f,0x6f,0x6b,0x69,0x67,0x65,0x63,0x61,
	0x5f,0x5d,0x5b,0x59,0x57,0x55,0x53,0x51,0x4f,0x4f,0x4b,0x49,0x47,0x45,0x43,0x41,
	0x3f,0x3d,0x3b,0x39,0x37,0x35,0x33,0x31,0x2f,0x2f,0x2b,0x29,0x27,0x25,0x23,0x21,
	0x1f,0x1d,0x1b,0x19,0x17,0x15,0x13,0x11,0x0f,0x0f,0x0b,0x09,0x07,0x05,0x03,0x01
};

const uint8_t ECG_WAVE[] PROGMEM = {
	73,74,75,75,74,73,73,73,73,72,71,69,68,67,67,67,
	68,68,67,65,62,61,59,57,56,55,55,54,54,54,55,55,
	55,55,55,55,54,53,51,50,49,49,52,61,77,101,132,
	169,207,238,255,254,234,198,154,109,68,37,17,5,
	0,1,6,13,20,28,36,45,52,57,61,64,65,66,67,68,68,
	69,70,71,71,71,71,71,71,71,71,72,72,72,73,73,74,
	75,75,76,77,78,79,80,81,82,83,84,86,88,91,93,96,
	98,100,102,104,107,109,112,115,118,121,123,125,
	126,127,127,127,127,127,126,125,124,121,119,116,
	113,109,105,102,98,95,92,89,87,84,81,79,77,76,75,
	74,73,72,70,69,68,67,67,67,68,68,68,69,69,69,69,
	69,69,69,70,71,72,73,73,74,74,75,75,75,75,75,75,
	74,74,73,73,73,73,72,72,72,71,71,71,71,71,71,71,
	70,70,70,69,69,69,69,69,70,70,70,69,68,68,67,67,
	67,67,66,66,66,65,65,65,65,65,65,65,65,64,64,63,
	63,64,64,65,65,65,65,65,65,65,64,64,64,64,64,64,
	64,64,65,65,65,66,67,68,69,71,72,73
};

const uint8_t SINE_WAVE_FROM_ZERO[] PROGMEM = { //sine 256 values, start from 0
	0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08,
	0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
	0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
	0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c,
	0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
	0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
	0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5,
	0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,
	0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc,
	0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3,
	0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83,
	0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
	0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
	0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a,
	0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,0x02,0x01,0x01
};

const uint8_t NOISE_SIGNAL[] PROGMEM = {
	0x0a,0x0e,0x2d,0x73,0xc4,0x40,0xaa,0x8f,0xdd,0xf3,0x6b,0x97,0xb9,0x8d,0x77,0x57,
	0xe3,0x52,0x93,0x3f,0x25,0x07,0x99,0x5f,0x8b,0x37,0x30,0x7b,0x3a,0x89,0xc6,0xae,
	0x4e,0x58,0xe4,0x4b,0x48,0x05,0xd6,0xf2,0x5c,0x44,0xef,0xf8,0x69,0xf6,0x92,0x56,
	0x1d,0x96,0xab,0x2f,0x88,0x35,0xf5,0x36,0x83,0xfc,0x8e,0x60,0xe0,0xda,0xa8,0x5b,
	0xdf,0x7e,0x4d,0x3b,0x38,0x91,0x2b,0xfa,0x21,0xc2,0x23,0x0d,0x2e,0xce,0x3c,0xb6,
	0x03,0x32,0xed,0x86,0xe6,0x0f,0x55,0x6a,0x34,0xb8,0x70,0x45,0x49,0x9b,0x76,0xbc,
	0x18,0x5a,0x41,0x46,0x28,0xfd,0x2c,0xb0,0xea,0xb2,0xde,0x65,0xbb,0x10,0x59,0xf1,
	0x9d,0xb7,0x29,0xd4,0xeb,0x42,0x85,0x6f,0x39,0xd5,0x26,0x90,0x7f,0xa7,0xe8,0xd9,
	0x98,0xc1,0x8c,0x11,0x62,0xad,0x81,0x66,0x0c,0x5d,0x19,0x01,0x1e,0xc8,0x87,0xe1,
	0x2a,0xd2,0x24,0xd1,0x43,0xe7,0x4f,0x68,0xc0,0xaf,0x5e,0x9e,0x84,0xe2,0x50,0xcb,
	0x1a,0xc3,0xb4,0x74,0x04,0xac,0x64,0xa0,0x13,0xd3,0x31,0x00,0x9c,0xfe,0x4a,0xb3,
	0x78,0x15,0x3e,0xee,0x94,0x7c,0x1c,0x72,0xa1,0x20,0x9f,0x95,0xcf,0x3d,0x82,0xb5,
	0xbd,0x54,0xa6,0x47,0x6e,0x75,0xc7,0x1b,0xd7,0x09,0x16,0xf0,0x12,0x02,0xb1,0x06,
	0x4c,0xcd,0xa9,0xa2,0x6c,0xa5,0x61,0xca,0x7d,0x1f,0x22,0x17,0x14,0xc5,0xd8,0x6d,
	0x8a,0xf7,0x51,0xa3,0xfb,0xf4,0x63,0xbf,0x79,0xc9,0x27,0xec,0x7a,0x9a,0xbe,0x80,
	0xff,0xe5,0xba,0xcc,0x0b,0xdb,0xdc,0xf9,0x67,0xe9,0xa4,0x08,0xd0,0x71,0x33,0x53
};

const uint8_t * const SIGNALS[] PROGMEM = {
	SINE_WAVE,
	SQUARE_WAVE,
	TRIANGLE_WAVE,
	SAWTOOTH_WAVE,
	REV_SAWTOOTH_WAVE,
	ECG_WAVE
};

const uint8_t * const SWEEP_SIGNALS[] PROGMEM = {
	SINE_WAVE_FROM_ZERO
};

enum Button {
	Button_None,
	Button_Up,
	Button_Right,
	Button_Down,
	Button_Left,
	Button_Start,
	Button_Opt,
};

struct ButtonState {
	uint16_t now;
	uint16_t pressedUntil;
	uint16_t nextAuto;
	volatile enum Button pressed;
	volatile bool processed;
};

struct ButtonState buttonState;
static const uint16_t BUTTON_UNBOUNCE    = 20;
static const uint16_t BUTTON_AUTO_START  = 100;
static const uint16_t BUTTON_AUTO_REPEAT = 8;
static const uint16_t BUTTON_TIME_WRAP   = 32768;

uint8_t menuEntryNum = 0;                // active or last active main menu entry
uint8_t optMenuEntryNum = (uint8_t)-1;   // active opt-menu entry or -1 if not in the opt-menu
struct MenuEntry menuEntry;              // copy of active menu entry
struct ButtonHandlers * buttonHandlers;

uint8_t signalBuffer[256]
	__attribute__ ((aligned(256)))
	__attribute__ ((section (".noinit")));

// adjust LCD stream fuinction to use with printf()
static int LCDsendstream(char c , FILE *stream) {
	LCDsendChar(c);
	return 0;
}

inline const void * pgm_read_ptr(const void * addr)
{
	return (const void *)pgm_read_word(addr);
}

// initialize Timer2 (used for button reading)
void timer2Init(void) {
	TCNT2 = 0x00;
} 

void timer2Start(void) {
	TCCR2 |= (1 << CS22) | (1 << CS21); // prescaller 256 => ~244 interrupts/s
	TIMSK |= (1 << TOV2);               // enable overflow interrupts
}

void timer2Stop(void) {
	TCCR0 &= ~((1 << CS22) | (1 << CS21)); // stop
	TIMSK &= ~(1 << TOV2);                 // disable overflow interrupts
}

// External interrupts service routines
// used to stop DDS in the inline ASM by setting
// CPHA bit in SPCR register
ISR(INT0_vect) {
	SPCR |= (1 << CPHA);
}
ISR(INT1_vect) {
	SPCR |= (1 << CPHA);
}
ISR(INT2_vect) {
	SPCR |= (1 << CPHA);
}

// called every 4.1 ms, takes ~4 us
void checkButtons(void) {
	++buttonState.now;
	uint16_t now = buttonState.now;

	enum Button newButton;
	if(bit_is_clear(BPIN, UP))
		newButton = Button_Up;
	else if(bit_is_clear(BPIN, RIGHT))
		newButton = Button_Right;
	else if(bit_is_clear(BPIN, DOWN))
		newButton = Button_Down;
	else if(bit_is_clear(BPIN, LEFT))
		newButton = Button_Left;
	else if(bit_is_clear(BPIN, START))
		newButton = Button_Start;
	else if(bit_is_clear(BPIN2, OPT)) // must be checked as last one
		newButton = Button_Opt;
	else
		newButton = Button_None;

	if(buttonState.pressed != newButton) {
		bool ignore = false;
		if(newButton == Button_None) {
			if((buttonState.pressedUntil - now) < BUTTON_TIME_WRAP)
				ignore = true;
		}
		
		if(!ignore) {
			buttonState.pressedUntil = buttonState.now + BUTTON_UNBOUNCE;
			buttonState.pressed = newButton;
			buttonState.processed = false;
			buttonState.nextAuto = buttonState.now + BUTTON_AUTO_START;
		}
	}
	else if(buttonState.pressed != Button_None) {
		if((buttonState.nextAuto - now) >= BUTTON_TIME_WRAP) {
			buttonState.processed = false;
			buttonState.nextAuto = buttonState.now + BUTTON_AUTO_REPEAT;
		}
	}
}

uint8_t eepromLoadByte(uint16_t addr) {
	return eeprom_read_byte((uint8_t*)addr);
}

void eepromSaveByte(uint16_t addr, uint8_t v) {
	if(eeprom_read_byte((uint8_t*)addr) != v)  eeprom_write_byte((uint8_t*)addr, v);
}

void saveSettings(void) {
	eepromSaveByte(EE_MENU_ENTRY, menuEntryNum);

// FIXME
	//eepromSaveByte(EE_FREQ_1, (uint8_t)(SG.freq));
	//eepromSaveByte(EE_FREQ_2, (uint8_t)(SG.freq >> 8));
	//eepromSaveByte(EE_FREQ_3, (uint8_t)(SG.freq >> 16));

	//eepromSaveByte(EE_FREQ_STEP_1, (uint8_t)(SG.freqStep));
	//eepromSaveByte(EE_FREQ_STEP_2, (uint8_t)(SG.freqStep >> 8));

	eepromSaveByte(EE_HS_FREQ, SG.hsFreq);

	eepromSaveByte(EE_PWM_FREQ_1, (uint8_t)(SG.pwmFreq));
	eepromSaveByte(EE_PWM_FREQ_2, (uint8_t)(SG.pwmFreq >> 8));

	eepromSaveByte(EE_PWM_DUTY, SG.pwmDuty);
	eepromSaveByte(EE_OFF_LEVEL, SG.offLevel);
}

void initSettings(void) {
	menuEntryNum = 0;
	SG.freq      = 1000.0;
	SG.freqStep  = 100.0;
	SG.hsFreq    = 1;    // default 1MHz HS signal freq
	SG.pwmFreq   = 62500;
	SG.pwmDuty   = 128;
	SG.offLevel  = 0x80; // middle of the scale
	
	saveSettings();
	eeprom_write_byte((uint8_t*)EE_INIT, 'T');   // marks once that eeprom init is done
	//once this procedure is held, no more initialization is performed
}

void loadSettings(void) {
	if(eepromLoadByte(EE_INIT) != 'T') {
		initSettings();
	}

	menuEntryNum = eepromLoadByte(EE_MENU_ENTRY);

// FIXME
	//SG.freq = ((uint32_t)eepromLoadByte(EE_FREQ_1))
		//| ((uint32_t)eepromLoadByte(EE_FREQ_2) << 8)
		//| ((uint32_t)eepromLoadByte(EE_FREQ_3) << 16);

	//SG.freqStep = ((uint32_t)eepromLoadByte(EE_FREQ_STEP_1))
		    //| ((uint32_t)eepromLoadByte(EE_FREQ_STEP_2) << 8);

	SG.hsFreq = eepromLoadByte(EE_HS_FREQ);

	SG.pwmFreq = ((uint16_t)eepromLoadByte(EE_PWM_FREQ_1))
		   | ((uint16_t)eepromLoadByte(EE_PWM_FREQ_2) << 8);

	SG.pwmDuty = eepromLoadByte(EE_PWM_DUTY);
	SG.offLevel = eepromLoadByte(EE_OFF_LEVEL);
}

void buttonNop(void) {
}

void onNewMenuEntry(void) {
	memcpy_P(&menuEntry, &MENU[menuEntryNum], sizeof(menuEntry));
	buttonHandlers = &menuEntry.buttonHandlers;

	LCDclr();
	CopyStringtoLCD(menuEntry.title, 0, 0);
	menuEntry.updateDisplay();
}

void menu_onUp(void) {
	if(!SG.running) {
		if(menuEntryNum == 0) menuEntryNum = MENU_SIZE - 1;
		else                  --menuEntryNum;
		onNewMenuEntry();
	}
}

void menu_onDown(void) {
	if(!SG.running) {
		++menuEntryNum;
		if(menuEntryNum == MENU_SIZE) menuEntryNum = 0;
		onNewMenuEntry();
	}
}

void onNewOptMenuEntry(void) {
	memcpy_P(&menuEntry, &OPT_MENU[optMenuEntryNum], sizeof(menuEntry));
	buttonHandlers = &menuEntry.buttonHandlers;

	LCDclr();
	CopyStringtoLCD(menuEntry.title, 0, 0);
	menuEntry.updateDisplay();
}

void menu_onOpt(void) {
	if(!SG.running) {
		optMenuEntryNum = 0;
		onNewOptMenuEntry();
	}
}

void optMenu_onUp(void) {
	if(!SG.running) {
		if(optMenuEntryNum == 0) optMenuEntryNum = OPT_MENU_SIZE - 1;
		else                     --optMenuEntryNum;
		onNewOptMenuEntry();
	}
}

void optMenu_onDown(void) {
	if(!SG.running) {
		++optMenuEntryNum;
		if(optMenuEntryNum == OPT_MENU_SIZE) optMenuEntryNum = 0;
		onNewOptMenuEntry();
	}
}

void optMenu_onOpt(void) {
	optMenuEntryNum = (uint8_t)-1;
	onNewMenuEntry();
}

void signal_updateDisplay(void) {
	LCDGotoXY(0, 1);
	printf("%10.3fHz", SG.freq);
	CopyStringtoLCD(SG.running ? MNON : MNOFF, 13, 1);
}

void signal_onLeft(void) {
	SG.freq -= SG.freqStep;
	if(SG.freq < MIN_FREQ)
		SG.freq = MIN_FREQ;
	signal_updateDisplay();
}

void signal_onRight(void) {
	SG.freq += SG.freqStep;
	if(SG.freq > MAX_FREQ)
		SG.freq = MAX_FREQ;
	signal_updateDisplay();
}

void disableMenu(void) {
	while(buttonState.pressed != Button_None);       // wait until button release, otherwise the release interrupt will stop the generation
	GICR |= (1 << INT0) | (1 << INT1) | (1 << INT2); // set external interrupts to enable stop or modify

	timer2Stop();  // menu inactive
}

void enableMenu(void) {
	GICR &= ~((1 << INT0) | (1 << INT1) | (1 << INT2)); // stop external interrupts
	timer2Start();                                      // menu active
}

void signal_start(void) {
	saveSettings();

	SG.running = true;

	menuEntry.updateDisplay();
	disableMenu();
}

void signal_run(void) {
	while(SG.running) {
		uint32_t acc = SG.freq/(RESOLUTION*FREQ_CAL); // calculate accumulator value
		SPCR &= ~(1<<CPHA); // clear CPHA bit in SPCR register to allow DDS

		memcpy_P(signalBuffer, pgm_read_ptr(&SIGNALS[menuEntry.tag]), sizeof(signalBuffer));
		signalOut(signalBuffer,
			(uint8_t)(acc >> 16),
			(uint8_t)(acc >> 8),
			(uint8_t)acc);
		R2RPORT = SG.offLevel;

		// generation is interrupted, but not stopped - check buttons and continue
		enableMenu();
		while(buttonState.pressed != Button_None) {
			processButton();
		}
		disableMenu();
	}
}

void signal_stop(void) {
	enableMenu();
	SG.running = false;
	R2RPORT = SG.offLevel;
	menuEntry.updateDisplay();
	while(buttonState.pressed != Button_None); // wait until button release, otherwise the generation will be started again
}

void signal_onStart(void) {
	if(!SG.running) {
		signal_start();
		signal_run();
		signal_stop();
	}
	else {
		SG.running = false;
	}
}

void noise_updateDisplay(void) {
	LCDGotoXY(0, 1);
	CopyStringtoLCD(RND, 0, 1);
	CopyStringtoLCD(SG.running ? MNON : MNOFF, 13, 1);
}

void noise_onStart(void) {
	signal_start();
	SPCR &= ~(1<<CPHA); // clear CPHA bit in SPCR register to allow DDS

	memcpy_P(signalBuffer, NOISE_SIGNAL, sizeof(signalBuffer));
	randomSignalOut(signalBuffer);

	signal_stop();
}

void freqStep_updateDisplay(void) {
	LCDGotoXY(0, 1);
	printf("%10.3fHz", SG.freqStep);
}

void freqStep_onLeft(void) {
	SG.freqStep /= 10;
	if(SG.freqStep < MIN_FREQ_STEP)
		SG.freqStep = MIN_FREQ_STEP;
	freqStep_updateDisplay();
}

void freqStep_onRight(void) {
	SG.freqStep *= 10;
	if(SG.freqStep > MAX_FREQ_STEP)
		SG.freqStep = MAX_FREQ_STEP;
	freqStep_updateDisplay();
}

void hs_updateDisplay(void) {
	LCDGotoXY(0, 1);
	printf(" %5uMHz", SG.hsFreq);
	CopyStringtoLCD(SG.running ? MNON : MNOFF, 13, 1);
}

void hs_restart(void) {
	if(SG.running)  {
		timer1Start(SG.hsFreq);
	}
}

void hs_onLeft(void) {
	if(SG.hsFreq == 1)
		SG.hsFreq = 8;
	else
		SG.hsFreq /= 2;
	hs_updateDisplay();
	hs_restart();
}

void hs_onRight(void) {
	if(SG.hsFreq == 8)
		SG.hsFreq = 1;
	else
		SG.hsFreq *= 2;
	hs_updateDisplay();
	hs_restart();
}

void hs_onStart(void) {
	if(SG.running) {
		SG.running = false;
		timer1Stop();
		HSPORT &= ~(1<<HS);   // set HS pin to LOW FIXME sometimes it stays in HIGH
	}
	else {
		saveSettings();
		SG.running = true;
		menuEntry.updateDisplay();

		hs_restart();
		while(SG.running) {
			processButton();
		}

		menuEntry.updateDisplay();
	}
}

void pwm_updateDisplay(void) {
	double freq;
	switch(SG.pwmFreq) {
		case 61:    freq = 61.04;     break;
		case 244:   freq = 244.14;    break;
		case 976:   freq = 976.56;    break;
		case 7813:  freq = 7812.50;   break;
		default:    freq = 62500.00;  break;
	}

	LCDGotoXY(13, 0);
	printf("%3u", SG.pwmDuty);
	LCDGotoXY(0, 1);
	printf("%8.2fHz", freq);
	CopyStringtoLCD(SG.running ? MNON : MNOFF, 13, 1);
}

void pwm_onStart(void) {
	if(SG.running) {
		SG.running = false;
		timer1Stop();
		HSPORT &= ~(1 << HS);   // set HS pin to LOW
	}
	else {
		saveSettings();
		SG.running = true;
		menuEntry.updateDisplay();

		OCR1A = SG.pwmDuty;
		timer1StartPwm(SG.pwmFreq);
		while(SG.running) {
			processButton();
		}

		menuEntry.updateDisplay();
	}
}

void pwm_onLeft(void) {
	if(!SG.running) {
		switch(SG.pwmFreq) {
			case 61:    SG.pwmFreq = 62500; break;
			case 244:   SG.pwmFreq = 61;    break;
			case 976:   SG.pwmFreq = 244;   break;
			case 7813:  SG.pwmFreq = 976;   break;
			case 62500: SG.pwmFreq = 7813;  break;
		}
		pwm_updateDisplay();
	}
	else {
		if(SG.pwmDuty > 0) --SG.pwmDuty;
		OCR1A = SG.pwmDuty;
		pwm_updateDisplay();
	}
}

void pwm_onRight(void) {
	if(!SG.running) {
		switch(SG.pwmFreq) {
			case 61:    SG.pwmFreq = 244;   break;
			case 244:   SG.pwmFreq = 976;   break;
			case 976:   SG.pwmFreq = 7813;  break;
			case 7813:  SG.pwmFreq = 62500; break;
			case 62500: SG.pwmFreq = 61;    break;
		}
		pwm_updateDisplay();
	}
	else {
		if(SG.pwmDuty < 255) ++SG.pwmDuty;
		OCR1A = SG.pwmDuty;
		pwm_updateDisplay();
	}
}

void sweep_onStart(void) {
	saveSettings();
	SG.running = true;
	menuEntry.updateDisplay();
	disableMenu();

	uint32_t acc = SG.freq/RESOLUTION; // calculate accumulator value
	SPCR &= ~(1<<CPHA); // clear CPHA bit in SPCR register to allow DDS

	memcpy_P(signalBuffer, SINE_WAVE_FROM_ZERO, sizeof(signalBuffer));
	sweepOut(signalBuffer,
		(uint8_t)(acc >> 16),
		(uint8_t)(acc >> 8),
		(uint8_t)acc,
		0,
		0,
		1);

	// output only once
	SG.running = false;
	signal_stop();
}

void offLevel_updateDisplay(void) {
	LCDGotoXY(0, 1);
	printf("%3u", SG.offLevel);
}

void offLevel_onLeft(void) {
	if(SG.offLevel > 0) --SG.offLevel;
	R2RPORT = SG.offLevel;
	offLevel_updateDisplay();
}

void offLevel_onRight(void) {
	if(SG.offLevel < 255) ++SG.offLevel;
	R2RPORT = SG.offLevel;
	offLevel_updateDisplay();
}

//timer overflow interrupt service tourine
//checks all button status and if button is pressed
//value is updated
ISR(TIMER2_OVF_vect)
{
	checkButtons();
}

/*DDS signal generation function
Original idea is taken from
http://www.myplace.nu/avr/minidds/index.htm
small modification is made - added additional command which
checks if CPHA bit is set in SPCR register if yes - exit function
*/
void static inline signalOut(const uint8_t *signal, uint8_t ad2, uint8_t ad1, uint8_t ad0)
{
	asm volatile(
		"eor r18, r18 			; r18<-0"			"\n\t"
		"eor r19, r19 			; r19<-0"			"\n\t"
		"1:"								"\n\t"
		"add r18, %[ad0]		; 1 cycle"			"\n\t"
		"adc r19, %[ad1]		; 1 cycle"			"\n\t"	
		"adc %A[sig], %[ad2]		; 1 cycle"			"\n\t"
		"ld __tmp_reg__, Z 		; 2 cycles" 			"\n\t"
		"out %[out], __tmp_reg__	; 1 cycle"			"\n\t"
		"sbis %[cond], 2		; 1 cycle if no skip" 		"\n\t"
		"rjmp 1b			; 2 cycles. Total 9 cycles"	"\n\t"
		:
		: [ad0] "r"(ad0), [ad1] "r"(ad1), [ad2] "r"(ad2), // phase increment
		  [sig] "z"(signal),                              // signal source
		  [out] "I"(_SFR_IO_ADDR(R2RPORT)),               // output port
		  [cond] "I"(_SFR_IO_ADDR(SPCR))                  // exit condition
		: "r18", "r19" 
	);
}

void static inline randomSignalOut(const uint8_t *signal)
{
	asm volatile(
		"1:"								"\n\t"
		"ld __tmp_reg__, Z 		; 2 cycles" 			"\n\t"
		"inc r30			; 1 cycle"			"\n\t"
		"out %[out], __tmp_reg__	; 1 cycle"			"\n\t"
		"sbis %[cond], 2		; 1 cycle if no skip" 		"\n\t"
		"rjmp 1b			; 2 cycles. Total 7 cycles"	"\n\t"
		:
		: [sig] "z"(signal),                              // signal source
		  [out] "I"(_SFR_IO_ADDR(R2RPORT)),               // output port
		  [cond] "I"(_SFR_IO_ADDR(SPCR))                  // exit condition
	);
}

void static inline sweepOut(const uint8_t *signal, uint8_t ad2, uint8_t ad1, uint8_t ad0,
                             uint8_t ad2i, uint8_t ad1i, uint8_t ad0i)
{
	asm volatile(
		"eor r18, r18 			; r18<-0"			"\n\t"
		"eor r19, r19 			; r19<-0"			"\n\t"
		"ldi r16, 11            	; "				"\n\t" // stop frequency
		"ldi r17, 0             	; "				"\n\t"
		"1:"								"\n\t"
		"add r18, %[ad0]		; 1 cycle"			"\n\t"
		"adc r19, %[ad1]		; 1 cycle"			"\n\t"	
		"adc %A[sig], %[ad2]		; 1 cycle"			"\n\t"
		"breq 2f			; 1 cycle if no jump" 		"\n\t"
		"ld __tmp_reg__, Z		; 2 cycles" 			"\n\t"
		"out %[out], __tmp_reg__	; 1 cycle"			"\n\t"
		"rjmp 1b			; 2 cycles. Total 9 cycles"	"\n\t"
		"2:                     	; "				"\n\t"
		"add %[ad0], %[ad0i]		; "				"\n\t"
		"adc %[ad1], %[ad1i]		; "				"\n\t"
		"adc %[ad2], %[ad2i]		; "				"\n\t"
		"cp %[ad2], r17			; "				"\n\t" // avoid frequency inc on next step at begin
		"brne 3f			; "				"\n\t"
		"inc %A[sig]			; "				"\n\t"
		"3:             	        ; "				"\n\t"
		"cp %[ad2], r16			; "				"\n\t"
		"sbis %[cond], 2		; "		 		"\n\t"
		"brne 1b			; "				"\n\t"
		:
		: [ad0] "r"(ad0), [ad1] "r"(ad1), [ad2] "r"(ad2),      // phase increment
		  [sig] "z"(signal),                                   // signal source
		  [out] "I"(_SFR_IO_ADDR(R2RPORT)),                    // output port
		  [cond] "I"(_SFR_IO_ADDR(SPCR)),                      // exit condition
		  [ad0i] "r"(ad0i), [ad1i] "r"(ad1i), [ad2i] "r"(ad2i) // increment of the increment
		: "r16", "r17", "r18", "r19"
	);
}

// FIXME no HS-signal
void timer1Start(uint8_t freqMHz)
{
	switch(freqMHz) {
		case 2:  OCR1A = 3; break;
		case 4:  OCR1A = 1; break;
		case 8:  OCR1A = 0; break;
		default: OCR1A = 7; // 1 MHz
	}
	TCCR1A = 0x40;       // output compare toggles OC1A pin
	TCCR1B = 0b00001001; // start timer without prescaler
}

void timer1StartPwm(uint16_t freqHz)
{
	uint8_t prescaler;
	switch(freqHz) {
		case 61:   prescaler = 0b101; break;
		case 244:  prescaler = 0b100; break;
		case 976:  prescaler = 0b011; break;
		case 7813: prescaler = 0b010; break;
		default:   prescaler = 0b001; break;
	}
	
	// Fast PWM 8 bit; non-inverting
	TCCR1A = (1 << WGM10) | (1 << COM1A1);
	TCCR1B = (1 << WGM12) | prescaler;
}

void timer1Stop(void)
{
	TCCR1B = 0x00; // timer off
}

void init(void) {
	//stderr = &lcd_str;
	stdout = &lcd_str;

	// init LCD
	LCDinit();
	LCDclr();
	LCDcursorOFF();

	loadSettings();

	SG.running = false;

	// init DDS output
	R2RPORT = SG.offLevel;
	R2RDDR  = 0xFF; // set A port as output

	// set ports pins for buttons
	BDDR   &= ~(_BV(START) | _BV(UP) | _BV(DOWN) | _BV(RIGHT) | _BV(LEFT));
	BPORT  |=  (_BV(START) | _BV(UP) | _BV(DOWN) | _BV(RIGHT) | _BV(LEFT));
	BDDR2  &= ~(_BV(OPT));
	BPORT2  =  (_BV(OPT));

	HSDDR |= _BV(HS); // configure HS as output
	timer2Init();
	enableMenu();
	onNewMenuEntry();
	sei();
}

void processButton(void) {
	if(!buttonState.processed) {
		enum Button pressed = buttonState.pressed;
		buttonState.processed = true;

		switch(pressed) {
			case Button_None:  break;
			case Button_Up:    buttonHandlers->onUp();    break;
			case Button_Right: buttonHandlers->onRight(); break;
			case Button_Down:  buttonHandlers->onDown();  break;
			case Button_Left:  buttonHandlers->onLeft();  break;
			case Button_Start: buttonHandlers->onStart(); break;
			case Button_Opt:   buttonHandlers->onOpt();   break;
		}
	}
}

int main(void) {	
	init();
	while(1) {
		processButton();
	}

	return 0;
}

