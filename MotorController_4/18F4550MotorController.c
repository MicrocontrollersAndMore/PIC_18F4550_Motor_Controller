// 18F4550MotorController4.c

// chip config ////////////////////////////////////////////////////////////////////////////////////
					// clock options, see 18F4550 data sheet figure 2-1 "clock diagram" for explanation
#pragma config PLLDIV = 5				// 20 MHz external clock / PLL prescaler value of 5 = 4 MHz required input to PLL circuit
#pragma config CPUDIV = OSC1_PLL2		// non-PLL postscale / 1 OR PLL postscale / 2 for CPU clock speed, depending on FOSC setting below
#pragma config USBDIV = 2				// USB clock source = 96 MHz PLL source / 2, (full-speed USB mode)

						// if desired, could change this line to "FOSC = HS" & "oscillator postscaler" gate would be used 
						// (not the "PLL postscaler" gate), CPU speed would be 20MHz, USB circuitry would still receive 48Mhz clock
#pragma config FOSC = HSPLL_HS			// use high-speed external osc crystal, & use PLL postscaler gate to feed CPU (CPU speed = 48 MHz)

					// now the other less confusing options . . .
#pragma config FCMEN = OFF				// fail-safe clock monitor disabled
#pragma config IESO = OFF				// internal / external osc switchover bit disabled
#pragma config PWRT = OFF				// power-up timer disabled
#pragma config BOR = OFF				// brown-out reset disabled in hardware & software
#pragma config BORV = 3					// brown-out reset voltage bits, does not matter since brown-out is disabled 
#pragma config VREGEN = ON				// USB voltage regulator enabled (If using USB, capacitor goes on pin 18 (VUSB))
#pragma config WDT = OFF				// watchdog timer disabled
#pragma config WDTPS = 32768			// watchdog timer postscale, does not matter since watchdog timer is disabled
#pragma config CCP2MX = ON				// use RC1 (pin #16) as CCP2 MUX (this is the default pin for CCP2 MUX)
#pragma config PBADEN = OFF				// RB0, RB1, RB2, RB3, & RB4 are configured as digital I/O on reset
#pragma config LPT1OSC = OFF			// disable low-power option for timer 1 (timer 1 in regular mode)
#pragma config MCLRE = OFF				// master clear disabled, pin #1 is for VPP and / or RE3 use
#pragma config STVREN = ON				// stack full/underflow will cause reset
#pragma config LVP = OFF				// single-supply ICSP disabled
#pragma config ICPRT = OFF				// in-circuit debug/programming port (ICPORT) disabled, this feature is not available on 40 pin DIP package
#pragma config XINST = OFF				// instruction set extension and indexed addressing mode disabled (this is the default setting)
#pragma config DEBUG = OFF				// background debugger disabled, RA6 & RB7 configured as general purpose I/O pins
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF			// code protection bits off
#pragma config CPB = OFF				// boot block code protection off
#pragma config CPD = OFF				// data EEPROM code protection off
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF		// write protection bits off
#pragma config WRTC = OFF				// config registers write protection off
#pragma config WRTB = OFF				// boot block is not write protected
#pragma config WRTD = OFF				// data EEPROM is not write protected
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF	// table read protection bits off
#pragma config EBTRB = OFF				// boot block table read protection off

// includes ///////////////////////////////////////////////////////////////////////////////////////
#include<p18f4550.h>
#include<timers.h>
#include<delays.h>

// #defines ///////////////////////////////////////////////////////////////////////////////////////

#define SERVO_POT						PORTAbits.RA0	// input pins

#define LEFT_FORWARD_SWITCH				PORTAbits.RA1
#define RIGHT_FORWARD_SWITCH			PORTAbits.RA2
#define LEFT_REVERSE_SWITCH				PORTAbits.RA3
#define RIGHT_REVERSE_SWITCH			PORTAbits.RA4
#define STEP_COUNTERCLOCKWISE_SWITCH	PORTAbits.RA5
#define STEP_CLOCKWISE_SWITCH			PORTEbits.RE0

#define SERVO_CONTROL_PIN	PORTBbits.RB4				// output pins

#define H_BRIDGE_1_d	PORTBbits.RB3
#define H_BRIDGE_1_c	PORTBbits.RB2
#define H_BRIDGE_1_b	PORTBbits.RB1
#define H_BRIDGE_1_a	PORTBbits.RB0

#define H_BRIDGE_2_d	PORTDbits.RD7
#define H_BRIDGE_2_c	PORTDbits.RD6
#define H_BRIDGE_2_b	PORTDbits.RD5
#define H_BRIDGE_2_a	PORTDbits.RD4

#define H_BRIDGE_3_d	PORTCbits.RC7
#define H_BRIDGE_3_c	PORTCbits.RC6
#define H_BRIDGE_3_b	PORTDbits.RD3
#define H_BRIDGE_3_a	PORTDbits.RD2

#define H_BRIDGE_4_d	PORTDbits.RD1
#define H_BRIDGE_4_c	PORTDbits.RD0
#define H_BRIDGE_4_b	PORTCbits.RC2
#define H_BRIDGE_4_a	PORTCbits.RC1

#define LED2	PORTEbits.RE1			// signal / debug LEDs
#define LED1	PORTEbits.RE2			//
#define LED0	PORTCbits.RC0			//

// global variables ///////////////////////////////////////////////////////////////////////////////
int g_stepperMotorDirection = 0;		// 0 => stepper motor not moving
										// 1 => stepper motor stepping clockwise
										// 2 => stepper motor stepping counterclockwise
										
int g_stepperPosition = 0;				// current position of stepper motor in full step algorithm

	// full step algorithm is as follows:
	// g_stepperPosition	A coil	B coil
	//		0
	//						  A		  0
	//		1
	//						  0		  B
	//		2
	//						  A'	  0
	//		3
	//						  0		  B'
	//		0

// function prototypes ////////////////////////////////////////////////////////////////////////////
void high_ISR(void);
void yourHighPriorityInterruptCode(void);

void servoControl(void);
void DCmotorControl(void);
void stepperControl(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
#pragma code HIGH_INTERRUPT_VECTOR = 0x000008
void high_ISR(void) {
	_asm goto yourHighPriorityInterruptCode _endasm
}
#pragma code

///////////////////////////////////////////////////////////////////////////////////////////////////
#pragma interrupt yourHighPriorityInterruptCode
void yourHighPriorityInterruptCode(void) {
	
	if(INTCONbits.TMR0IF == 1) {					// if timer 0 interrupt occurred . . .
		INTCONbits.TMR0IF = 0;						// clear TMR0 overflow interrupt flag
		stepperControl();
	}
	
	if(PIR1bits.TMR1IF == 1) {						// if timer 1 interrupt occurred . . .
		PIR1bits.TMR1IF = 0;
		servoControl();
	}
	
}
#pragma code

///////////////////////////////////////////////////////////////////////////////////////////////////
void main(void) {
	
								// until we have configured timers and related functionality, turn timers off
	T0CONbits.TMR0ON = 0;			// timer 0 off
	T1CONbits.TMR1ON = 0;			// timer 1 off
	T2CONbits.TMR2ON = 0;			// timer 2 off
	T3CONbits.TMR3ON = 0;			// timer 3 off
	
		// config input pins, pin #
	TRISAbits.RA0 = 1;		// 2
	TRISAbits.RA1 = 1;		// 3
	TRISAbits.RA2 = 1;		// 4
	TRISAbits.RA3 = 1;		// 5
	TRISAbits.RA4 = 1;		// 6
	TRISAbits.RA5 = 1;		// 7
	TRISEbits.RE0 = 1;		// 8
	
		// config output pins, pin #
	TRISEbits.RE1 = 0;		// 9
	TRISEbits.RE2 = 0;		// 10
	TRISCbits.RC0 = 0;		// 15
	TRISCbits.RC1 = 0;		// 16
	TRISCbits.RC2 = 0;		// 17
	TRISDbits.RD0 = 0;		// 19
	TRISDbits.RD1 = 0;		// 20
	TRISDbits.RD2 = 0;		// 21
	TRISDbits.RD3 = 0;		// 22
	TRISCbits.RC6 = 0;		// 25
	TRISCbits.RC7 = 0;		// 26
	TRISDbits.RD4 = 0;		// 27
	TRISDbits.RD5 = 0;		// 28
	TRISDbits.RD6 = 0;		// 29
	TRISDbits.RD7 = 0;		// 30
	TRISBbits.RB0 = 0;		// 33
	TRISBbits.RB1 = 0;		// 34
	TRISBbits.RB2 = 0;		// 35
	TRISBbits.RB3 = 0;		// 36
	TRISBbits.RB4 = 0;		// 37
	
		// init all output pins to off	
	SERVO_CONTROL_PIN = 0;

	H_BRIDGE_1_d = 0;
	H_BRIDGE_1_c = 0;
	H_BRIDGE_1_b = 0;
	H_BRIDGE_1_a = 0;

	H_BRIDGE_2_d = 0;
	H_BRIDGE_2_c = 0;
	H_BRIDGE_2_b = 0;
	H_BRIDGE_2_a = 0;

	H_BRIDGE_3_d = 0;
	H_BRIDGE_3_c = 0;
	H_BRIDGE_3_b = 0;
	H_BRIDGE_3_a = 0;

	H_BRIDGE_4_d = 0;
	H_BRIDGE_4_c = 0;
	H_BRIDGE_4_b = 0;
	H_BRIDGE_4_a = 0;

	LED2 = 0;
	LED1 = 0;
	LED0 = 0;
								// A to D config stuff
	ADCON0bits.CHS3 = 0;			// set analog channel to AN0 / RA0
	ADCON0bits.CHS2 = 0;			//
	ADCON0bits.CHS1 = 0;			//
	ADCON0bits.CHS0 = 0;			//
	
	ADCON1bits.VCFG1 = 0;			// set A/D converter negative reference to gnd
	ADCON1bits.VCFG0 = 0;			// set A/D converter positive reference to pwr
	
	ADCON1bits.PCFG3 = 1;			// set RA0 / AN0 to analog input, all other pins digital input
	ADCON1bits.PCFG2 = 1;			//
	ADCON1bits.PCFG1 = 1;			//
	ADCON1bits.PCFG0 = 0;			//
	
	ADCON2bits.ADFM = 0;			// left justify A/D result, "8 bit" A/D result will be in reg ADRESH
	
	ADCON2bits.ACQT2 = 1;			// set A/D acq time
	ADCON2bits.ACQT1 = 0;			//
	ADCON2bits.ACQT0 = 1;			//
	
	ADCON2bits.ADCS2 = 1;			// set A/D conversion clock
	ADCON2bits.ADCS1 = 0;			//
	ADCON2bits.ADCS0 = 1;			//
	
	ADCON0bits.ADON = 1;			// turn A/D converter on
								// end A to D config stuff
	
	RCONbits.IPEN = 1;				// enable priority level on interrupts
	INTCONbits.GIE_GIEH = 1;		// enable high-priority interrupts
	
								// timer 0 config
	INTCONbits.TMR0IE = 1;			// enable timer 0 overflow interrupt
	INTCON2bits.TMR0IP = 1;			// set timer 0 overflow interrupt to high priority
	T0CONbits.T08BIT = 0;			// set timer 0 to 16 bits
	T0CONbits.T0CS = 0;				// use internal clock to increment timer 0
	T0CONbits.PSA = 0;				// use prescaler with timer 0
	
	T0CONbits.T0PS2 = 1;			// timer 0 prescaler 1:64
	T0CONbits.T0PS1 = 0;			//
	T0CONbits.T0PS0 = 1;			//
								// end timer 0 config
	
								// timer 1 config
	PIE1bits.TMR1IE = 1;			// enable timer 1 overflow interrupt
	IPR1bits.TMR1IP = 1;			// timer 1 overflow interrupt priority set to high
	
	T1CONbits.T1CKPS1 = 0;			// timer 1 prescale 1:2
	T1CONbits.T1CKPS0 = 1;			//
	
	T1CONbits.T1OSCEN = 0;			// turn off separate oscillator that is internal to timer 1
	T1CONbits.TMR1CS = 0;			// use internal clock to increment timer 1
								// end timer 1 config
	
								// turn timers on
	T0CONbits.TMR0ON = 1;			// timer 0 on
	T1CONbits.TMR1ON = 1;			// timer 1 on
	
	while(1) {						// loop forever
									// we call the DC motor function directly,
		DCmotorControl();			// note that the interrupts will call the servo and stepper functions
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void servoControl(void) {
	int i;
	int servo_delay;
	
	ADCON0bits.GO_DONE = 1;			// start A/D conversion
	
	while(ADCON0bits.GO_DONE == 1) {
		// wait here until GO_DONE != 1
	}
	
	servo_delay = (int)(((float)ADRESH / 12.0)*30.0);
	
						// now we begin servo pulse high, pulse times are:
						// 1.0 ms => servo at   0 deg
						// 1.5 ms => servo at  90 deg
						// 2.0 ms => servo at 180 deg
	SERVO_CONTROL_PIN = 1;			// begin pulse high
	
	for(i=-135; i<servo_delay; i++) {
		Delay10TCY();			// TCY = 0.083333us, delay = 0.083333us * 10 = 0.83333us per iteration * servo_delay
	}
	SERVO_CONTROL_PIN = 0;			// end pulse high
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void DCmotorControl(void) {
								// remember all switches are logic low !!
	
	if(LEFT_FORWARD_SWITCH == 1 && LEFT_REVERSE_SWITCH == 1) {				// left motor coast
		H_BRIDGE_1_d = 0;
		H_BRIDGE_1_c = 0;
		H_BRIDGE_1_b = 0;
		H_BRIDGE_1_a = 0;
	} else if(LEFT_FORWARD_SWITCH == 0 && LEFT_REVERSE_SWITCH == 1) {		// left motor forward
		H_BRIDGE_1_d = 1;
		H_BRIDGE_1_c = 0;
		H_BRIDGE_1_b = 1;
		H_BRIDGE_1_a = 0;
	} else if(LEFT_FORWARD_SWITCH == 1 && LEFT_REVERSE_SWITCH == 0) {		// left motor reverse
		H_BRIDGE_1_d = 0;
		H_BRIDGE_1_c = 1;
		H_BRIDGE_1_b = 0;
		H_BRIDGE_1_a = 1;
	} else if(LEFT_FORWARD_SWITCH == 0 && LEFT_REVERSE_SWITCH == 0) {		// left motor brake
		H_BRIDGE_1_d = 0;
		H_BRIDGE_1_c = 1;
		H_BRIDGE_1_b = 1;
		H_BRIDGE_1_a = 0;
	}
	
	if(RIGHT_FORWARD_SWITCH == 1 && RIGHT_REVERSE_SWITCH == 1) {			// right motor coast
		H_BRIDGE_2_d = 0;
		H_BRIDGE_2_c = 0;
		H_BRIDGE_2_b = 0;
		H_BRIDGE_2_a = 0;
	} else if(RIGHT_FORWARD_SWITCH == 0 && RIGHT_REVERSE_SWITCH == 1) {		// right motor forward
		H_BRIDGE_2_d = 1;
		H_BRIDGE_2_c = 0;
		H_BRIDGE_2_b = 1;
		H_BRIDGE_2_a = 0;
	} else if(RIGHT_FORWARD_SWITCH == 1 && RIGHT_REVERSE_SWITCH == 0) {		// right motor reverse
		H_BRIDGE_2_d = 0;
		H_BRIDGE_2_c = 1;
		H_BRIDGE_2_b = 0;
		H_BRIDGE_2_a = 1;
	} else if(RIGHT_FORWARD_SWITCH == 0 && RIGHT_REVERSE_SWITCH == 0) {		// right motor brake
		H_BRIDGE_2_d = 0;
		H_BRIDGE_2_c = 1;
		H_BRIDGE_2_b = 1;
		H_BRIDGE_2_a = 0;
	}
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void stepperControl(void) {
								// remember all switches are logic low !!
	
	if(STEP_CLOCKWISE_SWITCH == 0) {					// if forward switch is on . . .
		g_stepperMotorDirection = 1;					// set global variable direction to clockwise
	} else if(STEP_COUNTERCLOCKWISE_SWITCH == 0) {		// else if reverse switch is on . . .
		g_stepperMotorDirection = 2;					// set global variable direction to counterclockwise
	} else {											// else if neither switch is on . . .
		g_stepperMotorDirection = 0;					// set global variable direction to off
	}
	
	if(g_stepperMotorDirection == 0) {				// if we are not stepping either way . . .
		return;
	} else if(g_stepperMotorDirection == 1) {				// if we are stepping forward . . .
		
		if(g_stepperPosition == 0) {
			
			H_BRIDGE_3_d = 1;		// A
			H_BRIDGE_3_c = 0;		//
			H_BRIDGE_3_b = 1;		//
			H_BRIDGE_3_a = 0;		//
			
			H_BRIDGE_4_d = 0;		// 0 (coil B)
			H_BRIDGE_4_c = 0;		//
			H_BRIDGE_4_b = 0;		//
			H_BRIDGE_4_a = 0;		//
			
		} else if(g_stepperPosition == 1) {
			
			H_BRIDGE_3_d = 0;		// 0 (coil A)
			H_BRIDGE_3_c = 0;		//
			H_BRIDGE_3_b = 0;		//
			H_BRIDGE_3_a = 0;		//
			
			H_BRIDGE_4_d = 1;		// B
			H_BRIDGE_4_c = 0;		//
			H_BRIDGE_4_b = 1;		//
			H_BRIDGE_4_a = 0;		//
			
		} else if(g_stepperPosition == 2) {
			
			H_BRIDGE_3_d = 0;		// A'
			H_BRIDGE_3_c = 1;		//
			H_BRIDGE_3_b = 0;		//
			H_BRIDGE_3_a = 1;		//
			
			H_BRIDGE_4_d = 0;		// 0 (coil B)
			H_BRIDGE_4_c = 0;		//
			H_BRIDGE_4_b = 0;		//
			H_BRIDGE_4_a = 0;		//
			
		} else if(g_stepperPosition == 3) {
			
			H_BRIDGE_3_d = 0;		// 0 (coil A)
			H_BRIDGE_3_c = 0;		//
			H_BRIDGE_3_b = 0;		//
			H_BRIDGE_3_a = 0;		//
			
			H_BRIDGE_4_d = 0;		// B'
			H_BRIDGE_4_c = 1;		//
			H_BRIDGE_4_b = 0;		//
			H_BRIDGE_4_a = 1;		//
			
		}
		g_stepperPosition++;
		if(g_stepperPosition >= 4) g_stepperPosition = 0;
		
	} else if (g_stepperMotorDirection == 2) {		// if we are stepping in reverse . . .
		
		if(g_stepperPosition == 1) {
			
			H_BRIDGE_3_d = 1;		// A
			H_BRIDGE_3_c = 0;		//
			H_BRIDGE_3_b = 1;		//
			H_BRIDGE_3_a = 0;		//
			
			H_BRIDGE_4_d = 0;		// 0 (coil B)
			H_BRIDGE_4_c = 0;		//
			H_BRIDGE_4_b = 0;		//
			H_BRIDGE_4_a = 0;		//
			
		} else if(g_stepperPosition == 2) {
			
			H_BRIDGE_3_d = 0;		// 0 (coil A)
			H_BRIDGE_3_c = 0;		//
			H_BRIDGE_3_b = 0;		//
			H_BRIDGE_3_a = 0;		//
			
			H_BRIDGE_4_d = 1;		// B
			H_BRIDGE_4_c = 0;		//
			H_BRIDGE_4_b = 1;		//
			H_BRIDGE_4_a = 0;		//
			
		} else if(g_stepperPosition == 3) {
			
			H_BRIDGE_3_d = 0;		// A'
			H_BRIDGE_3_c = 1;		//
			H_BRIDGE_3_b = 0;		//
			H_BRIDGE_3_a = 1;		//
			
			H_BRIDGE_4_d = 0;		// 0 (coil B)
			H_BRIDGE_4_c = 0;		//
			H_BRIDGE_4_b = 0;		//
			H_BRIDGE_4_a = 0;		//
			
		} else if(g_stepperPosition == 0) {
			
			H_BRIDGE_3_d = 0;		// 0 (coil A)
			H_BRIDGE_3_c = 0;		//
			H_BRIDGE_3_b = 0;		//
			H_BRIDGE_3_a = 0;		//
			
			H_BRIDGE_4_d = 0;		// B'
			H_BRIDGE_4_c = 1;		//
			H_BRIDGE_4_b = 0;		//
			H_BRIDGE_4_a = 1;		//
			
		}
		g_stepperPosition--;
		if(g_stepperPosition <= -1) g_stepperPosition = 3;

	}
	
}
