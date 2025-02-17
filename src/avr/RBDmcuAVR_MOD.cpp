#if defined(ARDUINO_ARCH_AVR)

#include "RBDmcuAVR_MOD.h"

#define _TCNT(X) TCNT ## X
#define TCNT(X) _TCNT(X)
#define _TCCRxA(X) TCCR ## X ## A
#define TCCRxA(X) _TCCRxA(X)
#define _TCCRxB(X) TCCR ## X ## B
#define TCCRxB(X) _TCCRxB(X)
#define _TIMSKx(X) TIMSK ## X
#define TIMSKx(X) _TIMSKx(X)
#define _TIFRx(X) TIFR ## X
#define TIFRx(X) _TIFRx(X)
#define _OCFxA(X) OCF ## X ## A
#define OCFxA(X) _OCFxA(X)
#define _OCFxB(X) OCF ## X ## B
#define OCFxB(X) _OCFxB(X)
#define _TOIEx(X) TOIE ## X
#define TOIEx(X) _TOIEx(X)
#define _OCIExA(X) OCIE ## X ## A
#define OCIExA(X) _OCIExA(X)
#define _TIMER_COMPA_VECTOR(X) TIMER ## X ## _COMPA_vect
#define TIMER_COMPA_VECTOR(X) _TIMER_COMPA_VECTOR(X)

int pulseWidth = 2;

volatile uint16_t dimPower;
volatile uint16_t dimOutPin;
volatile uint16_t zeroCross;
volatile ON_OFF_typedef dimState;
volatile uint16_t dimCounter;
static uint16_t dimPulseBegin;

dimmerLamp::dimmerLamp(int user_dimmer_pin):
	dimmer_pin(user_dimmer_pin)
{
	timer_num = DIMMER_TIMER;
	toggle_state = false;

	dimPulseBegin = 1;
	dimOutPin = user_dimmer_pin;
	dimCounter = 0;
	zeroCross = 0;
	pinMode(user_dimmer_pin, OUTPUT);
}

void dimmerLamp::timer_init(void)
{
	TCCRxA(DIMMER_TIMER) &= ~(0xFF); // clean TCCRxA register
	TCCRxB(DIMMER_TIMER) &= ~(0xFF); // clean TCCRxB register

  //set the interrupt enable bit of the 1st counter to match OCR1A(H and L)
	TIMSKx(DIMMER_TIMER) |= (1 << OCIExA(DIMMER_TIMER));

	TCCRxB(DIMMER_TIMER) = TCCRxB_VALUE; //0x09 =  0b1010  (1 << WGMx2)|(1 << CSx1)
	if (DIMMER_TIMER != 2) OCRxAH(DIMMER_TIMER) = OCRxAH_VALUE; //0x00
	OCRxAL(DIMMER_TIMER) = OCRxAL_VALUE; //0xFF

	TIMSKx(DIMMER_TIMER) |= (1 << TOIEx(DIMMER_TIMER)); //timer interrupt enable
}

void dimmerLamp::ext_int_init(void)
{
	EICRX &= ~0xFF;

	EIMSK |= (1 << INTx);
	EICRX |= (1 << ISCx1)|(1 << ISCx0);//0b00001100
}


void dimmerLamp::begin(DIMMER_MODE_typedef DIMMER_MODE, ON_OFF_typedef ON_OFF)
{
	dimState = ON_OFF;
	timer_init();
	ext_int_init();
}

void dimmerLamp::setPower(int power)
{
	if (power >= 99)
	{
		power = 99;
	}
	dimPower = power;
	dimPulseBegin = powerBuf[power];

	delay(1);
}

int dimmerLamp::getPower(void)
{
	if (dimState == ON)
		return dimPower;
	else return 0;
}

void dimmerLamp::setState(ON_OFF_typedef ON_OFF)
{
	dimState = ON_OFF;
}

bool dimmerLamp::getState(void)
{
	bool ret;
	if (dimState == ON) ret = true;
	else ret = false;
	return ret;
}

ISR(INT_vect)
{
//Serial.println("i");
		if (dimState == ON)
		{
			zeroCross = 1;
		}
}

static int k;
ISR (TIMER_COMPA_VECTOR(DIMMER_TIMER))
{
		if (zeroCross == 1 )
		{
			dimCounter++;

			/*****
			 * DEFAULT DIMMING MODE (NOT TOGGLE)
			 *****/
			if (dimCounter >= dimPulseBegin )
			{
				digitalWrite(dimOutPin, HIGH);

			}

			if (dimCounter >=  (dimPulseBegin + pulseWidth) )
			{
				digitalWrite(dimOutPin, LOW);
				zeroCross = 0;
				dimCounter = 0;

			}

	}

	TIFRx(DIMMER_TIMER) |= ((1<<OCFxB(DIMMER_TIMER))|(1<<OCFxA(DIMMER_TIMER)));
}


void dimmerLamp::clear(void){

	//TCCR2A &= ~(0xFF); // clean TCCRxA register
	//TCCR2B &= ~(0xFF); // clean TCCRxB register
	TIMSK2 &= (0 << OCIE2A);
	//OCRxAH(DIMMER_TIMER) = OCRxAH_VALUE;
	//OCRxAL(DIMMER_TIMER) = OCRxAH_VALUE;

	//EICRA &= ~0xFF; //low level of int0 and int1 generates an interruption

	//TIMSK2 &= (0 << TOIE2); //timer interrupt overflow disable
	EIMSK &= (0 << INT0);

}
#endif
