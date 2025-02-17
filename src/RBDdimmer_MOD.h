#ifndef RBDDIMMER_H
#define RBDDIMMER_H

#include <stdlib.h>

#include "avr/RBDmcuAVR_MOD.h"

typedef enum
{
    NORMAL_MODE = 0,
    TOGGLE_MODE = 1
} DIMMER_MODE_typedef;

typedef enum
{
    OFF = false,
    ON = true
} ON_OFF_typedef;

class dimmerLamp
{
    private:
		int timer_num;
    bool toggle_state;
    int tog_current;

		void port_init(void);
		void timer_init(void);
		void ext_int_init(void);

    public:
        uint16_t pulse_begin;
        int dimmer_pin;
        int tog_max;
        int tog_min;

#if !(defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAMD))
        int zc_pin;

        dimmerLamp(int user_dimmer_pin, int zc_dimmer_pin);
#else
		dimmerLamp(int user_dimmer_pin);
#endif
        void begin(DIMMER_MODE_typedef DIMMER_MODE, ON_OFF_typedef ON_OFF);
//added by f3rcode: so port_init and timer_init registers are cleared (for experimening purposes)
	void clear(void);

    void setPower(int power);
		int  getPower(void);
		void setState(ON_OFF_typedef ON_OFF);
    bool getState(void);
		//void changeState(void);
	//void setMode(DIMMER_MODE_typedef DIMMER_MODE);
        //DIMMER_MODE_typedef getMode(void);
        //void toggleSettings(int minValue, int maxValue);
};

#endif
