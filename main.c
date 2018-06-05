#include <avr/io.h>
#include <avr/interrupt.h>  /* for sei() */
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

#include "Wheel.h"

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

// PROGMEM const char usbHidReportDescriptor[52] = { /* USB report descriptor, size must match usbconfig.h */
//     0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
//     0x09, 0x02,                    // USAGE (Mouse)
//     0xa1, 0x01,                    // COLLECTION (Application)
//     0x09, 0x01,                    //   USAGE (Pointer)
//     0xA1, 0x00,                    //   COLLECTION (Physical)
//     0x05, 0x09,                    //     USAGE_PAGE (Button)
//     0x19, 0x01,                    //     USAGE_MINIMUM
//     0x29, 0x03,                    //     USAGE_MAXIMUM
//     0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
//     0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
//     0x95, 0x03,                    //     REPORT_COUNT (3)
//     0x75, 0x01,                    //     REPORT_SIZE (1)
//     0x81, 0x02,                    //     INPUT (Data,Var,Abs)
//     0x95, 0x01,                    //     REPORT_COUNT (1)
//     0x75, 0x05,                    //     REPORT_SIZE (5)
//     0x81, 0x03,                    //     INPUT (Const,Var,Abs)
//     0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
//     0x09, 0x30,                    //     USAGE (X)
//     0x09, 0x31,                    //     USAGE (Y)
//     0x09, 0x38,                    //     USAGE (Wheel)
//     0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
//     0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
//     0x75, 0x08,                    //     REPORT_SIZE (8)
//     0x95, 0x03,                    //     REPORT_COUNT (3)
//     0x81, 0x06,                    //     INPUT (Data,Var,Rel)
//     0xC0,                          //   END_COLLECTION
//     0xC0,                          // END COLLECTION
// };
PROGMEM const char usbHidReportDescriptor[] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xA1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM
    0x29, 0x03,                    //     USAGE_MAXIMUM
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Const,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x06,                    //     INPUT (Data, Variable, Relative)
    0xC0,                          //   END_COLLECTION
    0xC0,                          // END COLLECTION
};
/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */
typedef struct{
    uchar   buttonMask;
    char    dx;
    char    dy;
    char    dWheel;
}report_t;

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */


static void read_Wheel(void)
{
    reportBuffer.dx = 0;
	reportBuffer.dy = 0;
	reportBuffer.dWheel = 0;

	int Direction = (AnalogIn < 512) ? -1 : 1;

    timing = get_timing(AnalogIn);

    // PINB |= _BV(PB1);

	if (AnalogIn < LowerDead[Debounce] || AnalogIn > UpperDead[Debounce])
	{
		if (Debounce == 1 || ticktock >= timing)
		{
			reportBuffer.dWheel = Direction;
			Debounce = 0;
			ticktock = 0;
            PINB |= _BV(PB1);
            // PINB &= ~(1<<PB1);
			// LEDs_ToggleLEDs(LEDS_LED2);
		}
	}
	else
	{
		if (Debounce == 0)
		{
			// PINB |= (1<<PB1);
			Debounce = 1;
		}
		
	}

    // *ReportSize = sizeof(USB_MouseReport_Data_t);
    // return true;
}

void clearReport()
{
    reportBuffer.dx = 0;
	reportBuffer.dy = 0;
	reportBuffer.dWheel = 0;
    reportBuffer.buttonMask = 0;
}

uint8_t get_timing(uint16_t value)
{
   int index = 0;
   while (value > timingtable_in[index])
   {
      index++;
   }
   return timingtable_out[index];
}

/* ------------------------------------------------------------------------- */

ISR(ADC_vect)
{
	AnalogIn = ADC;
}

ISR(TIM0_COMPA_vect)
{
    // usbPoll();
    // PINB |= _BV(PB1);
    
    if (ticktock > 10)
    {
        // PINB |= _BV(PB1);
        
        // usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        // PINB |= (1<<PB1);
        // PINB |= _BV(PB1);
        read_Wheel();
        if(usbInterruptIsReady())
        {
            // PINB |= _BV(PB1);
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        }
        ticktock = 0;
    }
    else
    {
        // PINB &= ~(1<<PB1);
        ticktock++;
    }
	
    // ticktock++;
	// LEDs_ToggleLEDs(LEDS_LED1);
    // wdt_reset();
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
    cli();
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Hardware Initialization */

    // clearReport();

    usbInit();
    /* enforce re-enumeration, do this while interrupts are disabled! */
    usbDeviceDisconnect();  
    for (int i = 0; i<250; i++)
    {             /* fake USB disconnect for > 250 ms */
        // wdt_reset();
        _delay_ms(4);
    }
    usbDeviceConnect();

	/* ADC Settings */
	// prescaler = 128
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	// VCC Ref, ADC1
	ADMUX = (1<<MUX0);
	ADCSRA |= (1<<ADIE)|(1<<ADSC)|(1<<ADATE);
    ADCSRA |= (1<<ADEN);

	// /* Timer1 settings */
	// TCCR1A = 0;
	// // Mode = CTC, Prescaler = 64
	// TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);


    // TCCR1 |= (1<<CTC1)|(1<<CS13)|(1<<CS11)|(1<<CS10);

	// // initialize counter
	// TCNT1 = 0;
	
	// // initialize compare value
    // OCR1A = 241; // 15ms
    // // OCR1C = 241; // 15ms

	// // // enable compare interrupt
    // TIMSK |= (1 << OCIE1A);


    TCCR0A = (1<<WGM01);
    TCCR0B = (1<<CS02);

    TCNT0 = 0;

    OCR0A = 241;

    TIMSK |= (1<<OCIE0A);

    DDRB |= (1<<PB1);

    sei();
    // wdt_enable(WDTO_1S);
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

// Called by V-USB after device reset
void hadUsbReset()
{
    cli();
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal, bestCal, step, region;

    bestCal = OSCCAL;

    // do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
    for(region = 0; region <= 1; region++) {
        frameLength = 0;
        trialCal = (region == 0) ? 0 : 128;
        
        for(step = 64; step > 0; step >>= 1) { 
            if(frameLength < targetLength) // true for initial iteration
                trialCal += step; // frequency too low
            else
                trialCal -= step; // frequency too high
                
            OSCCAL = trialCal;
            frameLength = usbMeasureFrameLength();
            
            if(abs(frameLength-targetLength) < bestDeviation) {
                bestCal = trialCal; // new optimum found
                bestDeviation = abs(frameLength -targetLength);
            }
        }
    }

    OSCCAL = bestCal;
    sei();
}

/* ------------------------------------------------------------------------- */

int main(void)
{
// uchar   i;

    SetupHardware();
    
    /* If you don't use the watchdog, replace the call above with a wdt_disable().
     * On newer devices, the status of the watchdog (on/off, period) is PRESERVED
     * OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    // for (int a = 0; a<6; a++)
    // {
    //     PINB |= _BV(PB1);
    //     _delay_ms(500);
    // }

    for(;;)
    {                /* main event loop */
        // if (USBIN&USBMASK)
        // {
        //     sleep_cpu();	// sleep, except at SE0, until SOF
        // }
        // DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        //wdt_reset();
        usbPoll();
        // usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        // PINB |= _BV(PB1);
        // if(usbInterruptIsReady())
        // {
        //     // PINB |= _BV(PB1);
            
        // //     /* called after every poll of the interrupt endpoint */
        //     // read_Wheel();
        // //     // DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
        //     usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        //     clearReport();
        // }
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
