//*****************************************************************************
// ENCE361 - Helicopter Assignment
// James Wagner - 89960097
// Mathew Hylkema - 35742180
//
// This is the final submission for the Helicopter Rig Control Assignment.
// It is an interrupt based program that implements PID controllers to
// allow the helicopter rig to fly. It includes ADC and pin change interrupt
// and outputs PWM to control the main and tail rotors of the helicopter.
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "boards/ek-lm3s1968/drivers/rit128x96x4.h"
#include "stdio.h"
#include "stdlib.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define MAX_32BIT_VAL 0X0FFFFFF
#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 50L
#define PWM_RATE_HZ 150
#define PWM_DIV_CODE SYSCTL_PWMDIV_2
#define PWM_DIVIDER 2
#define PWM_DEF_DUTY 0

//*****************************************************************************
// Buffer type declaration - set of unsigned longs
//*****************************************************************************
typedef struct {
	unsigned int size;	// Number of entries in buffer
	unsigned int windex;	// index for writing, mod(size)
	unsigned int rindex;	// index for reading, mod(size)
	unsigned long *data;	// pointer to the data
} circBuf_t;

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;
unsigned long ulInitialValue;

signed long yaw, duty1 = PWM_DEF_DUTY, duty4 = PWM_DEF_DUTY;
signed long desiredAltitude = 0, desiredYaw = 0, tempAltitude = 0;
int running = false, landing = false, takeoff = false; // Helicopter States


//*****************************************************************************
// SysTick Interrupt Handler:
//  Used to make a function call to the ADC to get a value
//*****************************************************************************
void
SysTickIntHandler(void)
{
    ADCProcessorTrigger(ADC0_BASE, 3);

}

//*****************************************************************************
// Button Pin Change Interrupt Handler:
//	Reads all pins and performs an action depending on which was pressed
//*****************************************************************************
void
ButChangeIntHandler (void)
{
	unsigned long ulPin1, ulPin2, ulPin3, ulPin4, ulPin5, ulPin6;

	// Clear the interrupt
	GPIOPinIntClear (GPIO_PORTB_BASE,
			GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

	// Read the pins
	ulPin1 = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_1);
	ulPin2 = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_2);
	ulPin3 = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_3);
	ulPin4 = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_4);
	ulPin5 = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_5);
	ulPin6 = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_6);

	if (ulPin1 == 0) {
		// RESET - Call SysCtlReset() to reset the program
		SysCtlReset();
		running = false;
	} else if (ulPin2 == 0) {
		// CW - While in motion, increment the desired yaw 15 degrees clockwise
		if (running && !landing) {
			desiredYaw += 18;
		}
	} else if (ulPin3 == 0) {
		// CCW - While in motion, increment the desired yaw 15 degrees anticlockwise
		if (running && !landing) {
			desiredYaw -= 18;
		}
	} else if (ulPin4 == 0) {
		// SELECT - Toggle the states of the helicopter
		if (!running) {
			running = true;
			duty1 = 5;
			duty4 = 5;
		} else if (running) {
			landing = true;
		}
		tempAltitude = 0;
	} else if (ulPin5 == 0) {
		// UP - Increments the desired altitude, keeps it within 0-100%
		//  	ignores it if helicopter is landing
		if (running && !landing) {
			if (desiredAltitude == 0) {
				// Temporary altitude is used within takeoff function to temporarily set the altitude
				if (tempAltitude <= 90) {
					tempAltitude += 10;
				}
				takeoff = true;
			} else if (desiredAltitude > 0 && desiredAltitude <= 90) {
				if (tempAltitude <= 90) {
					tempAltitude += 10;
				}
				desiredAltitude += 10;
			} else if (desiredAltitude < 0) {
				desiredAltitude = 0;
			} else if (desiredAltitude > 100) {
				desiredAltitude = 100;
			}
		}
	} else if (ulPin6 == 0) {
		// DOWN - Increments the desired altitude, also keeps within 0-100% range
		//		  Does not increment if helicopter is landing
		if (running && !landing) {
			if (tempAltitude  >= 10) {
				tempAltitude -= 10; // Used for takeoff function
			}
			if (desiredAltitude >= 10 && desiredAltitude <= 100) {
				desiredAltitude -= 10;
			} else if (desiredAltitude < 0) {
				desiredAltitude = 0;
			} else if (desiredAltitude > 100) {
				desiredAltitude = 100;
			}
		}
	}
}

//*****************************************************************************
// The interrupt handler for the for the pin change interrupt.  These pins
//  are used for yaw calculation which is incremented depending on which
//  state the pins were previously in and their current state.
//*****************************************************************************
void
PinChangeIntHandler (void)
{
	unsigned long ulPortVal27;
	unsigned long ulPortVal29;
	static int stateA;
	static int stateB;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5);
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_7);

	// Read the pins
	ulPortVal27 = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_5);
	ulPortVal29 = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_7);

	// Depending on previous interrupt state and current state, yaw is incremented
	if (stateA && stateB) {
		if (!ulPortVal27) {
			yaw--;
		} else if (!ulPortVal29) {
			yaw++;
		}
	} else if (stateA && !stateB) {
		if (!ulPortVal27) {
			yaw++;
		} else if (ulPortVal29) {
			yaw--;
		}
	} else if (!stateA && !stateB) {
		if (ulPortVal27) {
			yaw--;
		} else if (ulPortVal29) {
			yaw++;
		}
	} else if (!stateA && stateB) {
		if (ulPortVal27) {
			yaw++;
		} else if (!ulPortVal29) {
			yaw--;
		}
	}

	// Sets the current states for next interrupt
	if (ulPortVal27) {
		stateA = 1;
	} else {
		stateA = 0;
	}

	if (ulPortVal29) {
		stateB = 1;
	} else {
		stateB = 0;
	}
}

//*****************************************************************************
// ADC interrupt service routine:
//  Collects a single sample from ADC0 and stores it in a circuler buffer
//*****************************************************************************
void
ADCIntHandler(void)
{
	unsigned long ulValue;
	static int run;

	// If this is the first time running the ISR,
	//  Get the single sample from ADC0 and store in ulInitialValue.
	if (run == 0) {
		ADCSequenceDataGet(ADC0_BASE, 3, &ulInitialValue);
		run = 1;
	}

	// Get the single sample from ADC0 and store in ulValue.
	ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

	// Places it in the circular buffer
	g_inBuffer.data[g_inBuffer.windex] = (int) ulValue;
	g_inBuffer.windex++;
	if (g_inBuffer.windex >= g_inBuffer.size)
		g_inBuffer.windex = 0;

	// Clean up, clearing the interrupt
	ADCIntClear(ADC0_BASE, 3);
}



//*****************************************************************************
// Clock Initialisation functions:
//  Sets the CPU Clockrate along with the SysTick period
//	Initialises the SysTick Interrupt
//*****************************************************************************
void
initClock (void)
{
	// Sets the CPU Clockrate to 20 MHz
	SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_8MHZ);

	// Set up the period for the SysTick timer to get the maximum span.
	SysTickPeriodSet (SysCtlClockGet() / SAMPLE_RATE_HZ);

	// Register the interrupt handler
	SysTickIntRegister(SysTickIntHandler);

	// Enable SysTick Interrupt
	SysTickIntEnable ();
	SysTickEnable ();
}

//*****************************************************************************
// Button Pins Initialisation:
//  This function enables the pins corresponding to the buttons and
//  configues them, also sets up the interrupt for falling edge
//*****************************************************************************
void
initButPins (void)
{
    // Register the handler for Port B into the vector table
    GPIOPortIntRegister (GPIO_PORTB_BASE, ButChangeIntHandler);

    // Enable and configure the port and pin used:  input on PB1 - PB6
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    GPIOPadConfigSet (GPIO_PORTB_BASE,
    		GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6,
    		GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Set up the pin change interrupt (falling edge)
    GPIOIntTypeSet (GPIO_PORTB_BASE,
    		GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
    		GPIO_FALLING_EDGE);

    // Enable the pin change interrupt
    GPIOPinIntEnable (GPIO_PORTB_BASE,
    		GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    IntEnable (INT_GPIOB);	// Note: INT_GPIOB defined in inc/hw_ints.h
}

//*****************************************************************************
// Pin Initialisation:
//  This function enables the pins corresponding to the yaw control and
//  configues them, also sets up the interrupt for both edges of the pins.
//*****************************************************************************
void
initPin (void)
{
    // Register the handler for Port F into the vector table.
    GPIOPortIntRegister (GPIO_PORTF_BASE, PinChangeIntHandler);
    // Enable and configure the port and pin used:  input on PF5 & PF7: Pin 27 and Pin 29.
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPU);
    // Set up the pin change interrupt (both edges).
    GPIOIntTypeSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_BOTH_EDGES);
    // Enable the pin change interrupt.
    GPIOPinIntEnable (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7);
    IntEnable (INT_GPIOF);	// Note: INT_GPIOF defined in inc/hw_ints.h
}


//*****************************************************************************
// ADC Initialisation:
//  This function enables the ADC pins corresponding to the altitude control,
//  configues them, and also sets up the interrupt.
//*****************************************************************************
void
initADC (void)
{
	// Enable the ADC0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	// Enable sample sequence 3 with a processor signal trigger.
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

	// Configure step 0 on sequence 3.
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
							 ADC_CTL_END);

	// Enable sequence 3.
	ADCSequenceEnable(ADC0_BASE, 3);

	// Register the ADC interrupt handler.
	ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

	// Enable the ADC interrupt.
	ADCIntEnable(ADC0_BASE, 3);
}


//******************************************************************
// Initialise the PWM generators:
//  This function initialises the two PWM pins and configures them.
//  These pins are the output PWM used for the main and tail rotors.
//******************************************************************
void
initPWMchan (void)
{
	unsigned long period;

	// Enable the two ports containing the PWM outputs
	SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD | SYSCTL_PERIPH_GPIOF);

	GPIOPinTypePWM (GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinTypePWM (GPIO_PORTF_BASE, GPIO_PIN_2);

    SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM);

    // Compute the PWM period based on the system clock.
    SysCtlPWMClockSet (PWM_DIV_CODE);
    period = SysCtlClockGet () / PWM_DIVIDER / PWM_RATE_HZ;

    // Configure PWM1 and its output state
    PWMGenConfigure (PWM_BASE, PWM_GEN_0,
    		PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet (PWM_BASE, PWM_GEN_0, period);
    PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * duty1 / 100);
	PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable (PWM_BASE, PWM_GEN_0);

	// Configure PWM4 and its output state
	PWMGenConfigure (PWM_BASE, PWM_GEN_2,
			PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet (PWM_BASE, PWM_GEN_2, period);
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * duty1 / 100);
	PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
	PWMGenEnable (PWM_BASE, PWM_GEN_2);

}

//*****************************************************************************
// Display Initialisation Function
//*****************************************************************************
void
initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);
}

//*****************************************************************************
// Circular Buffer Initialisation Function
//*****************************************************************************
unsigned long *
initCircBuf (circBuf_t *buffer, unsigned int size)
{
	buffer->windex = 0;
	buffer->rindex = 0;
	buffer->size = size;
	buffer->data =
        (unsigned long *) calloc (size, sizeof(unsigned long));
     // Note use of calloc() to clear contents.
	return buffer->data;
}

//******************************************************************
// Update PWM1 Function:
//  This function is where the PID control algorithm takes place
//  and updates the duty cycle depending on the current and desired
//  altitude values. Also holds the take off and landing tasks.
//******************************************************************
void
updatePWM1(long meanAltitude)
{
	signed long altitudeError;
	static signed long previousAltError;
	static float altErrorIntegrated;
	float altErrorDerivative;
	float Kp = 0.3;		// PID Proportional Constant
	float Ki = 0.01;	// PID Integral Constant
	float Kd = 0.8;		// PID Derivative Constant
	unsigned long period = SysCtlClockGet () / PWM_DIVIDER / PWM_RATE_HZ;
	static int count = 0;

	//The following action is a critical section to eliminate the chance of data sharing problems.
	IntMasterDisable();

	// Loop used for when helicopter is in landing state. Lowers the helicopter until
	//  the ground is reached at which point it will toggle the running state off.
	if (landing) {
		count++;
		// Decrease the desired altitude by 1% every 600 cycles to slowly
		//  lower the helicopter.
		if (count > 600) {
			desiredAltitude -= 1;
			count = 0;
		}
		// Once the helicopter reaches 0% desired altitude, turn off landing state,
		//  disable running state and reset the yaw postion.
		if (desiredAltitude == 0) {
			landing = false;
			running = false;
			desiredAltitude = 0;
			altErrorIntegrated = 0;
			yaw = 0;
			desiredYaw = 0;
		}
	}

	// Takeoff loop used to initially lift the helicopter off the ground.
	if (takeoff) {
		// When the current altitude is less than 8% it will keep the desired
		//  altitude at 50% to speed up the takeoff.
		if (meanAltitude < 8) {
			desiredAltitude = 50;
		}
		// This is used when the helicopter is over 8% altitude.
		//  Stopping takeoff state and setting desired to temperary altitude.
		else {
			desiredAltitude = tempAltitude;
			tempAltitude = 0;
			takeoff = false;
		}
		// However if the temporary desired altitude is set above 30% this will kick in
		//  setting the desired altitude to the temporary value and toggling takeoff state off.
		if (tempAltitude >= 30) {
			desiredAltitude = tempAltitude;
			tempAltitude = 0;
			takeoff = false;
		}
	}

	// The following section of this function is the PID algorithm.

	altitudeError = desiredAltitude - meanAltitude;

	IntMasterEnable();

	altErrorIntegrated += altitudeError * 0.001;

	altErrorDerivative = (altitudeError-previousAltError)/0.001;

	previousAltError = altitudeError;

	// This is the PID control calculation.
	duty1 = (Kp * altitudeError) + (Ki * altErrorIntegrated) + (Kd * altErrorDerivative);

	// Keeps the duty cycle within its bounds.
	if (running) {
		if (duty1 < 10) {
			duty1 = 10;
		} else if (duty1 > 95) {
			duty1 = 95;
		}
	} else if (!running) {
		duty1 = 0;
	}

	// Update the PWM width set using updated duty cycle.
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * duty1 /100);
}

//******************************************************************
// Update PWM4 Function:
//  This function is where the PID control algorithm takes place
//  and updates the duty cycle depending on the current and desired
//  altitude values.
//******************************************************************
void
updatePWM4(long yaw)
{
	signed long yawError;
	static signed long previousYawError;
	static float yawErrorIntegrated;
	float yawErrorDerivative;
	float Kp = 0.2;
	float Ki = 0.005;
	float Kd = 2;
	unsigned long period = SysCtlClockGet () / PWM_DIVIDER / PWM_RATE_HZ;

	// The following section of this function is the PID algorithm.

	// This is another critical section as the desired yaw is updated via an interrupt.
	//  This stops the chance of data sharing problems.
	IntMasterDisable();

	yawError = desiredYaw - yaw;

	IntMasterEnable();

	yawErrorIntegrated += yawError * 0.001;

	yawErrorDerivative = (yawError-previousYawError)/0.001;

	duty4 = (Kp * yawError) + (Ki * yawErrorIntegrated) + (Kd * yawErrorDerivative);

	previousYawError = yawError;

	// Keeps the duty cycle within its bounds.
	if (running) {
		if (duty4 < 5) {
			duty4 = 5;
		} else if (duty4 > 95) {
			duty4 = 95;
		}
	} else if (!running) {
		duty4 = 0;
		yawErrorIntegrated = 0;
	}

	// Update the PWM width set using updated duty cycle.
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * duty4 /100);
}

//*****************************************************************************
// Display Function:
//	Excluded from the final build but was used during testing to check
//  The current altitude/yaw and the main and tail rotor duty cycles.
//*****************************************************************************
void
displayStatus (long Altitude, long yaw)
{
    char string[30], string2[30], string3[30], string4[30];

    //Display the current altitude
    RIT128x96x4StringDraw ("Altitude:", 5, 5, 15);
    sprintf (string, "%d %%  ", Altitude);
    RIT128x96x4StringDraw (string, 70, 5, 15);

    //Display current Yaw
    RIT128x96x4StringDraw ("Yaw:", 5, 15, 15);
	sprintf (string2, "%d %%  ", yaw);
	RIT128x96x4StringDraw (string2, 70, 15, 15);

	//Display current Main rotor Duty cycle
	RIT128x96x4StringDraw ("Main Duty:", 5, 25, 15);
	sprintf (string3, "%d %%  ", duty1);
	RIT128x96x4StringDraw (string3, 70, 25, 15);

	//Display current tail rotor Duty cycle
	RIT128x96x4StringDraw ("Tail Duty:", 5, 35, 15);
	sprintf (string4, "%d %%  ", duty4);
	RIT128x96x4StringDraw (string4, 70, 35, 15);
}

//*****************************************************************************
// Main Program Function:
//	Initialises pins, clocks, PWM outputs, ADC and circular buffer.
//  Maintains Kernel during runtime.
//*****************************************************************************
int
main(void)
{
	unsigned long sum, *dataPtr;
	signed long meanAltitude;
	int i;

	initClock ();
	initButPins ();
	initPin ();
	initADC ();
	initPWMchan ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);

	IntMasterEnable ();

	while (1)
	{
		// Sum and average the data in the circular buffer and calculater the
		//  mean altitude of the helicopter.
		sum = 0ul;
		for (i = 0, dataPtr = g_inBuffer.data; i < BUF_SIZE; i++, dataPtr++)
			sum = sum + *dataPtr;
		meanAltitude = (long)-((long long)sum/BUF_SIZE - ulInitialValue)/((long)1024/375);

		//displayStatus(meanAltitude, yaw*250/300);

		// Function calls to update both the main and tail rotor controls using current
		//  altitude and yaw.
		updatePWM1(meanAltitude);
		updatePWM4(yaw);
	}
}


