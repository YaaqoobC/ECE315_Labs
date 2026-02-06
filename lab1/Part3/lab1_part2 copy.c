/*
 * Lab 1, Part 2 - Seven-Segment Display & Keypad
 *
 * ECE-315 WINTER 2025 - COMPUTER INTERFACING
 * Created on: February 5, 2021
 * Modified on: July 26, 2023
 * Modified on: January 20, 2025
 * Author(s):  Shyama Gandhi, Antonio Andara Lara, Rowan Rasmusson, Yaaqoob Choulli
 *
 * Summary:
 * 1) Declare & initialize the 7-seg display (SSD).
 * 2) Use xDelay to alternate between two digits fast enough to prevent flicker.
 * 3) Output pressed keypad digits on both SSD digits: current_key on right, previous_key on left.
 * 4) Print status changes and experiment with xDelay to find minimum flicker-free frequency.
 *
 * Deliverables:
 * - Demonstrate correct display of current and previous keys with no flicker.
 * - Print to the SDK terminal every time that theh variable `status` changes.
 */
 


// Include FreeRTOS Libraries
#include <FreeRTOS.h>
#include <portmacro.h>
#include <task.h>
#include <queue.h>

// Include xilinx Libraries
#include <xparameters.h>
#include <xgpio.h>
#include <xscugic.h>
#include <xil_exception.h>
#include <xil_printf.h>
#include <sleep.h>
#include <xil_cache.h>

// Other miscellaneous libraries
#include "pmodkypd.h"
#include "rgb_led.h"

#define DELAY     10UL

// Device ID declarations
#define KYPD_DEVICE_ID   	XPAR_GPIO_KYPD_BASEADDR
/*************************** Enter your code here ****************************/
// TODO: Define the seven-segment display (SSD) base address.
#define SSD_BASE_ADDR       XPAR_GPIO_SSD_BASEADDR
#define PSHBTN_BASE_ADDR    XPAR_XGPIO_0_BASEADDR

/*****************************************************************************/

// keypad key table
#define DEFAULT_KEYTABLE 	"0FED789C456B123A"

// Declaring the devices
PmodKYPD 	KYPDInst;

/*************************** Enter your code here ****************************/
// TODO: Declare the seven-segment display peripheral here.
static XGpio SSDInst;
static XGpio rgbLedInst;
static XGpio pshbtnInst;

/*****************************************************************************/

// Function prototypes
void InitializeKeypad();
static void vKeypadTask( void *pvParameters );
static void vRgbTask(void* pvParameters);
u32 SSD_decode(u8 key_value, u8 cathode);


int main(void)
{
	int status;
	// Initialize keypad
	InitializeKeypad();

/*************************** Enter your code here ****************************/
	// TODO: Initialize SSD and set the GPIO direction to output.
    status = XGpio_Initialize(&SSDInst, SSD_BASE_ADDR);
    if (status != XST_SUCCESS)
    {
        xil_printf("GPIO Initialization for SSD unsuccessful.\r\n");
        return XST_FAILURE;
    }

    status = XGpio_Initialize(&rgbLedInst, RGB_LED_BASEADDR);
    if (status != XST_SUCCESS)
    {
        xil_printf("GPIO Initialization for RGBLed unsuccessful.\r\n");
        return XST_FAILURE;
    }

    status = XGpio_Initialize(&pshbtnInst, PSHBTN_BASE_ADDR);
    if (status != XST_SUCCESS)
    {
        xil_printf("GPIO Initialization for psh unsuccessful.\r\n");
        return XST_FAILURE;
    }

    XGpio_SetDataDirection(&SSDInst, 1, 0x00);
    XGpio_SetDataDirection(&rgbLedInst, 2, 0x00);
    // THIS MAY BE WRONG
    XGpio_SetDataDirection(&pshbtnInst, 1, 0x01);
/*****************************************************************************/

	xil_printf("Initialization Complete, System Ready!\n");

	xTaskCreate(vKeypadTask,					/* The function that implements the task. */
				"main task", 				/* Text name for the task, provided to assist debugging only. */
				configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
				NULL);

    xTaskCreate(vRgbTask,					/* The function that implements the task. */
            "rgb task", 				/* Text name for the task, provided to assist debugging only. */
            configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
            NULL, 						/* The task parameter is not used, so set to NULL. */
            tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
            NULL);


	vTaskStartScheduler();
	while(1);
	return 0;
}


static void vKeypadTask( void *pvParameters )
{
	u16 keystate;
	XStatus status, previous_status = KYPD_NO_KEY;
	u8 new_key, current_key = 'x', previous_key = 'x';
	u32 ssd_value=0;

/*************************** Enter your code here ****************************/
	// TODO: Define a constant of type TickType_t named 'xDelay' and initialize
	//       it with a value of 100.
    const TickType_t xDelay = pdMS_TO_TICKS(DELAY);

/*****************************************************************************/

    xil_printf("Pmod KYPD app started. Press any key on the Keypad.\r\n");
	while (1){
		// Capture state of the keypad
		keystate = KYPD_getKeyStates(&KYPDInst);

		// Determine which single key is pressed, if any
		// if a key is pressed, store the value of the new key in new_key
		status = KYPD_getKeyPressed(&KYPDInst, keystate, &new_key);
		// Print key detect if a new key is pressed or if status has changed
		if (status == KYPD_SINGLE_KEY && previous_status == KYPD_NO_KEY){
			xil_printf("Key Pressed: %c\r\n", (char) new_key);
/*************************** Enter your code here ****************************/
			// TODO: update value of previous_key and current_key
            previous_key = current_key;
            current_key = new_key;
            
/*****************************************************************************/
		} else if (status == KYPD_MULTI_KEY && status != previous_status){
			xil_printf("Error: Multiple keys pressed\r\n");
		}
		
/*************************** Enter your code here ****************************/
		// TODO: display the value of `status` each time it changes

/*****************************************************************************/
		previous_status = status;

/*************************** Enter your code here ****************************/
		/* TODO: Decode the current and previous keys using the `SSD_decode` function.
		* Write each decoded value to the seven-segment display, one at a time,
		* using the `XGpio_DiscreteWrite` function.
		* Add a delay between updates for persistence of vision using `vTaskDelay`.
		*/

        ssd_value = SSD_decode(previous_key, 0);
        XGpio_DiscreteWrite(&SSDInst, 1, ssd_value);
        
        vTaskDelay(xDelay);

        ssd_value = SSD_decode(current_key, 1);
        XGpio_DiscreteWrite(&SSDInst, 1, ssd_value);

        vTaskDelay(xDelay);


/*****************************************************************************/
	}
}

static void vRgbTask(void *pvParameters)
{
    const uint8_t color = RGB_CYAN;
	TickType_t xPeriod = 14;
    TickType_t xDelay = xPeriod / 2;

    TickType_t xOnDelay = 7;
    TickType_t xOffDelay = 7;

    while (1){

        u32 val = XGpio_DiscreteRead(&pshbtnInst, 1);

        vTaskDelay(xDelay);
        
        if (val == 8) {
            if (!(xOnDelay < 1)) {
                xOnDelay -= 1;
                xOffDelay += 1;
            }
            xil_printf("\nOffDelay=%u, OnDelay=%u\n", xOffDelay, xOnDelay);
            vTaskDelay(50UL);
        } else if (val == 1) {
            if (!(xOffDelay < 1)) {
                xOnDelay += 1;
                xOffDelay -= 1;
            }
            xil_printf("\nOffDelay=%u, OnDelay=%u\n", xOffDelay, xOnDelay);
            vTaskDelay(50UL);
        }
        
        XGpio_DiscreteWrite(&rgbLedInst, RGB_CHANNEL, color);
        vTaskDelay(xOnDelay);
        XGpio_DiscreteWrite(&rgbLedInst, RGB_CHANNEL, 0);
        vTaskDelay(xOffDelay);
    }
}


void InitializeKeypad()
{
	KYPD_begin(&KYPDInst, KYPD_DEVICE_ID);
	KYPD_loadKeyTable(&KYPDInst, (u8*) DEFAULT_KEYTABLE);
}

// This function is hard coded to translate key value codes to their binary representation
u32 SSD_decode(u8 key_value, u8 cathode)
{
    u32 result;

	// key_value represents the code of the pressed key
	switch(key_value){ // Handles the coding of the 7-seg display
		case 48: result = 0b00111111; break; // 0
        case 49: result = 0b00110000; break; // 1
        case 50: result = 0b01011011; break; // 2
        case 51: result = 0b01111001; break; // 3
        case 52: result = 0b01110100; break; // 4
        case 53: result = 0b01101101; break; // 5
        case 54: result = 0b01101111; break; // 6
        case 55: result = 0b00111000; break; // 7
        case 56: result = 0b01111111; break; // 8
        case 57: result = 0b01111100; break; // 9
        case 65: result = 0b01111110; break; // A
        case 66: result = 0b01100111; break; // B
        case 67: result = 0b00001111; break; // C
        case 68: result = 0b01110011; break; // D
        case 69: result = 0b01001111; break; // E
        case 70: result = 0b01001110; break; // F
        default: result = 0b00000000; break; // default case - all seven segments are OFF
    }

	// cathode handles which display is active (left or right)
	// by setting the MSB to 1 or 0
    if(cathode==0){
            return result;
    } else {
            return result | 0b10000000;
	}
}
