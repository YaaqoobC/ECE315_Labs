/*
 *	Lab 2: Part 2: UART in Polling Mode
 *
 *	ECE 315		: Computer Interfacing
 *  Created on	: July 27, 2021
 *  Author	: Shyama M. Gandhi, Mazen Elbaz
 *  Modified by : Antonio Andara
 *  Modified on	: January, 2026
 *
 *     ------------------------------------------------------------------------------------------------------------------------------
 *
 *     This is the main file that uses "sha256.h" header file.
 *     The final objective is to finish the implementation of a hashing and verification system.
 *
 *     ------------------------------------------------------------------------------------------------------------------------------
 */
 // Include FreeRTOS Libraries
#include <projdefs.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sha256.h"

#include "xuartps.h"
#include "xparameters.h"

#include <portmacro.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <xil_io.h>
#include <xil_printf.h>

// Include xilinx Libraries
#include <xparameters.h>
#include <xgpio.h>
#include <xscugic.h>
#include <xil_exception.h>
#include <sleep.h>
#include <xil_cache.h>

// Other miscellaneous libraries
#include "pmodkypd.h"
#include "rgb_led.h"

// Device ID declarations
#define KYPD_DEVICE_ID   	XPAR_GPIO_KYPD_BASEADDR
/*************************** Enter your code here ****************************/
// TODO: Define the seven-segment display (SSD) base address.
#define SSD_BASE_ADDR       XPAR_GPIO_SSD_BASEADDR
#define PSHBTN_BASE_ADDR    XPAR_XGPIO_0_BASEADDR

/*****************************************************************************/

// keypad key table
#define DEFAULT_KEYTABLE 	"0FED789C456B123A"

#define DELAY 10


// Declaring the devices
PmodKYPD 	KYPDInst;

static XGpio SSDInst;
static XGpio rgbLedInst;
static XGpio pshbtnInst;

// ======================================================
// Configuration
// ======================================================
#define UART_BASEADDR 	XPAR_UART1_BASEADDR
#define RX_QUEUE_LEN     512
#define CMD_QUEUE_LEN     16
#define TX_QUEUE_LEN     512

#define INPUT_TEXT_LEN   256
#define HASH_HEX_LEN      64   // SHA-256 hex chars
#define HASH_LEN          32

#define POLL_DELAY_MS     10
#define POLL_DELAY_RX_MS  10


// GLOBAL VARS FOR PART 2
TickType_t xOnDelay = 7;
TickType_t xOffDelay = 7;
u8 current_key = 'x', previous_key = 'x';

// ======================================================
// Types
// ======================================================
typedef enum {
  CMD_NONE,
  CMD_HASH = '1',
  CMD_VERIFY = '2',
  CMD_LED = '3',
  CMD_SSD = '4'
} command_type_t;

typedef struct {
    command_type_t type;
    char input_text[INPUT_TEXT_LEN];
    char expected_hash[HASH_HEX_LEN + 1];  // used only for VERIFY
} crypto_request_t;


typedef struct {
    command_type_t type;
    char calculated_hash[HASH_HEX_LEN + 1];
    u8 match;   // 0 = false, 1 = true
} crypto_result_t;


// ======================================================
// FreeRTOS objects
// ======================================================
static QueueHandle_t q_rx_byte = NULL;   // uint8_t
static QueueHandle_t q_cmd     = NULL;   // crypto_request_t
static QueueHandle_t q_result  = NULL;   // crypto_result_t
static QueueHandle_t q_tx      = NULL;   // char

// ======================================================
// UART instance
// ======================================================
static XUartPs UartPs;

// ======================================================
// Task prototypes
// ======================================================
static void UART_RX_Task(void *pvParameters);
static void CLI_Task(void *pvParameters);
static void Crypto_Task(void *pvParameters);
static void UART_TX_Task(void *pvParameters);

void InitializeKeypad();
static void vKeypadTask( void *pvParameters );
static void vRgbTask(void* pvParameters);
static void vButtonsTask(void* pvParameters);
static void vDisplayTask(void* pvParameters);


// ======================================================
// Crypto helpers
// ======================================================
void hash_to_string(BYTE *hash, char *hashString);
void sha256_string(const char* input, BYTE output[32]);


// ======================================================
// UART helpers
// ======================================================
uint8_t receive_byte(uint8_t *out_byte);
void receive_string(char *buf, size_t buf_len);
static void uart_init(void);
static int uart_poll_rx(uint8_t *b);
static void uart_tx_byte(uint8_t b);


// ======================================================
// Custom UART functions
// ======================================================
void print_string(const char *str);
void print_new_lines(int count);
void flush_uart(void);

// Queues
TickType_t ReadTimeout = 0;

QueueHandle_t xDisplayQueue;    // Keypad to SSD
QueueHandle_t xRgbLedQueue;     // BTN to RGB

typedef struct {
    u8 previous_key;
    u8 current_key;
} displayPacket;

typedef struct {
    TickType_t xOnDelay;
    TickType_t xOffDelay;
} rgbPacket;

u32 SSD_decode(u8 key_value, u8 cathode);

const char *init_message =
	"A hash function is a mathematical algorithm that takes an input (or \"message\")\n"
	"and returns a fixed-size string of bytes. The output, often called the hash\n"
	"value or hash code, is unique (within reason) to the given input. In this lab,\n"
	"we use the sha256 algorithm to compute the hash of a given string or to verify\n"
	"a signature.\n";


// ======================================================
// main()
// ======================================================

int main(void)
{

    uart_init();

    int status;
	// Initialize keypad
	InitializeKeypad();

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
    XGpio_SetDataDirection(&pshbtnInst, 1, 0x01);

    xTaskCreate(UART_RX_Task,
                    "UART_RX",
                    1024, 
                    NULL, 
                    3, 
                    NULL);

    xTaskCreate(UART_TX_Task, 
                    "UART_TX", 
                    1024, 
                    NULL, 
                    3, 
                    NULL);

    xTaskCreate(CLI_Task, 
                    "CLI",     
                    2048, 
                    init_message, 
                    2, 
                    NULL);

    xTaskCreate(Crypto_Task,  
                    "CRYPTO",  
                    2048, 
                    NULL, 
                    2, 
                    NULL);

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

    xTaskCreate(vDisplayTask,					/* The function that implements the task. */
            "display task", 				/* Text name for the task, provided to assist debugging only. */
            configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
            NULL, 						/* The task parameter is not used, so set to NULL. */
            tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
            NULL);

    xTaskCreate(vButtonsTask,					/* The function that implements the task. */
            "buttons task", 				/* Text name for the task, provided to assist debugging only. */
            configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
            NULL, 						/* The task parameter is not used, so set to NULL. */
            tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
            NULL);

    xDisplayQueue = xQueueCreate(   // receive and send: u8 Keyboard Input ie. 1,2,3....A, B..
            1, 
            sizeof(displayPacket));

    xRgbLedQueue = xQueueCreate(
            1, 
            sizeof(rgbPacket)); 
  
    q_rx_byte = xQueueCreate(RX_QUEUE_LEN, sizeof(uint8_t));
    q_cmd     = xQueueCreate(CMD_QUEUE_LEN, sizeof(crypto_request_t));
    q_result  = xQueueCreate(CMD_QUEUE_LEN, sizeof(crypto_result_t)); 
    q_tx  = xQueueCreate(TX_QUEUE_LEN, sizeof(char)); 
  
    configASSERT(xDisplayQueue);
    configASSERT(xRgbLedQueue);
    configASSERT(vKeypadTask);
    configASSERT(vRgbTask);
    configASSERT(vDisplayTask);
    configASSERT(vButtonsTask);

    configASSERT(UART_RX_Task);
    configASSERT(UART_TX_Task);
    configASSERT(CLI_Task);
    configASSERT(Crypto_Task);
    configASSERT(q_rx_byte);
    configASSERT(q_cmd);
    configASSERT(q_result);
    configASSERT(q_tx);

    print_new_lines(50);
    print_string("Initialization complete\nSTARTING APP\n");

    vTaskStartScheduler();

    while (1) {}
}

// ======================================================
// UART RX Task
// ======================================================

static void UART_RX_Task(void *pvParameters)
{

  uint8_t byte;

  for (;;){
    if (uart_poll_rx(&byte)){
      xQueueSend(q_rx_byte, &byte, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_RX_MS));
  }
}

// ======================================================
// UART TX Task
// ======================================================

static void UART_TX_Task(void *pvParameters)
{

  char c;

  for (;;){
    if (xQueueReceive(q_tx, &c, 0) == pdTRUE){
      uart_tx_byte((uint8_t)c);
    }
    vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
  }
}


// ======================================================
// CLI Task
// ======================================================

static void CLI_Task(void *pvParameters)
{
    command_type_t op = CMD_NONE;
    crypto_request_t req;
    crypto_result_t  res;
    char led_input;
    char ssd_input;


    uint8_t dummy;

    print_string((const char *)pvParameters);

    for (;;){
        print_string("\n*******************************************\n");
        print_string("Menu:\n1. Hash a string\n2. Verify hash of a given string\n3. Change LED Brightness\n4. Input SSD value\n");
        print_string("\nEnter your option: ");

        receive_byte((uint8_t *)&op);

        print_string("\n*******************************************\n");

        switch (op){
            case CMD_HASH:
                req.type = CMD_HASH;
                print_string("\nEnter string to calculate hash: ");
                receive_string(req.input_text, sizeof(req.input_text));
                xQueueSend(q_cmd, &req, 0);
                /* Polling queue for result */
                while (xQueueReceive(q_result, &res, 0) != pdTRUE){
                    vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
                }
                print_string("\nCalculated hash: ");
                print_string(res.calculated_hash);
                print_string("\n");
                break;

            case CMD_VERIFY:
                req.type = CMD_VERIFY;
                print_string("\nEnter string to verify: ");
                receive_string(req.input_text, sizeof(req.input_text));

                print_string("\nEnter the precomputed hash: ");
                receive_string(req.expected_hash, sizeof(req.expected_hash));

                xQueueSend(q_cmd, &req, 0);

                /* Wait (polling) for result */
                while (xQueueReceive(q_result, &res, 0) != pdTRUE){
                    vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
                }

                print_string("\nCalculated hash: ");
                print_string(res.calculated_hash);
                print_string("\nExpected hash: ");
                print_string(req.expected_hash);

                if (res.match){
                    print_string("\nHashes are the same!\n");
				} else {
                    print_string("\nHashes are different\n");
				}
                break;
            
            case CMD_LED:
                while (1) {
                    print_string("\nEnter 8 to increase brightness or 1 to dim or 0 to exit LED: ");
                    receive_string(&led_input, 2);
                    int updated = 0;
                    rgbPacket packet;
                    if (led_input == '1') {
                        if (!(xOnDelay < 1)) {
                            xOnDelay -= 1;
                            xOffDelay += 1;
                        }
                        
                        // if (xOnDelay == 0) continue;
                        xil_printf("\nOffDelay=%u, OnDelay=%u\n", xOffDelay, xOnDelay);
                        vTaskDelay(50UL);
                        updated = 1;
                        

                    } else if (led_input == '8') {
                        if (!(xOffDelay < 1)) {
                            xOnDelay += 1;
                            xOffDelay -= 1;
                        }
                        xil_printf("\nOffDelay=%u, OnDelay=%u\n", xOffDelay, xOnDelay);
                        vTaskDelay(50UL);
                        updated = 1;
                    } else if (led_input == '0') {
                        break;
                    }

                    if (updated) {
                        packet.xOffDelay = xOffDelay;
                        packet.xOnDelay = xOnDelay;
                        xQueueOverwrite(xRgbLedQueue, &packet);
                        updated = 0;
                        vTaskDelay(10UL);
                    }
                }
            
                break;
            case CMD_SSD:

                while (1) {
                    displayPacket send_buff = {current_key, previous_key};  
                    
                    print_string("\nEnter character to print to the SSD (G to exit): ");
                    receive_string(&ssd_input, 2);

                    if (ssd_input == 'G')
                        break;

                    previous_key = current_key;
                    current_key = ssd_input;

                    send_buff.previous_key = previous_key;
                    send_buff.current_key = current_key;
                    
                    xQueueOverwrite(xDisplayQueue, &send_buff);

                }
                break;

            default:
                print_string("\nOption not recognized\n");
                break;
        }
		
        vTaskDelay(pdMS_TO_TICKS(1000));
        print_string("\nPress any key to continue.");
        receive_byte(&dummy);
        vTaskDelay(pdMS_TO_TICKS(1000));
        flush_uart();
        print_new_lines(5);
    }
}


// ======================================================
// Crypto Task
// ======================================================

static void Crypto_Task(void *pvParameters)
{
    crypto_request_t req;
    crypto_result_t  res;

    unsigned char hash[HASH_LEN];

    for (;;){
        if (xQueueReceive(q_cmd, &req, 0) == pdTRUE){
            sha256_string(req.input_text, hash);
            hash_to_string(hash, res.calculated_hash);

            res.type = req.type;
            res.match = 0;

            if (req.type == CMD_VERIFY){
                if (strcmp(res.calculated_hash, req.expected_hash) == 0){
                    res.match = 1;
                }
            }

            xQueueSend(q_result, &res, 0);
        }
		
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
}


uint8_t receive_byte(uint8_t *out_byte)
{
    while(1){
        if (xQueueReceive(q_rx_byte, out_byte, 0)!=pdTRUE){            
            vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
        } else {
            return *out_byte;
        }
    }
}


void receive_string(char *buf, size_t buf_len)
{
    uint8_t recvd;
    size_t idx = 0;
    buf[0] = '\0';

    while (1){
		if (xQueueReceive(q_rx_byte, &recvd, 0) == pdTRUE) {
            if (recvd != '\r' && idx < buf_len)
                buf[idx++] = recvd;
            else if (idx >= buf_len)
                continue;
            else {
                buf[idx] = '\0'; 
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
}


void flush_uart(void)
{
    uint8_t dummy;    
    while (xQueueReceive(q_rx_byte, &dummy, 0) == pdTRUE);
}


void print_string(const char *str)
{   
    for (int i = 0; i < strlen(str); i++) {
        xQueueSend(q_tx, &str[i], 0);
    }
}

void print_new_lines(int count)
{
    for (int i = 0; i < count; i++){
        print_string("\n");
    }
}


void hash_to_string(BYTE *hash, char *hash_string)
{
    for (int i = 0; i < HASH_LEN; i++){
        sprintf(&hash_string[i * 2], "%02X", hash[i]);
    }

    hash_string[HASH_LEN * 2] = '\0'; // Null terminate the string
}

void sha256_string(const char* input, BYTE output[32])
{
    SHA256_CTX ctx;
    sha256Init(&ctx);
    sha256Update(&ctx, (BYTE*)input, strlen(input));
    sha256Final(&ctx, output);
}


static void uart_init(void)
{
  XUartPs_Config *cfg;

  cfg = XUartPs_LookupConfig(UART_BASEADDR);
  if (!cfg){
    while (1) {}
  }

  if (XUartPs_CfgInitialize(&UartPs, cfg, cfg->BaseAddress) != XST_SUCCESS){
    while (1) {}
  }

  XUartPs_SetBaudRate(&UartPs, 115200);
}


static int uart_poll_rx(uint8_t *b)
{
  if (XUartPs_IsReceiveData(UartPs.Config.BaseAddress)){
    *b = XUartPs_ReadReg(UartPs.Config.BaseAddress, XUARTPS_FIFO_OFFSET);
    return 1;
  }
  return 0;
}

static void uart_tx_byte(uint8_t b)
{
  while (XUartPs_IsTransmitFull(UartPs.Config.BaseAddress)){

  }
  
  XUartPs_WriteReg(UartPs.Config.BaseAddress, XUARTPS_FIFO_OFFSET, b);
}

static void vKeypadTask( void *pvParameters )
{
	u16 keystate;
	XStatus status, previous_status = KYPD_NO_KEY;
	u8 new_key = 'x';
    displayPacket send_buff = {current_key, previous_key};

/*************************** Enter your code here ****************************/
	// TODO: Define a constant of type TickType_t named 'xDelay' and initialize
	//       it with a value of 100.

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

            send_buff.previous_key = previous_key;
            send_buff.current_key = current_key;
            
            xQueueOverwrite(xDisplayQueue, &send_buff);
            vTaskDelay(pdMS_TO_TICKS(DELAY));
/*****************************************************************************/
		} else if (status == KYPD_MULTI_KEY && status != previous_status){
			xil_printf("Error: Multiple keys pressed\r\n");
		}
		
		previous_status = status;
        vTaskDelay(pdMS_TO_TICKS(DELAY));
	}
}

static void vRgbTask(void *pvParameters)
{
    const uint8_t color = RGB_CYAN;
    rgbPacket recvd;
    TickType_t xOnDelay = 7;
    TickType_t xOffDelay = 7;

    while (1){
        if (xQueueReceive(xRgbLedQueue, &recvd, ReadTimeout) == pdTRUE) {
            xOnDelay = recvd.xOnDelay;
            xOffDelay = recvd.xOffDelay;
        }
        
        if (xOnDelay != 0) {
            XGpio_DiscreteWrite(&rgbLedInst, RGB_CHANNEL, color);
            vTaskDelay(xOnDelay);
        }

        XGpio_DiscreteWrite(&rgbLedInst, RGB_CHANNEL, 0);
        vTaskDelay(xOffDelay);
    }
}

static void vButtonsTask(void *pvParameters) {
	TickType_t xPeriod = 14;
    TickType_t xDelay = xPeriod / 2;
    rgbPacket packet;

    while (1) {

        u32 val = XGpio_DiscreteRead(&pshbtnInst, 1);

        vTaskDelay(xDelay);
        int updated = 0;
        
        if (val == 8) {
            if (!(xOnDelay < 1)) {
                xOnDelay -= 1;
                xOffDelay += 1;
            }
            // if (xOnDelay == 0) continue;
            xil_printf("\nOffDelay=%u, OnDelay=%u\n", xOffDelay, xOnDelay);
            vTaskDelay(50UL);
            updated = 1;
            

        } else if (val == 1) {
            if (!(xOffDelay < 1)) {
                xOnDelay += 1;
                xOffDelay -= 1;
            }
            xil_printf("\nOffDelay=%u, OnDelay=%u\n", xOffDelay, xOnDelay);
            vTaskDelay(50UL);
            updated = 1;
        }

        if (updated) {
            packet.xOffDelay = xOffDelay;
            packet.xOnDelay = xOnDelay;
            xQueueOverwrite(xRgbLedQueue, &packet);
            updated = 0;
            vTaskDelay(10UL);
        }
        
    }
}

static void vDisplayTask(void *pvParameters) {

    displayPacket recvd_packet;
    const TickType_t xDelay = pdMS_TO_TICKS(DELAY);
    u32 ssd_value=0;
    u8 current_key = 'x', previous_key = 'x';

    while(1) {
        if (xQueueReceive(xDisplayQueue, &recvd_packet, ReadTimeout) == pdTRUE) {
            current_key = recvd_packet.current_key;
            previous_key = recvd_packet.previous_key;
        }

        ssd_value = SSD_decode(previous_key, 0);
        XGpio_DiscreteWrite(&SSDInst, 1, ssd_value);
        
        vTaskDelay(xDelay);

        ssd_value = SSD_decode(current_key, 1);
        XGpio_DiscreteWrite(&SSDInst, 1, ssd_value);
        vTaskDelay(xDelay);
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
