// Include FreeRTOS Libraries
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Include xilinx Libraries
#include "xparameters.h"
#include "xgpio.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xil_cache.h"

// Other miscellaneous libraries
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include "pmodkypd.h"
#include "sleep.h"
#include "PmodOLED.h"
#include "OLEDControllerCustom.h"

// --- Hardware Definitions ---
#define BTN_DEVICE_ID    XPAR_GPIO_INPUTS_BASEADDR
#define KYPD_DEVICE_ID   XPAR_GPIO_KEYPAD_BASEADDR
#define KYPD_BASE_ADDR   XPAR_GPIO_KEYPAD_BASEADDR
#define BTN_CHANNEL      1

// RGB LED Definitions (from lab reference)
#define RGB_LED_ADDR     XPAR_GPIO_LEDS_BASEADDR
#define RGB_CHANNEL      2
#define LED_RED          0x4
#define LED_GREEN        0x2
#define LED_BLUE         0x1
#define LED_OFF          0x0

// keypad key table
#define DEFAULT_KEYTABLE "0FED789C456B123A"

// Declaring the devices
XGpio btnInst;
XGpio rgbLedInst; // New: RGB LED instance
PmodOLED oledDevice;
PmodKYPD KYPDInst;

// Function prototypes
void InitializeKeypad();
static void keypadTask( void *pvParameters );
static void gameTask( void *pvParameters );
static void buttonTask( void *pvParameters );
static void rgbTask( void *pvParameters ); // New: RGB monitoring task

// Display configurations
const u8 orientation = 0x0; 
const u8 invert = 0x1;      

// --- Game & Difficulty State ---
typedef enum { EASY, MEDIUM, HARD } Difficulty;
volatile Difficulty current_difficulty = EASY;

// Speeds mapping to difficulty
volatile int base_speed_x = 1; // Easy default
volatile int base_speed_y = 1; // Easy default

// Paddle definitions
int paddle_y = 12;            // Starting Y position of paddle
const int paddle_x = 5;       // Fixed X position of paddle
const int paddle_height = 8;  // Height of paddle

// Ball definitions
int ball_x = 100, ball_y = 16; 
int ball_dx = -1, ball_dy = 1; // Start moving left and down
const int ball_size = 2;       

int score = 0;
int lives = 3;

int main()
{
	int status = 0;
    
	// 1. Initialize Keypad
	InitializeKeypad();

	// 2. Initialize OLED
    OLED_Begin(&oledDevice,
               XPAR_GPIO_OLED_BASEADDR,
               XPAR_SPI_OLED_BASEADDR,
               orientation,
               invert);

	// 3. Initialize Buttons
	status = XGpio_Initialize(&btnInst, BTN_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization failed.\r\n");
		return XST_FAILURE;
	}

    // 4. Initialize RGB LEDs
    status = XGpio_Initialize(&rgbLedInst, RGB_LED_ADDR);
    if(status != XST_SUCCESS){
		xil_printf("RGB Initialization failed.\r\n");
		return XST_FAILURE;
	}
    // Set RGB channel as output
    XGpio_SetDataDirection(&rgbLedInst, RGB_CHANNEL, 0x0);

	xil_printf("Initialization Complete, System Ready!\n");
    xil_printf("Controls: 2 (Up), 8 (Down)\n");
    xil_printf("Difficulty: 4 (Easy), 5 (Medium), 6 (Hard)\n");

    srand(12345);

    // Create Tasks
	xTaskCreate(keypadTask, "keypad task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(gameTask,   "game task",   configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(buttonTask, "button task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(rgbTask,    "rgb task",    configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // Start Scheduler
	vTaskStartScheduler();

    while(1);
    return 0;
}

void InitializeKeypad()
{
   KYPD_begin(&KYPDInst, KYPD_BASE_ADDR);
   KYPD_loadKeyTable(&KYPDInst, (u8*) DEFAULT_KEYTABLE);
}

// -------------------------------------------------------------------------
// KEYPAD TASK: Paddle Movement & Difficulty Selection
// -------------------------------------------------------------------------
static void keypadTask( void *pvParameters )
{
   u16 keystate;
   XStatus status;
   u8 new_key = 'x';
   const TickType_t xDelay = 25 / portTICK_RATE_MS;

   while (1) {
	  keystate = KYPD_getKeyStates(&KYPDInst);
	  status = KYPD_getKeyPressed(&KYPDInst, keystate, &new_key);

	  if (status == KYPD_SINGLE_KEY) {
          // Paddle controls
          if (new_key == '2') {
              if (paddle_y > 0) paddle_y -= 2; 
          } 
          else if (new_key == '8') {
              if (paddle_y < (OledRowMax - paddle_height)) paddle_y += 2;
          }
          
          // Difficulty controls
          else if (new_key == '4') {
              current_difficulty = EASY;
              base_speed_x = 1;
              base_speed_y = 1;
          }
          else if (new_key == '5') {
              current_difficulty = MEDIUM;
              base_speed_x = 2;
              base_speed_y = 1; // Keep Y a bit slower to make it playable
          }
          else if (new_key == '6') {
              current_difficulty = HARD;
              base_speed_x = 3;
              base_speed_y = 2; 
          }
	  }

	  vTaskDelay(xDelay);
   }
}

// -------------------------------------------------------------------------
// GAME TASK: Physics, collisions, and drawing
// -------------------------------------------------------------------------
static void gameTask( void *pvParameters )
{
	char temp[20];
	OLED_SetDrawMode(&oledDevice, 0);
	OLED_SetCharUpdate(&oledDevice, 0); 

    const TickType_t frameDelay = 50 / portTICK_RATE_MS;

	while(1) {
		if (lives > 0) {
            
            // Apply current difficulty speeds dynamically while preserving direction
            ball_dx = (ball_dx > 0) ? base_speed_x : -base_speed_x;
            ball_dy = (ball_dy > 0) ? base_speed_y : -base_speed_y;

            // --- 1. Update Ball Position ---
            ball_x += ball_dx;
            ball_y += ball_dy;

            // --- 2. Roof and Floor Collisions ---
            if (ball_y <= 0) {
                ball_y = 0;             
                ball_dy = -ball_dy;     // Reverse Y 
            } else if (ball_y >= (OledRowMax - ball_size)) {
                ball_y = OledRowMax - ball_size; 
                ball_dy = -ball_dy;     // Reverse Y 
            }

            // --- 3. Right Wall Collision ---
            if (ball_x >= (OledColMax - ball_size)) {
                ball_x = OledColMax - ball_size;
                ball_dx = -ball_dx;     // Reverse X
            }

            // --- 4. Paddle & Left Wall Collision ---
            if (ball_x <= paddle_x + 1) { 
                // Hit the paddle?
                if ((ball_y + ball_size >= paddle_y) && (ball_y <= paddle_y + paddle_height)) {
                    ball_x = paddle_x + 2;   // Push ball slightly right
                    ball_dx = -ball_dx;      // Bounce back
                    score++;                 
                } 
                // Missed paddle
                else if (ball_x <= 0) {
                    lives--;                 
                    // Reset Ball
                    ball_x = OledColMax - 10;
                    ball_y = rand() % (OledRowMax - ball_size);
                    ball_dx = -base_speed_x; // Serve towards player
                }
            }

            // --- 5. Draw Frame ---
            OLED_ClearBuffer(&oledDevice);

            // Draw Paddle
            OLED_MoveTo(&oledDevice, paddle_x, paddle_y);
            OLED_DrawLineTo(&oledDevice, paddle_x, paddle_y + paddle_height);

            // Draw Ball 
            OLED_MoveTo(&oledDevice, ball_x, ball_y);
            OLED_RectangleTo(&oledDevice, ball_x + (ball_size - 1), ball_y + (ball_size - 1));

            OLED_Update(&oledDevice);

		} else {
            // --- Game Over Screen ---
			OLED_ClearBuffer(&oledDevice);
			
            OLED_SetCursor(&oledDevice, 0, 0);
			OLED_PutString(&oledDevice, "Game Over");
			
            OLED_SetCursor(&oledDevice, 0, 2);
			sprintf(temp, "Score: %d", score);
			OLED_PutString(&oledDevice, temp);
			
            OLED_Update(&oledDevice);
		}

        vTaskDelay(frameDelay);
	}
}

// -------------------------------------------------------------------------
// RGB LED TASK: Updates visual feedback based on difficulty setting
// -------------------------------------------------------------------------
static void rgbTask( void *pvParameters )
{
    u32 led_value = LED_OFF;

    while (1) {
        // Map current difficulty state to colors
        if (current_difficulty == EASY) {
            led_value = LED_GREEN;
        } else if (current_difficulty == MEDIUM) {
            led_value = LED_BLUE;
        } else if (current_difficulty == HARD) {
            led_value = LED_RED;
        }

        // Write to GPIO
        XGpio_DiscreteWrite(&rgbLedInst, RGB_CHANNEL, led_value);
        
        // Only needs to check periodically
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// -------------------------------------------------------------------------
// BUTTON TASK: Handles resetting the game when lives run out
// -------------------------------------------------------------------------
static void buttonTask( void *pvParameters )
{
	u8 buttonVal = 0;
    const TickType_t pollDelay = 100 / portTICK_RATE_MS;

	while(1) {
		buttonVal = XGpio_DiscreteRead(&btnInst, BTN_CHANNEL);
		
        // If any button is pressed while game is over, restart.
        if (buttonVal != 0 && lives == 0) {
			lives = 3;
			score = 0;
            
            // Reset paddle and ball
            paddle_y = 12;
            ball_x = OledColMax - 10;
            ball_y = rand() % (OledRowMax - ball_size);
            ball_dx = -base_speed_x;
            ball_dy = base_speed_y;
		}

		vTaskDelay(pollDelay);
	}
}
