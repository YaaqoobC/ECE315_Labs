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

#define BTN_DEVICE_ID  XPAR_GPIO_INPUTS_BASEADDR
#define KYPD_DEVICE_ID XPAR_GPIO_KEYPAD_BASEADDR
#define KYPD_BASE_ADDR XPAR_GPIO_KEYPAD_BASEADDR
#define BTN_CHANNEL    1

// keypad key table
#define DEFAULT_KEYTABLE 	"0FED789C456B123A"

// Declaring the devices
XGpio btnInst;
PmodOLED oledDevice;
PmodKYPD KYPDInst;

// Function prototypes
void InitializeKeypad();
static void keypadTask( void *pvParameters );
static void gameTask( void *pvParameters );
static void buttonTask( void *pvParameters );

// Display configurations
const u8 orientation = 0x0; 
const u8 invert = 0x1;      

// --- Pong Game Variables ---
// Paddle definitions
int paddle_y = 12;            // Starting Y position of paddle (middle of screen)
const int paddle_x = 5;       // Fixed X position of paddle (left side)
const int paddle_height = 8;  // How tall the paddle is

// Ball definitions
int ball_x = 100, ball_y = 16; // Ball starting position
int ball_dx = -2, ball_dy = 1; // Ball velocity (moving left and slightly down)
const int ball_size = 2;       // Make it a 2x2 square so it's easier to see

// Game State
int score = 0;
int lives = 3;

int main()
{
	int status = 0;
    
	// Initialize Keypad
	InitializeKeypad();

	// Initialize OLED
    OLED_Begin(&oledDevice,
               XPAR_GPIO_OLED_BASEADDR,
               XPAR_SPI_OLED_BASEADDR,
               orientation,
               invert);

	// Initialize Buttons
	status = XGpio_Initialize(&btnInst, BTN_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization failed.\r\n");
		return XST_FAILURE;
	}

	xil_printf("Initialization Complete, System Ready!\n");

    // Seed random number generator for ball spawning
    srand(12345);

    // Create Tasks
	xTaskCreate(keypadTask, "keypad task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
	xTaskCreate(gameTask, "game task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
	xTaskCreate(buttonTask, "button task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

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
// KEYPAD TASK: Handles moving the paddle up and down
// -------------------------------------------------------------------------
static void keypadTask( void *pvParameters )
{
   u16 keystate;
   XStatus status, last_status = KYPD_NO_KEY;
   u8 new_key = 'x';

   // Poll keypad every 25ms
   const TickType_t xDelay = 25 / portTICK_RATE_MS;

   while (1) {
	  keystate = KYPD_getKeyStates(&KYPDInst);
	  status = KYPD_getKeyPressed(&KYPDInst, keystate, &new_key);

      // We only care about continuous presses to smoothly move the paddle
	  if (status == KYPD_SINGLE_KEY) {
          if (new_key == '2') {
              // Move paddle UP, clamp to the top edge (Y = 0)
              if (paddle_y > 0) paddle_y -= 2; 
          } 
          else if (new_key == '8') {
              // Move paddle DOWN, clamp to the bottom edge
              if (paddle_y < (OledRowMax - paddle_height)) paddle_y += 2;
          }
	  }

	  last_status = status;
	  vTaskDelay(xDelay);
   }
}

// -------------------------------------------------------------------------
// GAME TASK: Handles physics, collision, and drawing to the OLED
// -------------------------------------------------------------------------
static void gameTask( void *pvParameters )
{
	char temp[20];
	xil_printf("Starting Pong Game Loop\n");
    
	OLED_SetDrawMode(&oledDevice, 0);
	OLED_SetCharUpdate(&oledDevice, 0); // Turn auto-updating off for smooth frames

    // ~20 frames per second
    const TickType_t frameDelay = 50 / portTICK_RATE_MS;

	while(1) {
		if (lives > 0) {
            
            // --- 1. Update Ball Position ---
            ball_x += ball_dx;
            ball_y += ball_dy;

            // --- 2. Roof and Floor Collisions ---
            if (ball_y <= 0) {
                ball_y = 0;              // Push out of bounds
                ball_dy = -ball_dy;      // Reverse Y direction
            } else if (ball_y >= (OledRowMax - ball_size)) {
                ball_y = OledRowMax - ball_size; 
                ball_dy = -ball_dy;      // Reverse Y direction
            }

            // --- 3. Right Wall Collision (Bounce back to player) ---
            if (ball_x >= (OledColMax - ball_size)) {
                ball_x = OledColMax - ball_size;
                ball_dx = -ball_dx;      // Reverse X direction
            }

            // --- 4. Paddle & Left Wall Collision ---
            if (ball_x <= paddle_x + 1) { 
                
                // Did it hit the paddle? (Check Y bounds)
                if ((ball_y + ball_size >= paddle_y) && (ball_y <= paddle_y + paddle_height)) {
                    ball_x = paddle_x + 2;   // Push ball slightly right of paddle
                    ball_dx = -ball_dx;      // Bounce back!
                    score++;                 // Increment score
                } 
                // Did it miss the paddle and hit the left wall?
                else if (ball_x <= 0) {
                    lives--;                 // Lose a life
                    
                    // Reset Ball
                    ball_x = OledColMax - 10;
                    ball_y = rand() % (OledRowMax - ball_size);
                    ball_dx = -2;            // Send back towards the player
                }
            }

            // --- 5. Draw Frame ---
            OLED_ClearBuffer(&oledDevice);

            // Draw Paddle
            OLED_MoveTo(&oledDevice, paddle_x, paddle_y);
            OLED_DrawLineTo(&oledDevice, paddle_x, paddle_y + paddle_height);

            // Draw Ball (as a small rectangle/pixel cluster)
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

        // Wait for next frame
        vTaskDelay(frameDelay);
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
		
        // If button is pressed while the game is over, restart.
        if (buttonVal != 0 && lives == 0) {
			xil_printf("Resetting game...\n");
			lives = 3;
			score = 0;
            
            // Reset paddle and ball
            paddle_y = 12;
            ball_x = OledColMax - 10;
            ball_y = rand() % (OledRowMax - ball_size);
            ball_dx = -2;
            ball_dy = 1;
		}

		vTaskDelay(pollDelay);
	}
}
