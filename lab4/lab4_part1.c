/*
 *  lab4_main.c
 *  Lab 4: Stepper Motor Main Source file
 *  Created on: Mar 24, 2021
 *  Modified on: Mar 8, 2022
 *  Modified on: Mar 9, 2026
 *  Author: Shyama Gandhi
 *  Modified by: Antonio Andara Lara
 *
 *  Description:
 *  The code has three tasks: _Task_Uart(), _Task_Motor() and _Task_Emerg_Stop().
 *
 *  _Task_Uart() displays the menu on the console and then allows the user to either keep the default values for the motor parameters or change it. This task is also responsible for
 *  sending the final motor parameters via Queue/FIFO to _Task_Motor(). In this task itself, the user can provide a maximum of 10 destination-delay pairs.
 *  For example,
 *  Current Position in Steps = 0
 *  Motor Speed = 500
 *  Motor acceleration = 120
 *  Motor deceleration = 120
 *  Target Position in Steps <target position, dwell time at the target position> = <2048, 1000>, <-2048, 2000>, <4096, 3000> (user enters total of three destination-delay pairs
 *
 *  please note this: <2048, 1000>  : one rotation Clock wise and dwell time of 1000 ms at this target position
 *                    <-2048, 2000> : going to -2048 from the previous 2048 position which is two rotations Counter Clockwise and dwell time of 2000 ms at -2048 target position.
 *                    <4096, 3000>  : going to 4096 from the previous position of -2048 which is three rotations Clockwise and dwell time of 3000 ms at 4096 position.
 *  This is what you should observe on motor if you use the same example as above.
 *
 *
 *  _Task_Motor() is responsible receiving the queue parameters and then setting up the motor parameters by calling the respective functions from the stepper.c file.
 *
 *  _Task_Emerg_Stop() will execute emergency stop when Btn0 is pressed for three consecutive polls. The motor will now begin to decelerate and come to a complete stop.
 *  And a red light on the RGB led will flash continuously at 2 Hz frequency.
 *
 */

#include "sleep.h"
#include "xil_cache.h"
#include "stepper.h"
#include "xuartps.h"        //UART definitions header file
#include "xgpio.h"          //GPIO functions definitions
#include "xparameters.h"    //DEVICE ID, UART BASEADDRESS, GPIO BASE ADDRESS definitions
#include <projdefs.h>


static void _Task_Uart( void *pvParameters );
static TaskHandle_t xUarttask;

static void _Task_Motor( void *pvParameters );
static TaskHandle_t xMotortask;

static void _Task_Emerg_Stop( void *pvParameters );
static TaskHandle_t xEmergStopTask;

int Initialize_UART();

/************************* Queue Function definitions *************************/
static QueueHandle_t xQueue_FIFO1 = NULL;   //queue between task1 and task2

/************************* Global Variables ***********************************/

//GPIO Button Instance and DEVICE ID
XGpio BTNInst;
#define EMERGENCY_BUTTON_BASEADDR       XPAR_GPIO_INPUTS_BASEADDR

//GPIO RGB led Instance and DEVICE ID
XGpio Red_RGBInst;
XGpio LEDInst;
#define RGB_LED_BASEADDR                    XPAR_GPIO_LEDS_BASEADDR

#define PMOD_MOTOR_BASEADDR                 XPAR_STEPPER_MOTOR_BASEADDR

#define PMOD_MOTOR_DEVICE_ID                XPAR_XGPIO_4_BASEADDR


// The number of positions/delays which can be sequenced
#define SEQUENCE_LENGTH 10
#define RGB_GREEN 0x02
#define RGB_RED 0x04

//struct for motor parameters
typedef struct {
    long  currentposition_in_steps;
    float rotational_speed;
    float rotational_acceleration;
    float rotational_deceleration;
} decision_parameters;

decision_parameters motor_parameters;
int parameters_flag = 0;

int positionSequence[SEQUENCE_LENGTH][2] = {{NO_OF_STEPS_PER_REVOLUTION_FULL_DRIVE, 0}}; // position-delay array
int sequenceIndex = 0; // the number of position-delay sequences
int loop_count = 1; // the number of times to repeat the position-delay sequence

//----------------------------------------------------
// MAIN FUNCTION
//----------------------------------------------------
int main (void)
{
    int status;
    //------------------------------------------------------
    // INITIALIZE THE PMOD GPIO PERIPHERAL FOR STEPPER MOTOR, STOP BUTTON AND RGB LED(that will flash the red light when emergency stop button is executed).
    //------------------------------------------------------

    // Initialize the PMOD for motor signals (JC PMOD is being used)
    status = XGpio_Initialize(&PModMotorInst, PMOD_MOTOR_DEVICE_ID);
    if(status != XST_SUCCESS){
    xil_printf("GPIO Initialization for PMOD unsuccessful.\r\n");
    return XST_FAILURE;
    }

    // button for emergency stop activation
    // Initialize the PMOD for getting the button value (btn0 is being used)
    status = XGpio_Initialize(&BTNInst, EMERGENCY_BUTTON_BASEADDR);
    if(status != XST_SUCCESS){
        xil_printf("GPIO Initialization for BUTTONS unsuccessful.\r\n");
        return XST_FAILURE;
    }

    // RGB Led for flashing the red light when stop button is activated
    // Initialize the PMOD for flashing the RED light on RGB LEDz
    status = XGpio_Initialize(&Red_RGBInst, RGB_LED_BASEADDR);
    if(status != XST_SUCCESS){
        xil_printf("GPIO Initialization for BUTTONS unsuccessful.\r\n");
        return XST_FAILURE;
    }

    //Initialize the UART
    status = Initialize_UART();
    if (status != XST_SUCCESS){
        xil_printf("UART Initialization failed\n");
    }


    // Set all buttons direction to inputs
    XGpio_SetDataDirection(&BTNInst, 1, 0xFF);
    // Set the RGB LED direction to output
    XGpio_SetDataDirection(&Red_RGBInst, 1, 0x00);


    // Initialization of motor parameter values here
    motor_parameters.currentposition_in_steps = 0;
    motor_parameters.rotational_speed = 500;
    motor_parameters.rotational_acceleration = 150;
    motor_parameters.rotational_deceleration = 150;

    xil_printf("\nStepper motor Initialization Complete! Operational parameters can be changed below:\n\n");

    xTaskCreate( _Task_Uart,
                ( const char * ) "Uart Task",
                configMINIMAL_STACK_SIZE*10,
                NULL,
                tskIDLE_PRIORITY+1,
                &xUarttask );

    xTaskCreate( _Task_Motor,
                ( const char * ) "Motor Task",
                configMINIMAL_STACK_SIZE*10,
                NULL,
                tskIDLE_PRIORITY+2,
                &xMotortask );

    xTaskCreate( _Task_Emerg_Stop,
                ( const char * ) "Emergency Stop Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY+3,
                &xEmergStopTask );

    //the queue size if set to 25 right now, you can change this size later on based on your requirements.
    xQueue_FIFO1 = xQueueCreate( 25, sizeof(struct decision_parameters*) ); //connects task1 -> task2

    configASSERT(xQueue_FIFO1);

    vTaskStartScheduler();

    while(1);

    return 0;
}

static void _Task_Uart( void *pvParameters ){

    int message_flag = 0;

    while(1){

        // print the message that corresponds to the parameter we want to get. This the menu that gets printed initially!
        if(message_flag == 0){
            if(parameters_flag == 0){
                xil_printf("Current position of the motor = %d steps\n", motor_parameters.currentposition_in_steps);
                xil_printf("Press <ENTER> to keep this value, or type a new starting position and then <ENTER>\n");
            }
            else if(parameters_flag == 1){
                printf("Current maximum speed of the motor = %0.1f steps/sec\n", motor_parameters.rotational_speed);
                xil_printf("Press <ENTER> to keep this value, or type a new maximum speed number and then <ENTER>\n");
            }
            else if(parameters_flag == 2){
                printf("Current maximum acceleration of the motor = %0.1f steps/sec/sec\n", motor_parameters.rotational_acceleration);
                xil_printf("Press <ENTER> to keep this value, or type a new maximum acceleration and then <ENTER>\n");
            }
            else if(parameters_flag == 3){
                printf("Current maximum deceleration of the motor = %0.1f steps/sec/sec\n", motor_parameters.rotational_deceleration);
                xil_printf("Press <ENTER> to keep this value, or type a new maximum deceleration and then <ENTER>\n");
            }
            else if(parameters_flag == 4){
                xil_printf("Destination position of the motor = %d steps\n", positionSequence[sequenceIndex][0]);
                xil_printf("Press <ENTER> to keep this value, or type a new destination position and then <ENTER>\n");
            }
            else if(parameters_flag == 5){
                xil_printf("Current dwell time of the motor between movements = %i ms\n", positionSequence[sequenceIndex][1]);
                xil_printf("Press <ENTER> to keep this value, or type a new dwell time and then <ENTER>\n");
            }
            else if(parameters_flag == 6){
                xil_printf("Position slots available: %i\n", SEQUENCE_LENGTH - sequenceIndex);
                xil_printf("Press <ENTER> to stop entering more positions, or enter a new position and then <ENTER>\n");
            }
        }


        char str_value_motor_value[] = "";
        char read_UART_character[100];  //an approximate size is being taken into consideration. You will use a larger size if you require.
        int  invalid_input_flag=0;

        int keep_default_value_flag = 0;
        int idx=0;
        while (1){
            // get input from the user
            if(XUartPs_IsReceiveData(XPAR_XUARTPS_0_BASEADDR)){
                read_UART_character[idx] = XUartPs_ReadReg(XPAR_XUARTPS_0_BASEADDR, XUARTPS_FIFO_OFFSET);
                idx++;
                // stop getting input when a return is found
                if(read_UART_character[idx-1] == 0x0D){
                    break;
                }
            }
        }

        // if only enter was pressed, keep the default parameter value
        if(idx == 1){
            if(read_UART_character[idx-1] == 0x0D){
                keep_default_value_flag = 1;
                invalid_input_flag = 0;
            }
        }
        // else get the number, ensuring it is positive
        else if(parameters_flag <= 5 && parameters_flag != 4){
            for(int i=0; i<idx-1; i++){
                // if an invalid character is found
                if(!(read_UART_character[i] >= '0' && read_UART_character[i] <= '9')){
                    invalid_input_flag = 1;
                    break;
                }
                // else copy the character
                else{
                    strncat(str_value_motor_value, &read_UART_character[i], 1);
                    invalid_input_flag = 0;
                }
            }
        }
        // parameters 4 and 6 allow for negative numbers, get the number
        else if(parameters_flag == 4 || parameters_flag == 6){
            for(int i=0; i<idx-1; i++){
                // if an invalid character is found
                if(!(read_UART_character[i] >= '0' && read_UART_character[i] <= '9') && read_UART_character[i]!='-'){
                    invalid_input_flag = 1;
                    break;
                }
                // else copy the character
                else{
                    strncat(str_value_motor_value, &read_UART_character[i], 1);
                    invalid_input_flag = 0;
                }
            }
        }

        // notify the user on invalid data entry
        if(invalid_input_flag == 1){
            message_flag = 1;
            xil_printf("There was an invalid input from user except the valid inputs between 0-9\n");
            xil_printf("Please input the value of this parameter again!\n");
        }
        // if we are still getting motor parameters from the user
        else if(parameters_flag <= 6){
            message_flag = 0;
            // update the value with the one given by the user, or keep it as the default
            if(parameters_flag == 0){
                if(keep_default_value_flag == 1){
                    xil_printf("User chooses to keep the default value of current position = %d steps\n\n",motor_parameters.currentposition_in_steps);
                }
                else{
                    motor_parameters.currentposition_in_steps = atoi(str_value_motor_value);
                    xil_printf("User entered the new current position = %d steps\n\n",motor_parameters.currentposition_in_steps);
                }
            }
            // update the value with the one given by the user, or keep it as the default
            else if(parameters_flag == 1){
                if(keep_default_value_flag == 1){
                    printf("User chooses to keep the default value of rotational speed = %0.1f steps/sec\n\n",motor_parameters.rotational_speed);
                }
                else{
                    motor_parameters.rotational_speed = atoi(str_value_motor_value);
                    printf("User entered the new rotational speed = %0.1f steps/sec\n\n",motor_parameters.rotational_speed);
                }
            }
            // update the value with the one given by the user, or keep it as the default
            else if(parameters_flag == 2){
                if(keep_default_value_flag == 1){
                    printf("User chooses to keep the default value of rotational acceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_acceleration);
                }
                else{
                    motor_parameters.rotational_acceleration = atoi(str_value_motor_value);
                    printf("User entered the new rotational acceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_acceleration);
                }
            }
            // update the value with the one given by the user, or keep it as the default
            else if(parameters_flag == 3){
                if(keep_default_value_flag == 1){
                    printf("User chooses to keep the default value of rotational deceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_deceleration);
                }
                else{
                    motor_parameters.rotational_deceleration = atoi(str_value_motor_value);
                    printf("User entered the new rotational deceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_deceleration);
                }
            }
            // update the value with the one given by the user, or keep it as the default
            else if(parameters_flag == 4){
                if(keep_default_value_flag == 1){
                    xil_printf("User chooses to keep the default value of destination position = %d\n\n", positionSequence[sequenceIndex][0]);
                }
                else{
                    positionSequence[sequenceIndex][0] = atoi(str_value_motor_value);
                    xil_printf("User entered the new destination position = %d steps\n\n",positionSequence[sequenceIndex][0]);
                }
            }
            // update the value with the one given by the user, or keep it as the default
            else if(parameters_flag == 5){
                if(keep_default_value_flag == 1){
                    xil_printf("User chooses to keep the default value of dwell time = %d\n", positionSequence[sequenceIndex][1]);
                }
                else{
                    positionSequence[sequenceIndex][1] = atoi(str_value_motor_value);
                    xil_printf("User entered new dwell time = %d ms\n",positionSequence[sequenceIndex][1]);
                }

                xil_printf("*** Pair: %d\t, <destination, delay> = <%d, %d>\n\n", sequenceIndex+1, positionSequence[sequenceIndex][0], positionSequence[sequenceIndex][1]);

                sequenceIndex ++;
            }

            // once we have collected all of the motor parameters
            else if(parameters_flag == 6){
                // show the user the next menu
                if(keep_default_value_flag == 1 || sequenceIndex == SEQUENCE_LENGTH){
                    xil_printf("User chooses to stop inputting sequences, or sequence full.\n\n");
                    xil_printf("\n****************************** MENU ******************************\n");
                    xil_printf("1. Press m<ENTER> to change the motor parameters again.\n");
                    xil_printf("2. Press g<ENTER> to start the movement of the motor.\n");

                    char command_1_or_2_values[100];
                    int index=0;
                    char command;
                    while (1){
                        // get the menu option from the user
                        if(XUartPs_IsReceiveData(XPAR_XUARTPS_0_BASEADDR)){
                            command_1_or_2_values[index] = XUartPs_ReadReg(XPAR_XUARTPS_0_BASEADDR, XUARTPS_FIFO_OFFSET);
                            index++;
                            // if the user enters their input and presses enter
                            if(command_1_or_2_values[index-1] == 0x0D){
                                if((index>2) | (index==1)){
                                    index=0;
                                }
                                // get the user command
                                else if(index == 2){
                                    command = command_1_or_2_values[index-2];
                                    if((command == 'm') | (command == 'L') | (command == 'g')){
                                        break;
                                    }
                                    else{
                                        index = 0;
                                    }
                                }
                            }
                        }
                    }
                    // if the user enters 'm' go back to setting the motor parameters
                    if(command == 'm'){
                        parameters_flag = 0;
                        sequenceIndex = 0;
                    }
                    // start the sequence if the user enters 'g'
                    else if(command == 'g'){
                        loop_count = 1;
                        decision_parameters * const pointer_to_motor_struct_values = &motor_parameters;
                        xQueueSendToBack(xQueue_FIFO1, &pointer_to_motor_struct_values, 0UL);
                    }
                }
                // else the user wishes to input another position in the sequence
                else{
                    positionSequence[sequenceIndex][0] = atoi(str_value_motor_value);
                    xil_printf("User entered the new destination position = %d steps\n\n",positionSequence[sequenceIndex][0]);
                    parameters_flag = 4;
                }
            }
            // increment to get the next parameter
            parameters_flag += 1;
        }

        vTaskDelay(1);
    }
}


/*-----------------------------------------------------------*/
static void _Task_Motor( void *pvParameters ){

    decision_parameters *read_motor_parameters_from_queue;
    Stepper_PMOD_pins_to_output();
    Stepper_Initialize();
    
    while(1){

        /**********************************************************************************************/
        // get the motor parameters from the queue (FIFO1). The structure "decision_parameters" to store the received data has been declared in this task for you.
        xQueueReceive(xQueue_FIFO1, &read_motor_parameters_from_queue, portMAX_DELAY);

        /**********************************************************************************************/



        /**********************************************************************************************/

        xil_printf("\nStarting the Motor Rotation...\n");

        // set the motor parameters by calling the respective stepper functions from the driver files.
        // you need to set the ROTATIONAL SPEED, ROTATIONAL ACCELERATION, ROTATIONAL DECELERATION and CURRENT POSITION IN STEPS
        // once you have done that, loop through the number of destination delay pairs (hint: "for" loop should iterate over "sequenceIndex" variable)
        // For example, the first destination-delay pair has value <2048,1000> which means motor should move to position 2048 and then dwell for 1000 ms at this position.
        // Find the function from the driver code that will help to move the motor by an absolute number of target steps.! The function is mentioned in the handout as well.
        // Once the motor reaches the desired position, disable the motor and then execute the dwell time delay using the conventional vTaskDelay().

        Stepper_setAccelerationInStepsPerSecondPerSecond(read_motor_parameters_from_queue->rotational_acceleration);
        Stepper_setDecelerationInStepsPerSecondPerSecond(read_motor_parameters_from_queue->rotational_deceleration);
        Stepper_setSpeedInStepsPerSecond(read_motor_parameters_from_queue->rotational_speed);
        Stepper_setCurrentPositionInSteps(read_motor_parameters_from_queue->currentposition_in_steps);


        // Loop through each destination-delay pair
        for (int i = 0; i < sequenceIndex; i++) {
            XGpio_DiscreteWrite(&Red_RGBInst, 2, RGB_GREEN);
            Stepper_moveToPositionInSteps(positionSequence[i][0]);
            XGpio_DiscreteWrite(&Red_RGBInst, 2, 0x00);
            Stepper_disableMotor();
            if (positionSequence[i][1] > 0) {
                vTaskDelay(pdMS_TO_TICKS(positionSequence[i][1]));
            }
        }

        motor_parameters.currentposition_in_steps = Stepper_getCurrentPositionInSteps();

        /**********************************************************************************************/

        // print the motor position, and reset variables to start getting the next position-delay sequence
        xil_printf("\n\nCurrent position of the motor = %d steps\n", motor_parameters.currentposition_in_steps);
        xil_printf("Press <ENTER> to keep this value, or type a new starting position and then <ENTER>\n");

        parameters_flag = 0;
        sequenceIndex = 0;

    }
}


static void _Task_Emerg_Stop( void *pvParameters ){
    int pressedCount = 0; // the number of consecutive polls the button is pressed for
    int btnState = 0; // the button state

    while(1){

        /**********************************************************************************************/
        //Read the Button value inside the variable "btnState"
        //i.e., poll the button
        btnState = XGpio_DiscreteRead(&BTNInst, 1);


        /**********************************************************************************************/

        // if the button is pressed, increase the counter. Else reset it to zero.
        if(btnState == 1) pressedCount ++;
        else pressedCount = 0;

        // if the button is pressed for 3 consecutive polls
        if(pressedCount >= 3){

            /**********************************************************************************************/
            //Set the "current stepper position" to the position at which it must now begin decelerating.
            //There is a stepper driver function for adjusting the current position in steps and you have used in the _Task_Motor().
            //Cancel the rest of the destination position-delay pairs.
            //Inside an infinite loop, flash the Red light on RGB led at 2Hz.
            //The Object Instance for RGB led is "Red_RGBInst".
            Stepper_SetupStop();
            memset(&positionSequence, 0, sizeof(positionSequence));
            while(1) {
                Stepper_disableMotor();
                XGpio_DiscreteWrite(&Red_RGBInst, 2, RGB_RED);
                vTaskDelay(pdMS_TO_TICKS(250));
                XGpio_DiscreteWrite(&Red_RGBInst, 2, 0x00);
                vTaskDelay(pdMS_TO_TICKS(250));
            }



            /**********************************************************************************************/
        }
        // wait 10ms (polling loop at 100Hz)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
