/**
 * @file Final_Project_main.c
 *
 * @brief Main source code for the Final Project.
 *
 * @author
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/EUSCI_A2_UART.h"
#include "../inc/EUSCI_A3_SPI.h"
#include "../inc/Nokia5110_LCD.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A1_Interrupt.h"
#include "../inc/Timer_A2_PWM.h"
#include "../inc/Motor.h"

#define QR_DATA_BUFFER_SIZE 1100

void QR_Data_Handler(char bufferString[], int bufferLength);
void Note_Pattern_1(void);
void Drive_Pattern_1(void);
void Buzzer_Init(void);
void Play_Encoded_Pattern(char bufferString[]);

const int BUZZER_DURATION   = 200;
const int NOTE_INTERVAL     = 500;

const int C_NOTE_FREQ       = 1980; // C4 (261 Hz)
const int D_NOTE_FREQ       = 1750; // D4 (293 Hz)
const int E_NOTE_FREQ       = 1555; // E4 (330 Hz)
const int F_NOTE_FREQ       = 1470; // F4 (349 Hz)
const int G_NOTE_FREQ       = 1310; // G4 (392 Hz)
const int A_NOTE_FREQ       = 1165; // A4 (440 Hz)
const int B_NOTE_FREQ       = 1040; // B4 (493 Hz)

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();
    LED2_Init();

    // Initialize the buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Enable the interrupts used by the modules
    EnableInterrupts();

    //MOTOR INIT
    Timer_A2_PWM_Init(60000, 0, 0);
    Motor_Init();

    //BUZZER INIT
    Buzzer_Init();

    //UART FOR QR CODE READER
    EUSCI_A2_UART_Init();

    //SPI INIT
    EUSCI_A3_SPI_Init();

    // Initialize the Nokia 5110 LCD
    Nokia5110_Init();

    // Clear the Nokia 5110 buffer
    Nokia5110_ClearBuffer();

    // Clear the Nokia 5110 LCD
    Nokia5110_Clear();

    Nokia5110_SetCursor(0, 1);

    Nokia5110_Set_Contrast(0xB8);

    char bufferString[QR_DATA_BUFFER_SIZE];

    while(1)
    {
        EUSCI_A2_UART_InString(bufferString, QR_DATA_BUFFER_SIZE);
        for(int i = 0; i < QR_DATA_BUFFER_SIZE; i++){
            printf("DATA: %c\n", bufferString[i]);
        }
        QR_Data_Handler(bufferString, QR_DATA_BUFFER_SIZE);
    }
}

//-------------------HANDLE QR DATA------------------------//
void QR_Data_Handler(char dataBuffer[], int bufferLength){

    if(dataBuffer[0] == 'P'){ //PRE-DEFINED COMMAND
        if(dataBuffer[1] == 'M'){ //MOVEMENT
            if(dataBuffer[2] == '1')
                Drive_Pattern_1();
            else if(dataBuffer[2] == '2'){}
                //Drive_Pattern_2();
            else if(dataBuffer[2] == '3'){}
                //Drive_Pattern_3();
            else if(dataBuffer[2] == '4'){}
                //Drive_Pattern_4();
        }
        else if(dataBuffer[1] == 'S'){ //SOUND
            if(dataBuffer[2] == '1')
                Note_Pattern_1();
            else if(dataBuffer[2] == '2'){}
                //Note_Pattern_2();
            else if(dataBuffer[2] == '3'){}
                //Note_Pattern_3();
            else if(dataBuffer[2] == '4'){}
                //Note_Pattern_4();
        }
    }
    else if(dataBuffer[0] == 'E'){ //ENCODED COMMAND
        if(dataBuffer[1] == 'M'){ //MOVEMENT ENCODING: EM + Direction -> {F,B,L,R}  + Speed -> {1-9} + Duration -> {1-9}
            for(int i = 1; i < (bufferLength-2)/3; i++){
                if(dataBuffer[2*i+i-1] != 'F' && dataBuffer[2*i+i-1] != 'B' && dataBuffer[2*i+i-1] != 'L' && dataBuffer[2*i+i-1] != 'R')
                    break;
                if(dataBuffer[2*i+i-1] == 'F'){
                    Motor_Forward((dataBuffer[3*i]-'0')*1000, (dataBuffer[3*i]-'0')*1000);
                }
                else if(dataBuffer[2*i+i-1] == 'B'){
                    Motor_Backward((dataBuffer[3*i]-'0')*1000, (dataBuffer[3*i]-'0')*1000);
                }
                else if(dataBuffer[2*i+i-1] == 'L'){
                    Motor_Left((dataBuffer[3*i]-'0')*1000, (dataBuffer[3*i]-'0')*1000);
                }
                else if(dataBuffer[2*i+i-1] == 'R'){
                    Motor_Right((dataBuffer[3*i]-'0')*1000, (dataBuffer[3*i]-'0')*1000);
                }
                Clock_Delay1ms((dataBuffer[4+(i-1)*3]-'0')*1000);
                Motor_Stop();
            }
        }
        else if(dataBuffer[1] == 'S'){ //Play Song
            Play_Encoded_Pattern(&dataBuffer[2]);
        }
        else if(dataBuffer[1] == 'I'){//Display Image
            unsigned char Image[504] = {};

            for(int i = 1; i < 1008/2; i++){

                if (dataBuffer[2*i] >= '0' && dataBuffer[2*i] <= '9'){
                    Image[i-1] = (dataBuffer[2*i] - '0') * 16;
                }
                else {
                    Image[i-1] = (dataBuffer[2*i] - 'A' + 10) * 16;
                }
                if (dataBuffer[2*i+1] >= '0' && dataBuffer[2*i+1] <= '9') {
                    Image[i-1] += (dataBuffer[2*i+1] - '0');
                }
                else {
                    Image[i-1] += (dataBuffer[2*i+1] - 'A' + 10);
                }
            }

            Nokia5110_DrawFullImage(&Image[0]);
        }
    }
}

int Buzzer_Output(uint8_t buzzer_value)
{
    P4->OUT = buzzer_value;
    return ((buzzer_value != 0) ? 1 : 0);
}

void Play_Note(int note_delay_value)
{
    int Buzzer_Status = Buzzer_Output(0x00);
    Clock_Delay1us(note_delay_value);
    Buzzer_Status = Buzzer_Output(0x01);
    Clock_Delay1us(note_delay_value);
}

void Note_Pattern_1(void)
{
    int Note_Array[3] = {C_NOTE_FREQ, F_NOTE_FREQ, B_NOTE_FREQ}; //Array of notes to play

    for (int i = 0; i <= 2; i++){
        for (int j = 0; j < BUZZER_DURATION; j++){
            Play_Note(Note_Array[i]); //Play each note in order
        }
    }

}

void Play_Encoded_Pattern(char bufferString[])
{

    for (int i = 0; i < QR_DATA_BUFFER_SIZE ; i++){

        if(bufferString[2*i] == 'X')
            break;

            for (int j = 0; j < ((bufferString[2*i + 1]-'0')*50); j++){
                if(bufferString[2*i] == 'C')
                    Play_Note(1980);
                if(bufferString[2*i] == 'D')
                    Play_Note(1750);
                if(bufferString[2*i] == 'E')
                    Play_Note(1555);
                if(bufferString[2*i] == 'F')
                    Play_Note(1470);
                if(bufferString[2*i] == 'G')
                    Play_Note(1310);
                if(bufferString[2*i] == 'A')
                    Play_Note(1165);
                if(bufferString[2*i] == 'B')
                    Play_Note(1040);
            }
            Clock_Delay1ms(50);
        }

}

void Buzzer_Init(void)
{
    P4->SEL0 &= ~0x01;
    P4->SEL1 &= ~0x01;
    P4->DIR |= 0x01;
}

void Drive_Pattern_1(void)
{
    Motor_Left(1500, 1500);
    Clock_Delay1ms(1000);
    Motor_Left(3000, 3000);
    Clock_Delay1ms(1000);
    Motor_Left(14000, 14000);
    Clock_Delay1ms(3000);
    // Stop the motors
    Motor_Stop();
    Clock_Delay1ms(1000);

}
