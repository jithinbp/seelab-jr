/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   commands.h
 * Author: Jithin B.P
 * Comments:  Contains definitions for byte headers sent for each function
 * Commands are classified under broad categories such as ADC , TIMING etc .. Therefore , each function requires a 2 byte header [group , function]
 * This file also contains definitions for setting peripherals into various modes { e.g ADC can be in 12-bit mode / 10-bit with 16 samples per reading etc..}
 * Revision history: 
 */

#ifndef COMMANDS_H
#define	COMMANDS_H


#define ACKNOWLEDGE 254
/*----flash memory----*/
#define FLASH 1
#define READ_FLASH   1
#define WRITE_FLASH  2
#define WRITE_BULK_FLASH  3
#define READ_BULK_FLASH  4


/*-----ADC------*/
#define ADC 2
#define CAPTURE_ONE 1
#define CAPTURE_TWO 2
#define CAPTURE_DMASPEED 3
#define CAPTURE_FOUR 4
#define CONFIGURE_TRIGGER 5
#define GET_CAPTURE_STATUS 6
#define GET_CAPTURE_CHANNEL 7
#define SET_PGA_GAIN 8
#define GET_VOLTAGE 9
#define GET_VOLTAGE_SUMMED 10
#define SELECT_PGA_CHANNEL  12
#define CAPTURE_12BIT   13
#define CAPTURE_12BIT_SCAN   14

#define SET_HI_CAPTURE   15
#define SET_LO_CAPTURE   16

#define SET_HI_CAPTURE12   17
#define SET_LO_CAPTURE12   18
//#define CAPTURE_DMASPEED12 19
#define MULTIPOINT_CAPACITANCE 20
#define SET_CAP 21

//#define PULSE_TRAIN 22


/*------I2C-------*/
#define I2C 4
#define I2C_START 1
#define I2C_SEND 2
#define I2C_STOP 3
#define I2C_RESTART 4
#define I2C_READ_END 5
#define I2C_READ_MORE 6
#define I2C_WAIT 7
#define I2C_SEND_BURST 8
#define I2C_CONFIG 9
#define I2C_STATUS 10
#define I2C_READ_BULK 11
#define I2C_WRITE_BULK 12
#define I2C_ENABLE_SMBUS 13
#define I2C_INIT 14
#define PULLDOWN_SCL 15
#define I2C_DISABLE_SMBUS 16
#define I2C_START_SCOPE 17


/*-----------DAC--------*/
#define DAC 6
#define SET_DAC 1


/*--------WAVEGEN-----*/
#define WAVEGEN 7
#define SET_WG 1
#define SET_SQR1  3

#define SET_SINE1 13

#define LOAD_WAVEFORM1 15

/*-----digital outputs----*/
#define DOUT 8
#define SET_STATE 1

/*-----digital inputs-----*/
#define DIN   9
#define GET_STATE  1

/*------TIMING FUNCTIONS-----*/
#define TIMING 10
#define GET_TIMING 1

#define TIMING_MEASUREMENTS 12
#define INTERVAL_MEASUREMENTS 13

/*--------MISCELLANEOUS------*/
#define COMMON 11

#define GET_CTMU_VOLTAGE 1
#define GET_CAPACITANCE 2
#define GET_FREQUENCY   3

#define GET_VERSION 5

#define RETRIEVE_BUFFER     8
#define GET_HIGH_FREQUENCY  9
#define CLEAR_BUFFER 10

#define READ_PROGRAM_ADDRESS 12
#define WRITE_PROGRAM_ADDRESS 13
#define READ_DATA_ADDRESS 14
#define WRITE_DATA_ADDRESS 15

#define GET_CAP_RANGE 16
#define READ_LOG 18

#define RESTORE_STANDALONE 19

#define GET_ALTERNATE_HIGH_FREQUENCY  20

#define START_CTMU 23
#define STOP_CTMU 24
#define START_COUNTING 25
#define FETCH_COUNT 26

#define FILL_BUFFER 27
#define HCSR04 28

/*---------- BAUDRATE for main comm channel----*/
#define SETBAUD				12
#define BAUD9600			1
#define BAUD14400			2
#define BAUD19200			3
#define BAUD28800			4
#define BAUD38400			5
#define BAUD57600			6
#define BAUD115200			7
#define BAUD230400			8
#define BAUD1000000			9
#define BAUD2000000			10
#define BAUD4000000			11


#define BUFFER_SIZE 10000
#define NOT_READY 0

/*---------ADC definitions---------*/
#define ADC_10BIT_SIMULTANEOUS 1
#define ADC_10BIT_SEQUENTIAL 2
#define ADC_12BIT 3
#define ADC_CTMU  4
#define ADC_12BIT_AVERAGING 5
#define ADC_12BIT_SCOPE 6
#define ADC_10BIT_DMA 7
#define ADC_12BIT_DMA 8


/*------------TIMER 5 modes---------*/
#define TIMER5_ADC  1
#define TIMER5_FC   2
#define TIMER5_LA   3

/*------INPUT CAPTURE---------*/
//capture modes
#define EVERY_SIXTEENTH_RISING_EDGE 0b101
#define EVERY_FOURTH_RISING_EDGE    0b100
#define EVERY_RISING_EDGE           0b011
#define EVERY_FALLING_EDGE          0b010
#define EVERY_EDGE                  0b001

#define ID1_REMAP 45
#define OD1_REMAP 42
#define PDC1_REMAP 43
#define PDC2_REMAP 44
#define COMP4_REMAP 4
#define RP41_REMAP 41
#define FREQ_REMAP 46 // RPI 46 ,RB14


/*-------ACKNOWLEDGE BYTES-----*/
#define DO_NOT_BOTHER 0
#define SUCCESS 1
#define ARGUMENT_ERROR 2
#define FAILED  3


/*---------DMA_MODES--------*/
#define DMA_LA_ONE_CHAN 1
#define DMA_LA_TWO_CHAN 2
#define DMA_LA_FOUR_CHAN 3




#endif	/* COMMANDS_H */

