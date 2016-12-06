/* 
 * File:   functions.h
 * Author: jithin
 *
 * Created on October 17, 2014, 12:33 PM
 */

#ifndef FUNCTIONS_H
#define	FUNCTIONS_H

/*
 * File:   functions.h
 * Author: jithin
 *
 * Created on October 17, 2014, 11:48 AM
 */

#include <p24EP256MC204.h>
#include<libpic30.h>
#include"commands.h"

#define FP 64000000
#define BAUDRATE 1000000    //1M
#define BAUDRATE2 10000    //10K
//If BRGH=0 (16 clocks per BAUD), replace '/4' with '/16'
#define BRGVAL ((FP/BAUDRATE)/4)-1
#define BRGVAL9600 ((FP/9600)/4)-1
#define BRGVAL14400 ((FP/14400)/4)-1
#define BRGVAL19200 ((FP/19200)/4)-1
#define BRGVAL28800 ((FP/28800)/4)-1
#define BRGVAL38400 ((FP/38400)/4)-1
#define BRGVAL57600 ((FP/57600)/4)-1
#define BRGVAL115200 ((FP/115200)/4)-1
#define BRGVAL230400 ((FP/230400)/4)-1
#define BRGVAL460800 ((FP/460800)/4)-1
#define BRGVAL500000 ((FP/500000)/4)-1
#define BRGVAL1000000 ((FP/1000000)/4)-1
#define BRGVAL2000000 ((FP/2000000)/4)-1
#define BRGVAL4000000 ((FP/4000000)/4)-1
#define BRGVAL2 ((FP/BAUDRATE2)/16)-1



/*
 * File:   globals.c
 * Author: jithin
 *
 * Created on 2 October, 2016, 10:09 AM
 */


#include "xc.h"
#include"functions.h"

#define Fs   		4000
#define SAMPPRD		(FP/Fs)-1
#define BYTE unsigned char

typedef BYTE bool;
typedef unsigned int uint16;
#define true 1
#define false 0

#define ERROR_BUFFLEN 1500

/*-----PPS and PIN DEFINITIONS-----*/
#define CSCH1 _LATA10
#define CSCH2 _LATA7
#define SCL_PIN _LATB4
#define SDA_PIN _RA8

#define LEDPIN _LATB5   //status LED
#define OD1_PPS RPOR5bits.RP54R
#define OD1_PIN _LATC6
#define CAP_CHARGE_PIN _TRISC0

#define CSNUM_A1 1
#define CSNUM_A2 2


#define DAC_WAVELENGTH 4095

#define TRUE 1
#define FALSE 0

#define DELAY_105uS asm volatile ("REPEAT, #6721"); Nop(); // 105uS delay



/*------------Sine Table--------------*/

#define WAVE_TABLE_FULL_LENGTH 512
#define WAVE_TABLE_SHORT_LENGTH 32

#define HIGH_RES_WAVE 512
#define LOW_RES_WAVE  64




/*Global Variables*/


extern __prog__ unsigned int __attribute__((section("CALIBS"), space(prog), aligned(_FLASH_PAGE * 2))) dat1[15][_FLASH_PAGE];

//__eds__ unsigned int ADCbuffer[BUFFER_SIZE] __attribute__((space(eds))); 
extern int __attribute__((section("adcbuff"),far)) ADCbuffer[BUFFER_SIZE];
extern int *buff0,*buff1,*endbuff,*buff2,*buff3;
extern int  *buffpointer, *endpointer, dma_channel_length,samples,I2CSamples;
extern BYTE *bytebuff1,*bytebuff2,ca,cb;

extern unsigned int dest[_FLASH_ROW * 8];
extern int ulsb,umsb; //DAC_OFFSETS[4],

/*-----SPI VARIABLES-------*/
extern BYTE location, value,ADC_MODE ,DMA_MODE;
extern BYTE SPI_PPRE,SPI_SPRE,SPI_CKE,SPI_CKP,SPI_SMP;

/*------UART VARIABLES-----*/
extern unsigned int I2C_BRGVAL,TCD ;

/*------LOGIC ANALYZER VARIABLES-----*/
extern BYTE INITIAL_DIGITAL_STATES,INITIAL_DIGITAL_STATES_ERR,DIGITAL_TRIGGER_CHANNEL,DIGITAL_TRIGGER_STATE,b1,b2,COMPARATOR_CONFIG,conversion_done ,I2CConvDone;
extern unsigned int i, lsb, msb, blk[8], c1, c2,adval,tmp_int1,tmp_int2,tmp_int3,tmp_int4,tmp_int5,tmp_int6;

extern unsigned int LAFinished , LASamples;
extern unsigned int samples_to_fetch , I2CTotalSamples;
extern unsigned long val,l1,l2;
extern BYTE DIN_REMAPS[] ,LAM1,LAM2,LAM3,LAM4;

/*-----OSCILLOSCOPE VARIABLES-------*/

extern BYTE ADC_CHANNELS ,CH123SA,CHOSA; // CH1 only
extern BYTE TRIGGER_CHANNEL,TRIGGERED,TRIGGER_READY,SH,ICG,I2C_TRIGGER_CHANNEL,I2C_TRIGGERED,I2C_TRIGGER_READY, I2C_SCOPE_LOCATION,I2C_SCOPE_ADDRESS,I2C_SCOPE_BYTES;
extern unsigned int TRIGGER_TIMEOUT,TRIGGER_WAITING,TRIGGER_LEVEL,TRIGGER_PRESCALER;
extern unsigned int ADC_DELAY ;

extern BYTE frequency_scaling,frequency_ready;
extern unsigned int freq_lsb,freq_msb,freq2_lsb,freq2_msb;
extern _prog_addressT p,pProg;

/*--------Stepper Motor--------*/
extern BYTE motor_phases[],current_motor_phase ;


/*--------Error handling definitions------*/
extern char errors[ERROR_BUFFLEN],tmpstr[25];
extern char *error_readpos,*error_writepos;


extern int  __attribute__((section("sine_table1"))) sineTable1[];

extern int  __attribute__((section("sine_table1_short"))) sineTable1_short[] ;




void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void) ;
void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void); //For frequency counter

void set_cap_voltage(BYTE v,unsigned int time);
unsigned int get_cc_capacitance(BYTE,BYTE,unsigned int);
unsigned int get_cap_range(unsigned int);
unsigned int get_ctmu_voltage(BYTE,BYTE,BYTE);
void get_high_frequency(BYTE,BYTE);
void init_IC_for_frequency(BYTE capture_pin,BYTE capture_mode,BYTE captures_per_interrupt);
void TimingMeasurements(BYTE,BYTE,BYTE,BYTE,BYTE,BYTE);
void Interval(BYTE,BYTE,BYTE,BYTE);
void EnableComparator();
void DisableComparator();
void disableCTMUSource(void);

void start_4chan_LA(unsigned int,unsigned int,BYTE);
void disable_input_capture();
void start_dma_dds();

void init(void);
void setFlashPointer(BYTE);
void delayTMR4(int);
void PrepareTrigger(void);
void initADC10(void);
void initADCDMA(BYTE);
void initADC12bit_scope(void);
void initADC12(void);
void initADCCTMU(void);
void initADC12_averaging16();

void setADCMode(BYTE,BYTE,BYTE);
unsigned int get_voltage_summed(BYTE channel);
unsigned int get_voltage(BYTE channel);
void setupADC10();
void preciseDelay(int t);
void configureADC();
void disableADCDMA();
void enableADCDMA();


void enableLogicAnalyser(void);
void disableLogicAnalyser(void);



void initUART(unsigned int);
bool hasChar();
void sendChar(BYTE val);
void sendInt(unsigned int val);
void sendLong(unsigned int lsb,unsigned int msb);
char getChar();
unsigned int getInt();
void ack(BYTE);


void configUART2(unsigned int BAUD);
bool hasChar2(void);
char getChar2(void);
unsigned int getInt2(void);
void sendAddress2(char address) ;
void initUART2(void);
void sendChar2(char val);
void sendInt2(unsigned int val);
void initUART2_passthrough(unsigned int);

void set_CS(BYTE channel,BYTE status);
void initSPI();
unsigned int spi_write16(unsigned int value);
void start_spi();
void stop_spi();

void sqr1(unsigned int,unsigned int,BYTE);
void sqr2(unsigned int,unsigned int,BYTE);
void initDAC();

void Delay_us(unsigned int);
void Delay_with_pulse(unsigned int);

void sineWave1(unsigned int wavelength,BYTE highres);
void alternate_get_high_frequency(BYTE channel,BYTE scale);
void get_high_frequency(BYTE channel,BYTE scale);

void setPGA(char, char);
void setSensorChannel(char);

void read_all_from_flash(_prog_addressT pointer);
void load_to_flash(_prog_addressT pointer, BYTE location, unsigned int * blk);
void read_flash(_prog_addressT pointer, BYTE location);

void initI2C(void);


void I2CStart();
void I2CStop();
void I2CRestart();
void I2CAck();
void I2CNak();
void I2CWait();
void I2CSend(BYTE dat);
BYTE I2CRead(BYTE ack);

#endif	/* FUNCTIONS_H */

