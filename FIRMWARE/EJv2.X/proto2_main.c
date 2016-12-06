/*
 * File:   proto2_main.c
 * Author: jithin
 *
 *
 */

// FICD

#pragma config ICS = NONE               // ICD Communication Channel Select bits (Reserved, do not use)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#include<xc.h>
#include <p24EP256MC204.h>
#include <stdlib.h>
#include<libpic30.h>
#include"commands.h"
#include"functions.h"
/*------------------define for standalone version--------------------*/
//#define STANDALONE


#ifdef STANDALONE
const BYTE version[] = "CSpark-SE.P.2.0";
#else
const BYTE version[] = "EJ-2.0";
#endif

//_FICD(ICS_PGD2 & JTAGEN_OFF) //Programmming pins  ..PGED2


/* PLL using external oscillator. */
_FOSCSEL(FNOSC_FRC & IESO_OFF); //Start up using internal oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT ); // enable failsafe osc, use external crystal
//_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_EC ); // For 20MHz fox

// FWDT
#pragma config WDTPOST = PS512          // Watchdog Timer Postscaler bits (1:512)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable bit (Watchdog timer always enabled)

#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)

int main() {
    LEDPIN=0;
    RCONbits.SWDTEN=1;
    unsigned char main_command,sub_command,RESPONSE;
    unsigned int i = 0, n = 0;
    unsigned int ADLOC;
    pProg=0x0;
    unsigned int *pData;
    init();
    initUART(BRGVAL500000);
    
    LEDPIN=1;
    while (1) {
        while (!hasChar()){asm("CLRWDT");}
        main_command = getChar();
        sub_command = getChar();
        RESPONSE = SUCCESS;
        LEDPIN=0;
        switch (main_command) {
            case FLASH:
                switch(sub_command){
                    case WRITE_FLASH:
                        value = getChar(); //fetch the page[0-19]
                        location = getChar(); //fetch the address(0-31)
                        /*-----------fetch 16 characters----------*/
                        for (i = 0; i < 8; i++) {    blk[i] = getInt();       } //two per loop
                        setFlashPointer(value);
                        load_to_flash(p, location, &blk[0]);
                        break;

                    case READ_FLASH:
                        value = getChar(); //fetch the page[0-19]
                        location = getChar();
                        setFlashPointer(value);
                        read_flash(p, location);
                        for (i = 0; i < 8; i++) {
                            sendInt(blk[i]);
                            }
                        break;
                    
                    case READ_BULK_FLASH:
                        lsb = getInt();
                        location = getChar();
                        setFlashPointer(location);
                        read_all_from_flash(p);
                        for (i = 0; i < lsb/2; i++) {
                            sendInt(dest[i]);
                        }
                        break;

                    case WRITE_BULK_FLASH:
                        lsb = getInt();
                        location = getChar();
                        for (i = 0; i < lsb/2; i += 1)  {
                            dest[i]=getInt();
                        }
                        setFlashPointer(location);
                        _erase_flash(p); /* erase a page */
                        for (i = 0; i < _FLASH_ROW * 4; i += 1) /*combine two ints each for each word32 write*/ {
                            tmp_int1 = dest[2 * i];
                            tmp_int2 = dest[2 * i + 1];
                            pProg = p + (4 * i);
                            _write_flash_word32(pProg, tmp_int1, tmp_int2); Nop(); Nop();
                        }
                        
                        break;
                }
                break;

            case ADC:
                switch(sub_command){
                    case CAPTURE_12BIT:
                        disable_input_capture();
                        value = getChar();  //channel number
                        samples_to_fetch = getInt();
                        ADC_DELAY = getInt();
                        ADC_CHANNELS = 0; //capture one channel
                        disableADCDMA();
                        setADCMode(ADC_12BIT_SCOPE,value&0x7F,0);

                        if(value&0x80)PrepareTrigger(); //bit 6 of value.
                        else TRIGGERED=true;
                        conversion_done = 0;samples=0;
                        buff0 = &ADCbuffer[0];
                        endbuff = &ADCbuffer[samples_to_fetch];
                        setupADC10();
                        _AD1IF = 0;   _AD1IE = 1;
                        LEDPIN=0;//set_RGB(0x060606);
                        break;

                    case CAPTURE_12BIT_SCAN:
                        disable_input_capture();
                        ADC_CHANNELS = getChar(); //total channels
                        lsb = getInt();
                        samples_to_fetch = getInt();
                        ADC_DELAY = getInt();
                        disableADCDMA();
                        setADCMode(ADC_12BIT_SCOPE,0,0);
                        TRIGGERED=true;
                        conversion_done = 0;samples=0;
                        buff0 = &ADCbuffer[0];
                        
                        buff0 = &ADCbuffer[0];buff1 = &ADCbuffer[samples_to_fetch];buff2 = &ADCbuffer[2*samples_to_fetch];buff3 = &ADCbuffer[3*samples_to_fetch];  //assume 4 channels. 

                        endbuff = &ADCbuffer[samples_to_fetch];
                        AD1CON1bits.ADON = 0;
                        AD1CON2bits.CSCNA = 1;  //Enable Scanning
                        AD1CON2bits.SMPI = (lsb>>12)&0xF;
                        AD1CSSL=lsb&0xFFF; //scan certain channels
                        AD1CON1bits.ADON = 1;Delay_us(20);

                        setupADC10();
                        _AD1IF = 0;   _AD1IE = 1;
                        LEDPIN=0;//set_RGB(0x060606);
                        break;
                        
                        
                    case CAPTURE_ONE:
                        //disable_input_capture();
                        value = getChar();  //channel number
                        samples_to_fetch = getInt();
                        ADC_DELAY = getInt();

                        ADC_CHANNELS = 0; //capture one channel
                        AD1CON2bits.CHPS = 0;
                        setADCMode(ADC_10BIT_SIMULTANEOUS,value&0x7F,0);
                        AD1CON2bits.CHPS = 0;

                        if(value&0x80)PrepareTrigger(); //bit 6 of value.
                        else TRIGGERED=true;
                        conversion_done = 0;samples=0;
                        buff0 = &ADCbuffer[0];
                        endbuff = &ADCbuffer[samples_to_fetch];
                        setupADC10();
                        _AD1IF = 0;   _AD1IE = 1;
                        LEDPIN=0;//set_RGB_now(0x000A06);
                        break;

                    case CAPTURE_TWO:
                        value = getChar();  //channel number
                        samples_to_fetch = getInt();
                        ADC_DELAY = getInt();
                        ADC_CHANNELS = 1; //capture two channels
                        AD1CON2bits.CHPS = 1;
                        setADCMode(ADC_10BIT_SIMULTANEOUS,value&0x7F,0);
                        AD1CON2bits.CHPS = 1;
                        buff0 = &ADCbuffer[0];buff1 = &ADCbuffer[samples_to_fetch];
                        endbuff = &ADCbuffer[samples_to_fetch];
                        if(value&0x80)PrepareTrigger(); //bit 6 of value.
                        else TRIGGERED=true;
                        conversion_done = 0;samples=0;
                        setupADC10();
                        _AD1IF = 0;   _AD1IE = 1;
                        LEDPIN=0;//set_RGB_now(0x0A0000);
                        break;

                    case CAPTURE_FOUR:
                        value = getChar();  //channel number for SH0
                        samples_to_fetch = getInt();
                        ADC_DELAY = getInt();
                        ADC_CHANNELS = 3;   //capture all four channels
                        AD1CON2bits.CHPS = ADC_CHANNELS;
                        setADCMode(ADC_10BIT_SIMULTANEOUS,(value&0xF),(value>>4)&0x1);
                        buff0 = &ADCbuffer[0];buff1 = &ADCbuffer[samples_to_fetch];
                        buff2 = &ADCbuffer[2*samples_to_fetch];buff3 = &ADCbuffer[3*samples_to_fetch];
                        endbuff = &ADCbuffer[samples_to_fetch];
                        if(value&0x80)PrepareTrigger(); //bit 8 of value.
                        else TRIGGERED=true;
                        conversion_done = 0;samples=0;
                        setupADC10();
                        _AD1IF = 0;   _AD1IE = 1;
                        LEDPIN=0;//set_RGB_now(0x060006);
                        break;

                    case SET_HI_CAPTURE:
                    case SET_LO_CAPTURE:
                        OD1_PPS = 0; //OD1 mappings disabled
                        //fall through
                    case MULTIPOINT_CAPACITANCE:
                    case CAPTURE_DMASPEED:
                        //disable_input_capture();
                        value = getChar();  //channel number
                        
                        samples_to_fetch = getInt();
                        ADC_DELAY = getInt();
                        if(ADC_DELAY<8)ADC_DELAY = 8;                           //maximum capacity is 1MSPS
                        ADC_CHANNELS = 0;                                       //capture one channel
                        AD1CON2bits.CHPS = 0;
                        _AD1IF = 0;   _AD1IE = 0;                               // Do not enable ADC interrupt. We're using DMA.
                        conversion_done = 1;samples=samples_to_fetch;           //assume it's all over already
                        if(value&0x80){setADCMode(ADC_12BIT_DMA,value&0x7F,0); } 
                        else {setADCMode(ADC_10BIT_DMA,value&0x7F,0); }

                        DMA0STAH = __builtin_dmapage (&ADCbuffer[0]);
                        DMA0STAL = __builtin_dmaoffset (&ADCbuffer[0]);
                        DMA0PAD = (int)&ADC1BUF0;                               // Address of the capture buffer register
                        DMA0CNT = samples_to_fetch-1;                           // Number of words to buffer


                        _DMA0IF = 0; _DMA0IE = 1; // Enable DMA interrupt enable bit
                        DMA0CONbits.CHEN = 1;
                        if(sub_command==SET_LO_CAPTURE)OD1_PIN=0; 
                        else if(sub_command==SET_HI_CAPTURE)OD1_PIN=1; 
                        else if(sub_command==MULTIPOINT_CAPACITANCE){ _TRISC0=0; _LATC0=0;}    //Prepare 20K impedance voltage source, and connect it to 0V.


                        setupADC10();
                        LEDPIN=0;
                        break;

                        
                    case SET_CAP:
                        value = getChar();
                        lsb = getInt();         //Delay uS
                        set_cap_voltage(value,lsb);
                        break;

                    case CONFIGURE_TRIGGER:
                        value = getChar();
                        TRIGGER_CHANNEL=value&0x0F;
                        TRIGGER_LEVEL=getInt();
                        TRIGGER_TIMEOUT=50000;
                        TRIGGER_PRESCALER = (value>>4)&0xF;
                        break;

                    case GET_CAPTURE_STATUS:
                        sendChar(conversion_done);
                        sendInt(samples);
                        break;

                    case SET_PGA_GAIN:
                        location = getChar();
                        value = getChar();
                        setPGA(location, value);
                        break;


                    case GET_VOLTAGE:
                        location = getChar();
                        setADCMode(ADC_12BIT,3,0);
                        i=get_voltage(location);
                        sendInt(i);
                        break;

                    case GET_VOLTAGE_SUMMED:
                        location = getChar();
                        i=get_voltage_summed(location);
                        sendInt(i);
                        //sendInt(ADC1BUF0);sendInt(ADC1BUF1);sendInt(ADC1BUF2);sendInt(ADC1BUF3);sendInt(ADC1BUF4);sendInt(ADC1BUF5);sendInt(ADC1BUF6);sendInt(ADC1BUF7);
                        //sendInt(ADC1BUF8);sendInt(ADC1BUF9);sendInt(ADC1BUFA);sendInt(ADC1BUFB);sendInt(ADC1BUFC);sendInt(ADC1BUFD);sendInt(ADC1BUFE);sendInt(ADC1BUFF);
                        break;


                    case GET_CAPTURE_CHANNEL:
                        //disable_input_capture();
                        _LATC0=0;_TRISC0=1; //set 20K CAP voltage source to high impedance mode.

                        value = getChar();      //channel number
                        lsb = getInt();   //number of bytes
                        msb = getInt();           //offset / starting position
                        for (i = msb; i < msb+lsb; i++) sendInt(ADCbuffer[i+samples_to_fetch*value]);
                        break;


                }
                break;
            case I2C:
                switch(sub_command){
                    case I2C_START:             //Initialize I2C and select device address
                        I2CStart();
                        location = getChar(); //=address<<1 | R/W   [r=1,w=0]
                        I2CSend(location);
                        RESPONSE|=(I2C2STATbits.ACKSTAT<<4)|(I2C2STATbits.BCL<<5);
                        break;

                    case I2C_STOP:
                        I2CStop();              // send stop condition as transfer finishes
                        break;

                    case I2C_WAIT:
                        I2CWait();              // send stop condition as transfer finishes
                        break;

                    case I2C_SEND:
                        value = getChar();
                        I2CSend(value);
                        RESPONSE|=(I2C2STATbits.ACKSTAT<<4)|(I2C2STATbits.BCL<<5);
                        break;

                    case I2C_SEND_BURST:
                        value = getChar();
                        I2CSend(value);
                        RESPONSE=DO_NOT_BOTHER;
                        break;

                    case I2C_RESTART:
                        I2CRestart();
                        location = getChar(); //=address<<1 | R/W   [r=1,w=0]
                        I2CSend(location);
                        RESPONSE|=(I2C2STATbits.ACKSTAT<<4)|(I2C2STATbits.BCL<<5);
                        break;

                    case I2C_READ_MORE:
                        location = I2CRead(1);
                        sendChar(location);
                        break;
                    case I2C_READ_END:
                        location = I2CRead(0);
                        sendChar(location);
                        break;

                    case I2C_CONFIG:
                        I2C_BRGVAL = getInt();
                        initI2C();
                        break;
                        
                    case I2C_STATUS:
                        sendInt(I2C2STAT);
                        break;

                    case I2C_READ_BULK:
                        location=getChar();
                        ca=getChar();
                        value=getChar();
                        I2CStart();
                        I2CSend(location<<1); //Address of I2C slave. write.
                        I2CSend(ca); //I2C slave Memory Address to read from
                        I2CRestart();
                        I2CSend((location<<1)|1); //Address , read
                        for(i=0;i<value-1;i++)sendChar(I2CRead(1));
                        sendChar(I2CRead(0));
                        I2CStop();
                        break;

                    case I2C_WRITE_BULK:
                        location=getChar();
                        value=getChar();
                        I2CStart();
                        I2CSend(location<<1);
                        for(i=0;i<value;i++)I2CSend(getChar());
                        I2CStop();
                        break;

                    case I2C_ENABLE_SMBUS:
                        I2C2STAT = 0x0000;
                        I2C2CONbits.SMEN=1;
                        I2C2CONbits.I2CEN = 1;
                        break;

                    case I2C_DISABLE_SMBUS:
                        I2C2STAT = 0x0000;
                        I2C2CONbits.SMEN=0;
                        I2C2CONbits.I2CEN = 1;
                        break;

                    case I2C_INIT:
                        initI2C();
                        break;

                    case PULLDOWN_SCL:
                        lsb=getInt();
                        _TRISB4 = 0;
                        _LATB4=0;
                        Delay_us(lsb);
                        _LATB4=1;
                        _TRISB4 = 1;
                        break;


                    case I2C_START_SCOPE:
                        I2C_SCOPE_ADDRESS=getChar();
                        I2C_SCOPE_LOCATION=getChar();
                        I2C_SCOPE_BYTES=getChar();
                        I2CTotalSamples=getInt();
                        I2CSamples=0; I2CConvDone=0;
                        bytebuff1 = &ADCbuffer[0];
                        _T2IF = 0;
                        _T2IE = 0;
                        T2CONbits.TON = 0;T2CONbits.T32 = 0;
                        T2CONbits.TSIDL = 1;
                        T2CONbits.TCKPS = 2;
                        PR2 = getInt();
                        TMR2 = 0x0000;
                        LEDPIN=0;//set_RGB_now(0x0A0A00);
                        T2CONbits.TON = 1;
                        _T2IF = 0;
                        _T2IE = 1;

                        break;


                }
                break;
            case DAC:
                switch(sub_command){

                case SET_DAC:
                    lsb = getInt();
                    if(lsb&0x8000)OC2R = lsb&0xFFF;
                    else OC3R = lsb&0xFFF;
                    break;
                }
                break;
            case WAVEGEN:
                switch(sub_command){
                    case SET_SINE1:
                        value = getChar();
                        lsb = getInt();
                        sineWave1(lsb,value);
                        break;

                        
                    case LOAD_WAVEFORM1:
                        for(lsb=0;lsb<WAVE_TABLE_FULL_LENGTH;lsb++)sineTable1[lsb]=getInt();
                        for(lsb=0;lsb<WAVE_TABLE_SHORT_LENGTH;lsb++)sineTable1_short[lsb]=getChar();                            
                        break;

                    case SET_SQR1:  //on OD1
                        lsb = getInt(); //wavelength
                        msb = getInt(); //high time
                        value = getChar(); //prescaler
                        if(value&0x4){sqr2(lsb,msb,value&0x3);}
                        else{sqr1(lsb,msb,value&0x3);}
                        break;


                    
                }
                break;
            case DOUT:
                switch(sub_command){
                    case SET_STATE:
                        lsb = getChar();
                        if(lsb&0x10){     OD1_PPS = 0;  OD1_PIN  = lsb&0x1;       } //OD1
                        if(lsb&0x20){     _TRISB14=(lsb>>1)&0x1;   }                // current source
                        if(lsb&0x40){                                               //square 1
                            RPOR3bits.RP41R = 0 ; //disconnect square wave
                            _LATB9 = (lsb>>2)&0x1;                            
                        }
                        if(lsb&0x80){                                              //OD2 / SQ2
                            RPOR2bits.RP38R = 0 ; //disconnect square wave
                            _LATB6 = (lsb>>3)&0x1;                            
                        }
                        break;

                }
                break;
            case DIN:
                switch(sub_command){
                    case GET_STATE:
                        value = getChar();
                        if(value==1)sendInt(PORTA);
                        else if(value==2)sendInt(PORTB);
                        else if(value==3)sendInt(PORTC);
                        RESPONSE = DO_NOT_BOTHER;
                        break;
                }
                break;
            case TIMING:
                switch(sub_command){
                    case GET_TIMING:        //Using input capture
                        _IC1IF=0;
                        lsb = getInt(); //timeout. [t(s)*64e6>>16]
                        value = getChar();
                        location = getChar();
                        init_IC_for_frequency(location&0xF,(value>>2)&0x7,value&0x3);
                        while((!_IC1IF) && (IC2TMR<lsb))asm("CLRWDT"); _IC1IF=0;
                        sendInt(IC2TMR);
                        for(n=0;n<(value&0x3);n++){
                        sendInt(IC1BUF);sendInt(IC2BUF); //read from FIFO
                        }
                        disable_input_capture();
                    break;


                    case TIMING_MEASUREMENTS:        //Using two input captures
                        lsb = getInt(); //timeout. [t(s)*64e6>>16]
                        location = getChar();
                        value = getChar();
                        cb = getChar();
                        TimingMeasurements(location&0xF,(location>>4)&0xF,value&0x7,(value>>3)&0x7,cb&0xF,(cb>>4)&0xF);
                        if((value>>6)&1){RPOR5bits.RP54R = 0;_LATC6=(value>>7)&1;}
                        while((!_IC1IF || !_IC3IF) && (IC2TMR<lsb))asm("CLRWDT");;
                        for(n=0;n<((cb)&0xF);n++){sendInt(IC1BUF);sendInt(IC2BUF);}   //send data from PIN 1
                        for(n=0;n<((cb>>4)&0xF);n++){sendInt(IC3BUF);sendInt(IC4BUF);}//send data from PIN 2
                        _IC3IF=0;_IC1IF=0;
                        sendInt(IC2TMR);
                        disable_input_capture();
                        T2CONbits.TON = 0;
                    break;


                    case INTERVAL_MEASUREMENTS:        //Using two input captures
                        lsb = getInt(); //timeout. [t(s)*64e6>>16]
                        location = getChar();
                        value = getChar();
                        Interval(location&0xF,(location>>4)&0xF,value&0x7,(value>>3)&0x7);
                        while((!_IC1IF) && (IC2TMR<lsb))asm("CLRWDT");
                        sendInt(IC1BUF);sendInt(IC2BUF);
                        while((!_IC3IF) && (IC2TMR<lsb))asm("CLRWDT");
                        sendInt(IC3BUF);sendInt(IC4BUF);
                        sendInt(IC2TMR);
                        disable_input_capture();
                    break;



                }
                break;
            case COMMON:
                switch(sub_command){
                    case GET_CTMU_VOLTAGE:
                        value=getChar();                                        //bits<0-4> = channel,bits <5-6> current range
                        msb = get_ctmu_voltage(value&0x1F,(value>>5)&0x3,(value>>7)&0x1);
                        //sendInt(ADC1BUF0);sendInt(ADC1BUF1);sendInt(ADC1BUF2);sendInt(ADC1BUF3);sendInt(ADC1BUF4);sendInt(ADC1BUF5);sendInt(ADC1BUF6);sendInt(ADC1BUF7);
                        //sendInt(ADC1BUF8);sendInt(ADC1BUF9);sendInt(ADC1BUFA);sendInt(ADC1BUFB);sendInt(ADC1BUFC);sendInt(ADC1BUFD);sendInt(ADC1BUFE);sendInt(ADC1BUFF);
                        sendInt(msb);
                    break;

                    case GET_CAP_RANGE:
                        msb=getInt();               //Charge time.  microseconds
                        sendInt(get_cap_range(msb));
                    break;

                    case START_CTMU:
                        value=getChar();    //bits<0-6> =  current range, bit 7=TGEN
                        location=getChar();    // current trim
                        CTMUCON1bits.CTMUEN = 0;
                        CTMUCON1bits.TGEN = (value>>7)&0x1;                     //(channel==5)?1:0;
                        CTMUICONbits.ITRIM = location;
                        CTMUICONbits.IRNG = (value)&0x7F;                       // 01->Base Range .53uA, 10->base*10, 11->base*100, 00->base*1000
                        CTMUCON1bits.CTTRIG = 0; //do not trigger the ADC
                        CTMUCON1bits.CTMUEN = 1;Delay_us(1000);
                        CTMUCON2bits.EDG1STAT = 1;  // Start current source
                        break;

                    case STOP_CTMU:
                        disableCTMUSource();
                        break;
                    
                    
                    case GET_CAPACITANCE:
                        value=getChar();            //current range for CTMU
                        location=getChar();         //current trimming bits
                        msb=getInt();               //Charge time.  microseconds
                        LEDPIN=0;
                        sendInt(get_cc_capacitance(value,location,msb));
                        LEDPIN=1;
                    break;


                    case GET_HIGH_FREQUENCY:        //This one shares TIMER5 with the ADC!
                        LEDPIN=0;
                        value=getChar();
                        get_high_frequency(value&0xF,(value>>4)&0x4); //0=> 1:1 scaling
                        while(!_T5IF);_T5IF=0;
                        LEDPIN=1;
                        freq_lsb=TMR2;
                        freq_msb=TMR3HLD;
                        sendChar(1);  //scaling factor
                        sendLong(freq_lsb,freq_msb);
                        break;

                    case GET_ALTERNATE_HIGH_FREQUENCY:        //This one shares TIMER5 with the ADC and uses IC timers
                        LEDPIN=0;
                        value=getChar();
                        alternate_get_high_frequency(value&0xF,(value>>4)&0x4); //0=> 1:1 scaling
                        while(!_T5IF);_T5IF=0;
                        LEDPIN=1;
                        freq2_lsb=IC1TMR;
                        freq2_msb=IC2TMR;
                        sendChar(1); //scaling factor
                        sendLong(freq2_lsb,freq2_msb);
                        break;

                    case GET_FREQUENCY:        //Using input capture
                        _IC1IF=0;
                        lsb = getInt(); //timeout. [t(s)*64e6>>16]
                        value = getChar();
                        init_IC_for_frequency(value,EVERY_SIXTEENTH_RISING_EDGE,2);
                        while((IC2TMR<lsb) && (!_IC1IF)); _IC1IF=0; RPINR7bits.IC1R =0; //disconnect
                        if((IC2TMR>=lsb) || (IC2CON1bits.ICOV))sendChar(1); //in case of buffer overflow/timeout
                        else sendChar(0);
                        sendInt(IC1BUF);sendInt(IC2BUF);
                        sendInt(IC1BUF);sendInt(IC2BUF);
                        disable_input_capture();
                        break;

                    case GET_VERSION:
                        for (i = 0; i < sizeof(version)-1; i++) sendChar(version[i]);
                        sendChar('\n');
                        RESPONSE = DO_NOT_BOTHER;
                        break;

                    case RETRIEVE_BUFFER:
                        lsb = getInt();   //starting point
                        msb = getInt();   //number of bytes
                        for (i = lsb; i < msb+lsb; i++) sendInt(ADCbuffer[i]);
                        LEDPIN=1;
                        break;
                    case CLEAR_BUFFER:
                        lsb = getInt();   //starting point
                        msb = getInt();   //number of bytes
                        for (i = lsb; i < msb+lsb; i++) ADCbuffer[i]=0;
                        break;

                    case FILL_BUFFER:
                        lsb = getInt();   //starting point
                        msb = getInt();   //number of bytes
                        for (i = lsb; i < msb+lsb; i++) ADCbuffer[i]=getInt();
                        break;




                    case READ_PROGRAM_ADDRESS:
                        pProg=0x0;
                        l1=getInt()&0xFFFF;
                        l2=getInt()&0xFFFF;
                        _memcpy_p2d16(&ADLOC, pProg+(l1|(l2<<16)),sizeof(unsigned int));
                        sendInt(ADLOC);
                        break;


                    case WRITE_PROGRAM_ADDRESS:
                        pProg=0x0;
                        l1=getInt()&0xFFFF;
                        l2=getInt()&0xFFFF;
                        ADLOC=getInt();
                        //__builtin_tbladdress(p,(l1|(l2<<16)) );  //initialize flash pointer
                        //load_to_flash(p, location, &blk[0]);
                        //_memcpy_p2d16(p,ADLOC ,sizeof(unsigned int));
                        break;

                    case READ_DATA_ADDRESS:
                        lsb=getInt()&0xFFFF;
                        pData=lsb;
                        sendInt(*pData);
                        break;

                    case WRITE_DATA_ADDRESS:
                        msb=getInt();
                        lsb=getInt();
                        pData=msb;
                        *pData=lsb;
                        break;

                    case READ_LOG:
                        while(error_readpos!=error_writepos){
                            sendChar(*error_readpos++);
                            if(error_readpos==&errors[ERROR_BUFFLEN])error_readpos=&errors[0];
                        }
                        sendChar('\n');
                        break;

                    case RESTORE_STANDALONE:
                        asm("goto 0x0000");

                        break;

                    case HCSR04:            //distance sensor
                        RPOR5bits.RP54R = 0;
                        _DMA2IF = 0;    _DMA2IE = 0; _DMA3IF = 0;    _DMA3IE = 0;
                        DMA2CONbits.CHEN = 0;DMA3CONbits.CHEN = 0;

                        lsb = getInt(); //timeout. [t(s)*64e6>>16]
                        _LATC6 = 1;  //SQR1  high
                        Interval(0,0,3,2);
                        Delay_us(10);
                        _LATC6 =0;   //SQR1 low
                        //msb=50000;
                        while((!_IC1IF) && (IC2TMR<lsb));// &&msb){Delay_us(1);msb-=1;}
                        sendInt(IC1BUF);sendInt(IC2BUF);
                        while((!_IC3IF) && (IC4TMR<lsb));// &&msb){Delay_us(1);msb-=1;}
                        sendInt(IC3BUF);sendInt(IC4BUF);
                        sendInt(IC2TMR);
                        disable_input_capture();
                        break;



                }
                break;

  

            case SETBAUD:
                    initUART(sub_command);
                    break;

         }
        if(RESPONSE)ack(RESPONSE);
        LEDPIN=1;


    }


    return (EXIT_SUCCESS);
}

