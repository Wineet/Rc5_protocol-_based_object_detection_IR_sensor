//
/*
 * File:   main.c
 * Author: Admin
 *
 * Created on 7 August, 2021, 12:41 PM
 */
// PIC12F1612 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config ZCD = OFF        // Zero Cross Detect Disable Bit (ZCD disable.  ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PLLEN = OFF       // PLL Enable Bit (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// CONFIG3
#pragma config WDTCPS = WDTCPS1F// WDT Period Select (Software Control (WDTPS))
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config WDTCWS = WDTCWSSW// WDT Window Select (Software WDT window size control (WDTWS bits))
#pragma config WDTCCS = SWC     // WDT Input Clock Selector (Software control, controlled by WDTCS bits)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define TIMER_VAL   0                   // 2097ms   
#define TX_PULSE_VAL    62410           // 100ms
#define TIMER_PULSE_VALUE   65507       // 890us
#define _XTAL_FREQ  1000000
#define RC5_BIT_TIMER 2

#include <xc.h>
#include <stdio.h>
#include <pic12f1612.h>

void ms_delay(unsigned int time);
void board_init();
void power_on_ind();
void power_down_ind();
void init_clock();
void interrupt_config();
void timer1_init();     // timer 1 -->16 bit
void timer1_on();
void timer1_off();
void load_timer(unsigned int timer_value);
void timer0_init();
void timer0_clear();
void load_timer0(unsigned char tmr0_val);
int timer0_Overflow_flag();
void timer0_off();
void timer0_on();

void set_tx(int tx_bit);
void tmr_tx_pulse();
void set_tx_high();
void set_tx_low();
void rc5_trans(unsigned short rc_data);
int get_rx_state();

void __interrupt() INTERRUPT_TMR( void);

typedef enum {OUTPUT=0,LOW=0,INPUT=1,HIGH=1}pinmode_e;
typedef enum{BIT_INVALID,STARTED,INPROG1,COMP1,INPROG2,COMP2} start_bit_e;
typedef enum{TRANSMIT_STARTED,TRANSMIT_INPROGRESS,TRANSMIT_DONE,TRANSMIT_INVALD}transmission_state_e;
int bit_count=0;
start_bit_e start_bit=BIT_INVALID;
char bit_arr[28]={0};
int rec_arr[28]={0};
int rec_bit_count=0;
transmission_state_e trans_state=TRANSMIT_INVALD;
int data_match_flag=1;

void __interrupt() INTERRUPT_TMR( void)
{
    
#if 1       // IOC Interrupt
    if(HIGH==INTCONbits.IOCIF){   
        if(HIGH==IOCAFbits.IOCAF1)
        {         // Low edge detected     
            rec_arr[rec_bit_count]=get_rx_state();
            if(rec_bit_count == 0)
            {
                IOCANbits.IOCAN1=HIGH;         // Falling Edge Trigger Active
                IOCAPbits.IOCAP1=HIGH;          //  Rising Edge Trigger            
            }
            rec_bit_count++;
#if 1
            if(rec_bit_count>28)
            {
                rec_bit_count = 0;              // All Data received start trigger on Low edge
                IOCANbits.IOCAN1=HIGH;         // Falling Edge Trigger Active
                IOCAPbits.IOCAP1=LOW;          // Disable Rising Edge Trigger

            }
#endif
        }
        
        IOCAFbits.IOCAF1=LOW;
        INTCONbits.IOCIF=LOW;
    }
#endif
    
    
#if 1
    /* Timer 1 ISR*/
   if( HIGH == PIR1bits.TMR1IF)
    {
        timer1_off();
        PIR1bits.TMR1IF=LOW;
        trans_state=TRANSMIT_INPROGRESS;
        if(bit_count>28)
        {
            trans_state=TRANSMIT_DONE;
            bit_count = 0;
            timer1_off();
            set_tx_high();
        }
        if(TRANSMIT_INPROGRESS == trans_state || TRANSMIT_INPROGRESS == trans_state)
        {
            set_tx(bit_arr[bit_count]);
            load_timer(TIMER_PULSE_VALUE); 
            bit_count++;
            timer1_on();
        }   
    }
#endif
    
#if 1
    if( HIGH == INTCONbits.TMR0IF)
    {
        timer0_off();
        LATAbits.LATA5=HIGH;
        ms_delay(1000);
        LATAbits.LATA5=LOW;
        ms_delay(1000);
        INTCONbits.TMR0IF = LOW;
        load_timer0(RC5_BIT_TIMER);
        timer0_on();
    }
    
#endif
}

void timer1_init()
{
    /* Internal clock (FOSC/4)
     * Pre scalar 1:8
     * 
     */
    T1CONbits.TMR1ON=0;     // Timer Off
    T1CONbits.T1CKPS=0b11;  //(prescalar 1:8 Enabled)
    load_timer(TIMER_VAL);
}
void timer1_on()
{
    T1CONbits.TMR1ON=1;     // Timer ON
}
void timer1_off()
{
    T1CONbits.TMR1ON=0;     // Timer Off
}
void load_timer(unsigned int timer_value)
{
    TMR1=timer_value;   
}

void timer_clear()
{
    TMR1=0X00;
}
int timer_Overflow_flag()
{
    if( PIR1bits.TMR1IF==1)
        return 1;
    else
        return 0;
}



void timer0_init()
{
    OPTION_REGbits.TMR0SE=LOW;
    OPTION_REGbits.PSA=LOW;     // Prescalar Enable
    OPTION_REGbits.PS =0b000;  // Prescaler Value
//    timer0_clear();
}
void timer0_clear()
{
    TMR0 = 0X00;    // 8 bit Register
}
void load_timer0(unsigned char tmr0_val)
{
    TMR0=tmr0_val;
}
int timer0_Overflow_flag()
{
if( INTCONbits.TMR0IF==1)
    return 1;
else
    return 0;
}
void timer0_on()
{
   OPTION_REGbits.T0CS=LOW;    //Internal oscillation Clock  & In this case used for timer on and off
}
void timer0_off()
{
     OPTION_REGbits.T0CS=HIGH;    //Internal oscillation Clock & In this case used for timer on and off
}

void interrupt_config()
{
    /* Global Interrupt Enable
     * Peripheral Interrupt Enable
     * Timer1 Interrupt Enable
     * Timer1 Flag clear
     */
    INTCONbits.GIE=1;
    INTCONbits.PEIE=1;
    INTCONbits.IOCIE=1; // Interrupt On change Enable
    INTCONbits.IOCIF=0; // Flag Clear
    INTCONbits.TMR0IE=1; // Timer0 Overflow Interrupt Enabled
    INTCONbits.TMR0IF=0;// Timer0 Flag cleared
    PIE1bits.TMR1IE=1;  // Timer Overflow Interrupt Enable
    PIR1bits.TMR1IF=0;  // Flag Clear 
}
void  rc5_trans(unsigned short rc_data)
{   
    int local_bit_count=0;
    int arr_count=0;
    while(1)
    {
    if(  ( (rc_data<<local_bit_count)&0x2000) )
    {
        bit_arr[arr_count++]=0;
        bit_arr[arr_count++]=1;
        local_bit_count++;
    }
    else
    {
        bit_arr[arr_count++]=1;
        bit_arr[arr_count++]=0;
        local_bit_count++;
    }
    if(local_bit_count>=14)
    {
        break;
    }
    }
    if(TRANSMIT_DONE == trans_state || TRANSMIT_INVALD == trans_state)
    {
        trans_state=TRANSMIT_STARTED;
        set_tx_high();
        tmr_tx_pulse();     // gives 100 ms Pulse
    }
}
void tmr_tx_pulse()
{
    load_timer(TX_PULSE_VAL);
    timer1_on();
}
void set_tx_high()
{
    LATAbits.LATA2 = HIGH;
}
void set_tx_low()
{
    LATAbits.LATA2 = LOW;
}
void set_tx(int tx_bit)
{
    LATAbits.LATA2= (tx_bit & 0x01);
}
void board_init()
{
    TRISAbits.TRISA5 = OUTPUT;
    TRISAbits.TRISA2 = OUTPUT;
    TRISAbits.TRISA1 = INPUT;
    TRISAbits.TRISA4 = OUTPUT;
    LATAbits.LATA5 = LOW;
    LATAbits.LATA2 = LOW;
    LATAbits.LATA4 = LOW;
    ANSELAbits.ANSA1=0;          //default analog pin =0;
    WPUAbits.WPUA1=1;           //internal pullup
    set_tx_high();
    init_clock();
    interrupt_config();
    timer1_init();
    timer0_init();
}
void init_clock(){
    
    OSCCONbits.IRCF = 0b1011;  //Set the Internal Oscillator Frequency to 1Mhz
    OSCCONbits.SCS = 0b10;
}
void power_on_ind()
{
    LATAbits.LATA5 = HIGH;
}
void power_down_ind()
{
    LATAbits.LATA5 = LOW;
}
void ms_delay(unsigned int time){
    for(int i=0;i<time;i++){
       __delay_ms(1);
    }
}

int get_rx_state(){
    return PORTAbits.RA1;
}
void compare_rx_tx_data()
{
    for(int i=0;i<28;i++)
        {
            if( rec_arr[i] != bit_arr[i] )
            {
                LATAbits.LATA4=LOW;
                data_match_flag=0;
            }
        }

}
void clear_rx_array()
{
    for(int i=0;i<28;i++)
    {
         rec_arr[i]=0;
    }
}

void main(void) {

    board_init();
    ms_delay(5000);
    timer0_on();
    while(1);
    IOCANbits.IOCAN1=HIGH;         // Falling Edge Trigger Active
    IOCAPbits.IOCAP1=LOW;          // Disable Rising Edge Trigger
    power_on_ind();                 //Power on LED Indication
    set_tx_high();
    int rc5_Tx_data=0x3FFF;
    
    while(1)
    {
   
        rc5_trans(rc5_Tx_data);
        ms_delay(500);
        
//commented Code is for debugging 
//        for(int i=0;i<28;i++)
//        {
//          LATAbits.LATA0= rec_arr[i];
//          __delay_us(895);
//        }
//        
        compare_rx_tx_data();      
        if(1 == data_match_flag)
        {
            LATAbits.LATA4=HIGH;
        }
        else
        {
            rec_bit_count = 0;              // All Data received start trigger on Low edge
            IOCANbits.IOCAN1=HIGH;         // Falling Edge Trigger Active
            IOCAPbits.IOCAP1=LOW;          // Disable Rising Edge Trigger
            LATAbits.LATA4=LOW;           
            ms_delay(1000);
        }
        clear_rx_array();
        data_match_flag=1;

    }
    return ;
}