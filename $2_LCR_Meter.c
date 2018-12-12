// Author : Fahad Syed Karim
// Project : $2 LCR Meter w/ TM4C123GH6PM
// Instructor : Jason H. Losh
// Institute : UT Arlington (Fall 2018)
// Description : Reads commands w/Terminal, outputs inductance, capacitance, resistance & effective series resistance

----------------------------------------------------------------------------------------------------------------------------------

//Libraries

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"

----------------------------------------------------------------------------------------------------------------------------------

//Bitbanded Addresses

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

#define MEAS_LR      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) //PE3
#define MEAS_C       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //PE2

#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4))) //PD0
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) //PD3

#define DUT1         (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) //PD2
#define DUT2        (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4))) //PD1
#define DUT2_COMP        (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) //PC7

-----------------------------------------------------------------------------------------------------------------------------------

// Blocking Function

void waitPbPress()
{
    while(PUSH_BUTTON);
}

-------------------------------------------------------------------------------------------------------------------------------------

// Waiting Function

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

-----------------------------------------------------------------------------------------------------------------------------------

// Initialize Hardware

void initHw()
{
    //Clocking Settings
    
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOA ;

    // Configuring Ports 
    
    GPIO_PORTF_DIR_R = 0x0A;  // Outputs - Bits 1 & 3
    GPIO_PORTF_DR2R_R = 0x0A; // Drive Current - 2mA
    GPIO_PORTF_DEN_R = 0x1A;  // Digital Enable - Bits 1 & 3
    GPIO_PORTF_PUR_R = 0x10;  // Internal Pull-Up Resistors
    
    GPIO_PORTD_DIR_R = 0xD;  // Outputs - Bits 0, 2 & 3
    GPIO_PORTD_DR2R_R = 0xD; // Drive Current - 2mA
    GPIO_PORTD_DEN_R = 0xD;  // Digital Enable - Bits 0, 2 & 3

    GPIO_PORTE_DIR_R = 0x0E;  // Outputs - Bits 1, 2 & 3
    GPIO_PORTE_DR2R_R = 0x0E; // Drive Current - 2mA
    GPIO_PORTE_DEN_R = 0x0E;  // Digital Enable - Bits 1, 2 & 3

    RED_LED = 1;              // Turn on RED LED for flashing upon inititalization

    // UART0 Pins Configuration
    
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;                                    // Enable UART0
    GPIO_PORTA_DEN_R |= 3;                                                     // Digital Enable - Bits 0 & 1
    GPIO_PORTA_AFSEL_R |= 3;                                                   // Alternate Function Enable - Bits 0 & 1
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // UART0 Settings : 115200 baud, 8N1 format
    
    UART0_CTL_R = 0;                                              // Turn off UART0 (for safe programming)
    UART0_CC_R = UART_CC_CS_SYSCLK;                               // Set to System Clock (40MHz)
    UART0_IBRD_R = 21;                                            // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                            // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;              // Configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;  // Enable TX, RX, and Module

    // AIN6 as Analog Input (PD1 Pin)

    SYSCTL_RCGCADC_R |= 1;                           // Turn on ADC module 0 clocking
    GPIO_PORTD_AFSEL_R |= 0x02;                      // Alternative Functions Enable 
    GPIO_PORTD_DEN_R &= ~0x02;                       // Digital Operation Disable 
    GPIO_PORTD_AMSEL_R |= 0x02;                      // Analog Operation Enable
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // PLL as Time Base 
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // Sample Sequencer 3 (SS3) Disable (for configuration)
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // SS3 bit in ADCPSSI as Trigger
    ADC0_SSMUX3_R = 0x6;                              // First Sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // Mark First Sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // SS3 Enable 


     // AIN5 as Analog Input (PD2 Pin)

    SYSCTL_RCGCADC_R |= 2;                           //  Turn on ADC module 0 clocking
    GPIO_PORTD_AFSEL_R |= 0x04;                      // Alternative Functions Enable 
    GPIO_PORTD_DEN_R &= ~0x04;                       // Digital Operation Disable
    GPIO_PORTD_AMSEL_R |= 0x04;                      // Analog Operation Enable
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // PLL as Time Base 
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // Sample Sequencer 3 (SS3) Disable (for configuration)
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // SS3 bit in ADCPSSI as Trigger
    ADC1_SSMUX3_R = 0x5;                             //  First Sample to AN0
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 //  Mark First Sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // SS3 Enable 

    //  Analog Comparator 0 (PC7) Configuration

    SYSCTL_RCGCACMP_R    = 0x01;                            // System Clock Select
    GPIO_PORTC_DEN_R    &= ~(1 << 7);                       // Digital Enable - Bit 7
    GPIO_PORTC_DIR_R    &= ~(1 << 7);                       // Outputs - Bit 7
    GPIO_PORTC_AFSEL_R  |= (1 << 7);                        // Alternate Function Enable - Bit 7
    GPIO_PORTC_AMSEL_R  |= (1 << 7);                        // Analog Mode Select - Bit 7
    COMP_ACCTL0_R       |= (0x02 << 9) | (0x05 << 1);       // Rising Edge, Output Inverted
    NVIC_EN0_R          |= (1 << INT_COMP0 - 16);           // Interrupt Priority Set

    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;                                    // Enable Wide-Timer 5
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                                              // Turn-off Counter before Reconfiguring
    WTIMER5_CFG_R = 4;                                                            // Configure as 32-bit counter
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;   // Configure for Edge Time Mode, Count Up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;                                        // Measure Time from positive-positive edge
    WTIMER5_TAV_R =0;                                                             // Clear timer before operation

}

// End of Initializing Hardware

-------------------------------------------------------------------------------------------------------------------------------

//ADC Read Functions

// Read ADC0 Value

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // Set Start Bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // Wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // Get single result from the FIFO
}

// Read ADC1 Value

int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // Set Start Bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // Wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // Get single result from the FIFO
}

--------------------------------------------------------------------------------------------------------------------------------------

//UART0 Conditions

// Writes a serial character when the UART buffer is not full

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Writes a string when the UART buffer is not full

    void putsUart0(char* str)
    
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Returns with Serial Data once the buffer is not empty

char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

-------------------------------------------------------------------------------------------------------------------------------------

// Declare Variables


char c;
uint8_t max_chars = 80;
uint8_t count = 0;
uint8_t getout = 0;
char str[81];
uint8_t i;
uint8_t k;
uint8_t argc;
uint8_t pos[10];
uint8_t arg[10];
char type[10];
char tempbuff;
uint8_t leng;
char str_1[50];
char str_2[50];
char str_3[50];
char tempstr[50];
bool valid_voltage = 0;
bool valid_reset = 0;
bool valid_resistance = 0;
bool valid_high=0;
bool valid_low=0;
bool valid_lr=0;
bool valid_c =0;
bool valid_int=0;
uint8_t arg_1 =0;
uint8_t arg_2 =0;
double raw_2;
double raw_1;
double voltz_1;
double voltz_2;
bool timeMode = false;
bool timeUpdate = false;
uint32_t tau = 0;
char resist_str[10];
char cap_str[10];
char ind_str[10];
char esr_str[10];
double res=0;
double cap=0;
double ind=0;
double esrval=0;
bool valid_capacitance=0;
bool valid_esr=0;
bool valid_inductance=0;


---------------------------------------------------------------------------------------------------------------------------------------

    // Comparator (PC7) Interrupt Routine
    

    void compareISR(void)
    {
        tau = WTIMER5_TAV_R;                              // Read Counter Value
        WTIMER5_TAV_R = 0;                                // Zero Value

        __asm("             NOP");                        //Waiting Clock Cycles
        __asm("             NOP");
        __asm("             NOP");
        __asm("             NOP");

        COMP_ACMIS_R = 0x01;                              // Halt Comaparator
        WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                 // Turn off Wide Timer 5

        if (valid_resistance==1){                         // Resistance Routine

            tau=tau/40;                                   // Divide Tau by 40
                                                            
            if(tau<500){                                  // Calibrated Formulae for Resistance
        res = (tau*0.7388);}
            if(tau>500){
        res = (tau*0.7388)+199.6140;}

        }

        if (valid_capacitance==1){                      // Capacitance Routine

            tau = tau/40;                                // Divide Tau by 40

            if (tau<12500){                              // Calibrated Formulae for Capacitance    
                cap = ((0.0073*tau) - 0.3512);
            }
            if (tau>12500 && tau<134200){
                 cap = ((0.0071*tau) + 1.8599) ;
             }
            if (tau>134200){
                 cap = ((0.0071*tau) + 3.6806) ;
             }
        }
        
        if (valid_inductance==1){                             // Inductance Routine
            ind = ((0.669*tau) - 6.4601);                     // Calibrated Formulae for Inductance  
        }
        
    }

--------------------------------------------------------------------------------------------------------------------------------------
    
    // Resistance, Capacitance and Inductance Commands
    

    void resistor(void){                                      // Resistance Command

        MEAS_LR = 0;                                          // Turn off MEASURE_LR
        INTEGRATE = 1;                                        // Turn on INTEGRATE
        LOWSIDE_R = 1;                                        // Turn on LOWSIDE_R

        waitMicrosecond(5000000);                             // Wait 5 seconds

        COMP_ACINTEN_R = 0x01;                                // Enable Comaparator

        WTIMER5_TAV_R = 0;                                    // Clear Wide Timer 5 
        LOWSIDE_R = 0;                                         // Turn off LOWSIDE_R         
        MEAS_LR = 1;                                           // Turn on MEASURE_LR 
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;                      // Turn on Wide Timer 5
    }
    

    void capacitor(void){

        //Disbale all pins
        
        MEAS_LR = 0;                              
        MEAS_C = 0;
        LOWSIDE_R = 0;
        HIGHSIDE_R = 0;
        INTEGRATE = 0;
        
        MEAS_C=1;                           // Turn on MEAS_C
        LOWSIDE_R=1;                        // Turn on LOWSIDE_R
  
        waitMicrosecond(10000000);          // Wait 10 Seconds

        COMP_ACINTEN_R = 0x01;              // Enable Comparator

        WTIMER5_TAV_R = 0;                  // Clear Wide Timer 5
        LOWSIDE_R = 0;                      // Turn off LOWSIDE_R
        HIGHSIDE_R=1;                       // Turn on HIGHSIDE_R 
        MEAS_LR = 0;                        // Turn off MEAS_LR
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;    // Enable Wide Timer 5

    }
    

    void inductor(void){
    
        // Disable all pins

        MEAS_LR = 0;
        INTEGRATE = 0;
        LOWSIDE_R = 0;
        MEAS_C=0;
        HIGHSIDE_R=0;
        
        waitMicrosecond(5000);              // Wait 5 milliseconds

        COMP_ACINTEN_R = 0x01;              // Enable Comparator
        WTIMER5_TAV_R = 0;                  // Clear Wide Timer 5
        LOWSIDE_R = 1;                      // Enable LOWSIDE_R
        MEAS_LR = 1;                        // Enable MEAS_LR
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;    // Turn on Wide Timer 5

        waitMicrosecond(3000000);           // Wait 3 Seconds
    }

----------------------------------------------------------------------------------------------------------------------------------------

// Main Function 

int main(void)
{
    // Initialize Hardware
    
    initHw();
    
    //  Flash RED LED for 500 milliseconds
    
    RED_LED = 1;
    waitMicrosecond(500000);
    RED_LED = 0;
    waitMicrosecond(500000);


    // Display Greeting on Serial Terminal
    
    putsUart0("\r\n\r\nLCR Meter\r\n");

    while(1){

    waitMicrosecond(50000);
    
    // Reset variables
    
    valid_reset = 0;
    valid_voltage = 0;
    valid_resistance = 0;
    valid_capacitance=0;
    valid_inductance=0;
    valid_esr=0;
    valid_high=0;
    valid_low=0;
    valid_lr =0;
    valid_c=0;
    valid_int=0;
    getout=0;
    count=0;
    char str[81];

    putsUart0("Type Command: \r\n");

    //Command Input

    while (getout==0){

    c = getcUart0();

    if(c==0x08){                                          // Backspace Case
        if(count > 0){
            count--;
        }
        }

    else if (c == 0x0D){                                 // Carriage Return

        str[count]=NULL;
        getout=1;

    }

    else if(c >= 0x20){                                  // Printability Test
        str[count]=tolower(c);
        count++;
    }

    if(count == max_chars){
        getout=1;
    }

    }

    putsUart0(str);
    putsUart0("\r\n");
    putsUart0("\r\n");



    // Input String Filtering

    leng = strlen(str);

    for(i=0;i<leng;i++){                                                                                // Replace Delimiters w/Nulls 
        if(str[i]<0x30 | (str[i]>0x39 && str[i]<0x41) | (str[i]>0x5A && str[i]<0x61) | str[i]>0x7A ){
            str[i] = NULL;
        }
    }


    for(i=0;i<leng;i++){                                                          // Record Positions and Types

    if(i==0){
                if(str[i]>0x2F && str[i]<0x3A){
                    pos[k] = i;
                    type[k]='n';
                    k++;

                }

                else if(str[i]>0x60 && str[i]<0x7B){
                    pos[k] = i;
                    type[k]='a';
                    k++;

                }

             }

    else{

         if( ((str[i]>0x2F && str[i]<0x3A) && (str[i-1]==0x00)) || ((str[i]>0x2F && str[i]<0x3A) && (str[i-1]>0x60 && str[i-1]<0x7B)) ) {
                 pos[k] = i;
                 type[k] = 'n';
                 k++;


         }

         else if( ( (str[i]>0x60 && str[i]<0x7B) ) && (str[i-1]==0x00) || ((str[i]>0x60 && str[i]<0x7B) && (str[i-1]>0x2F && str[i-1]<0x3A)) ){

             pos[k] = i;
             type[k] = 'a';
             k++;

         }

        }

    }



    // Record Arguments and Types

    strcpy(str_1,&str[pos[0]]);
    strcpy(str_2,&str[pos[1]]);
    strcpy(str_3,&str[pos[2]]);



    // Run Commands According to Input Commands

    if(strcmp("voltage",&str[pos[0]])==0){                    // Voltage Command
        putsUart0("Valid Command: Voltage");
        putsUart0("\n\r");
        valid_voltage = 1;
    }

    if(valid_voltage==1){

        raw_1 = readAdc1Ss3();
        voltz_1 = ((raw_1 / 4096) * 3.3);
        sprintf(tempstr, "%lf", voltz_1);
        putsUart0("Voltage at DUT1: ");
        putsUart0(tempstr);
        putsUart0("\n\r");

        raw_2 = readAdc0Ss3();
        voltz_2 = ((raw_2 / 4096) * 3.3);
        sprintf(tempstr, "%lf", voltz_2);
        putsUart0("Voltage at DUT2: ");
        putsUart0(tempstr);
        putsUart0("\n\r");
        putsUart0("\r\n");

    }
    

    if(strcmp("measurelr",&str[pos[0]])==0){                          //MEAS_LR On Command
        putsUart0("Valid Command: Turn on MEAS_LR");
        putsUart0("\n\r");
        valid_lr = 1;
    }

    if(valid_lr ==1){
        MEAS_LR=0;
        MEAS_C=0;
        waitMicrosecond(5000);
        MEAS_LR=1;
    }

    if(strcmp("measurec",&str[pos[0]])==0){                             //MEAS_C On Command
        putsUart0("Valid Command: Turn on MEAS_C");
        putsUart0("\n\r");
        valid_c = 1;
    }

    if(valid_c ==1){
        MEAS_C=0;
        MEAS_LR=0;
        waitMicrosecond(5000);
        MEAS_C=1;
    }

    if(strcmp("highsider",&str[pos[0]])==0){                              //HIGHSIDE_R On Command
        putsUart0("Valid Command: Turn on HIGHSIDE_R");
        putsUart0("\n\r");
        valid_high = 1;
    }

    if(valid_high ==1){
        HIGHSIDE_R=0;
        LOWSIDE_R=0;
        waitMicrosecond(5000);
        HIGHSIDE_R=1;
    }
  
    if(strcmp("lowsider",&str[pos[0]])==0){                                      //LOWSIDE_R On Command
        putsUart0("Valid Command: Turn on LOWSIDE_R");
        putsUart0("\n\r");
        valid_low = 1;
    }

    if(valid_low ==1){
        LOWSIDE_R=0;
        HIGHSIDE_R=0;
        waitMicrosecond(5000);
        LOWSIDE_R=1;
    }

    if(strcmp("integrate",&str[pos[0]])==0){                                     //INTEGRATE On Command
        putsUart0("Valid Command: Turn on INTEGRATE");
        putsUart0("\n\r");
        valid_int = 1;
    }

    if(valid_int ==1){
        INTEGRATE=0;
        waitMicrosecond(5000);
        INTEGRATE=1;
    }

    // Reset Command

    if (strcmp("reset",&str[pos[0]])==0){
        putsUart0("RESET!");
        putsUart0("\n\r");
        putsUart0("*********************************");
        putsUart0("\n\n\n\r");
        waitMicrosecond(5000);
        valid_reset = 1;
    }

    if(valid_reset==1){

        NVIC_APINT_R  =  NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
    }


    // Resistance Command 

    if ((strcmp("resistance",&str[pos[0]])==0)){
        putsUart0("Valid Command: Resistance");
        putsUart0("\n\r");
        valid_resistance = 1;
    }

    if(valid_resistance==1){

        MEAS_LR = 0; //PE3
        MEAS_C = 0;  //PE2
        HIGHSIDE_R = 0; //PD0
        LOWSIDE_R = 0; //PE1
        INTEGRATE = 0; //PD3

        waitMicrosecond(1000000);

        COMP_ACREFCTL_R      = 0xF | (1 << 9);
        resistor();
        waitMicrosecond(1000000);
        sprintf(resist_str, "%u", tau);
        putsUart0("Tau: ");
        putsUart0(resist_str);
        putsUart0("\n\r");

        if(res<1000.0){
        sprintf(resist_str, "%lf", res);
        putsUart0("Resistance: ");
        putsUart0(resist_str);
        putsUart0(" Ohm");
        putsUart0("\n\r");
        putsUart0("\n\r");}

        if(res>1000.0){
        res=res/1000.0;
        sprintf(resist_str, "%lf", res);
        putsUart0("Resistance: ");
        putsUart0(resist_str);
        putsUart0(" kOhm");
        putsUart0("\n\r");
        putsUart0("\n\r");}

    }

    // Capacitance Command

    if ((strcmp("capacitance",&str[pos[0]])==0)){
        putsUart0("Valid Command: Capacitance");
        putsUart0("\n\r");
        valid_capacitance = 1;
    }

    if(valid_capacitance==1){

        MEAS_LR = 0; //PE3
        MEAS_C = 0;  //PE2
        HIGHSIDE_R = 0; //PD0
        LOWSIDE_R = 0; //PE1
        INTEGRATE = 0; //PD3

        waitMicrosecond(1000000);

        COMP_ACREFCTL_R      = 0xF | (1 << 9);
        capacitor();
        waitMicrosecond(10000000);
        sprintf(cap_str, "%u", tau);
        putsUart0("Tau: ");
        putsUart0(cap_str);
        putsUart0("\n\r");

        if(cap<1000.0){
        sprintf(cap_str, "%lf", cap);
        putsUart0("Capacitance: ");
        putsUart0(cap_str);
        putsUart0(" nF");}

        if(cap>1000.0){
            cap = cap/1000.0;
            sprintf(cap_str, "%lf", cap);
            putsUart0("Capacitance: ");
            putsUart0(cap_str);
            putsUart0(" uF");
        }

        putsUart0("\n\r");
        putsUart0("\n\r");

    }

    // Inductance Command

    if ((strcmp("inductance",&str[pos[0]])==0)){
        putsUart0("Valid Command: Inductance");
        putsUart0("\n\r");
        valid_inductance = 1;
    }

    if(valid_inductance==1){

                COMP_ACREFCTL_R = 0xF | (1 << 9);
                inductor();

                sprintf(ind_str, "%u", tau);
                putsUart0("Tau: ");
                putsUart0(ind_str);
                putsUart0("\n\r");
                putsUart0("Inductance: ");
                sprintf(ind_str, "%lf", ind);
                putsUart0(ind_str);
                putsUart0(" uH");
                putsUart0("\n\r");
    }

    //ESR Command

    if ((strcmp("esr",&str[pos[0]])==0)){
        putsUart0("Valid Command: ESR");
        putsUart0("\n\r");
        valid_esr = 1;
    }

    if(valid_esr==1){

                raw_1 = readAdc1Ss3();
                voltz_1 = ((raw_1 / 4096) * 3.3);
                raw_2 = readAdc0Ss3();
                voltz_2 = ((raw_2 / 4096) * 3.3);

                MEAS_LR=0;
                LOWSIDE_R=0;
                INTEGRATE = 0;
                HIGHSIDE_R=0;
                MEAS_C=0;

                waitMicrosecond(5000);

                MEAS_LR=1;
                LOWSIDE_R=1;

                waitMicrosecond(500000);

                res = ((33.0*voltz_1)/(voltz_2))-33.0;

                if(res<1000.0){
                sprintf(resist_str, "%lf", res);
                putsUart0("Resistance: ");
                putsUart0(resist_str);
                putsUart0(" Ohm");
                putsUart0("\n\r");
                putsUart0("\n\r");}

                if(res>1000.0){
                res=res/1000.0;
                sprintf(resist_str, "%lf", res);
                putsUart0("Resistance: ");
                putsUart0(resist_str);
                putsUart0(" kOhm");
                putsUart0("\n\r");
                putsUart0("\n\r");}

    }

    // Invalid Command Condition
    
    if(valid_voltage==0 && valid_reset==0 && valid_resistance==0 && valid_lr ==0 && valid_c ==0 && valid_int ==0 && valid_high ==0 && valid_low==0 && valid_capacitance==0 && valid_inductance==0 && valid_esr == 0){
        putsUart0("ERROR!! Retype your command\n\r\n\r");
    }

    putsUart0("*********************************");
    putsUart0("\n\n\n\r");

    }

    return 0;
}
