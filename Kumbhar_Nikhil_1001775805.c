#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "uart0.h"

#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 3 * 4)))
#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 5 * 4)))
#define FSS (*((volatile uint32_t *)(0x42000000 + (0x400053FC - 0x40000000) * 32 + 5 * 4)))

// PortF masks
#define GREEN_LED_MASK 8

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4

// PortA masks
#define LDAC_MASK 32

// PortB masks
#define TX_MASK 128
#define FSS_MASK 32
#define CLK_MASK 16

#define MAX_CHAR 80
#define MAX_FIELDS 6
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
#define PI 3.14

char str[MAX_CHAR + 1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount;
uint8_t argNo, channel_1mode, channel_2mode, Channel_no, N, diff_no, hilbert_no, alc_no;
char temp_value[100];
char temp_value_2[100];
uint16_t raw, R, Base, val, val1, INA, INB, raw1;
float instVoltage, Ofs, Freq, Amp, Duty_cycle, gainf1, gainf2, b, Res, corrfact;
uint32_t phase, deltaphase,phase_2, deltaphase_2;
uint16_t LUT[2][4096];
uint32_t cycles = 0;
bool Mode = false;
bool CycleMode = false;
bool diff_mode = false;
bool hilb_mode = false;
bool alc_mode = false;
uint32_t temp= 0;
uint8_t wave = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // SSI2 Configuration
    SYSCTL_RCGCSSI_R = SYSCTL_RCGCSSI_R2; // turn-on SSI2 clocking

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB;

    // Configure LED pin
    GPIO_PORTF_DIR_R = GREEN_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK;  // enable LED
}

void initA1N0()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    // SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    //  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE;

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN0_MASK; // select alternative functions for AIN0 (PE3)
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;  // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= AIN0_MASK; // turn on analog operation on pin PE3

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;         // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;     // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                    // set first sample to AIN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;      // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;      // enable SS3 for operation
}

//Not Used //
void initA1N1()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    // SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    //  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE;

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN1_MASK; // select alternative functions for AIN1 (PE2)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;  // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= AIN1_MASK; // turn on analog operation on pin PE2

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;         // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;     // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 1;                    // set first sample to AIN2
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;      // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;      // enable SS3 for operation
}


void initA1N1changed()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    // SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    //  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE;

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN1_MASK; // select alternative functions for AIN1 (PE2)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;  // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= AIN1_MASK; // turn on analog operation on pin PE2

    // Configure ADC
    ADC1_CC_R = ADC_CC_CS_SYSPLL;         // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;     // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 1;                    // set first sample to AIN2
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;      // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;      // enable SS3 for operation
}




void initSsi()
{

    SYSCTL_GPIOHBCTL_R = 0;

    // SSI2 Configuration
    SYSCTL_RCGCSSI_R = SYSCTL_RCGCSSI_R2; // turn-on SSI2 clocking

    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;

    GPIO_PORTA_DIR_R |= LDAC_MASK; // make bit an output
                                   // GPIO_PORTA_DR2R_R |= LDAC_MASK;
    GPIO_PORTA_DEN_R |= LDAC_MASK;

    // Configure SSI2 pins for SPI configuration
    GPIO_PORTB_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK;                                        // make SSI2 TX, FSS, and CLK outputs
    GPIO_PORTB_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK;                                       // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK;                                      // select alternative functions
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK;                                        // enable digital operation
    GPIO_PORTB_PUR_R |= CLK_MASK | FSS_MASK;                                                  // SCLK must be enabled when SPO=1 (see 15.4)

    // Configure the SSI2 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                     // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                 // select master mode
    SSI2_CC_R = 0;                                  // select system clock as the clock source
    SSI2_CPSR_R = 10;                               // set bit rate to 4 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                      // turn on SSI2
    //FSS = 0;
    LDAC = 1;
    //  FSS = 1;
}

void initTimer()
{

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;        // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;  // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;                   // set load value to 400 for 100 KHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;        // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);  // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;         // turn-on timer
}

void clean(char *str)
{
    int i = 0;
    while (i < 30)
    {
        str[i] = '\0';
        i++;
    }
}

void getString(char *str)
{

    // clean(str);
    uint8_t i = 0;
    char c;
    while (1)
    {
        c = getcUart0();
        if (c == 8 || c == 127)
        {

            if (i > 0)
            {
                str[i] = str[i--];
                continue;
            }
        }
        else if (c == 13 || c == 10)
        {
            str[i] = 0;
            break;
        }
        else if (c >= 32)
        {
            str[i++] = c;
            if (i >= MAX_CHAR)
            {
                str[i] = 0;
                break;
            }
            else
            {
                continue;
            }
        }
        else
        {
            continue;
        }
    }
}

void parseString(char *str)
{

    uint8_t i, j, len, argC = 0;

    uint8_t test;

    len = strlen(str);

    for (i = 0, j = 0; i < len; i++)
    {

        if ((str[i] >= 97 && str[i] <= 122) || (str[i] >= 65 && str[i] <= 90) || (str[i] >= 48 && str[i] <= 57) || (str[i] == 46) || (str[i] == 45))
        {

            if (i > 0)
            {

                if (!((test >= 65 && test <= 90) || (test >= 97 && test <= 122) || (test >= 48 && test <= 57) || (test == 46) || (test == 45)))
                {

                    pos[j++] = i;
                    argC++;
                }
            }
            else
            {

                pos[j++] = i;
                argC++;
            }
        }
        else
        {

            str[i] = 0;
        }

        test = str[i];

        argCount = argC;
    }
}

char *getArgstring(uint8_t argNo)
{

    if (argNo < argCount)
    {

        return &str[pos[argNo]];
    }
    else
    {
        putsUart0("error\r\n");
    }

    return (0);
}

uint32_t getArgint(uint8_t argNo)
{

    return atoi(getArgstring(argNo));
}

float getArgfloat(uint8_t argNo)
{

    return atof(getArgstring(argNo));
}

bool isCommand(char strcmd[], uint8_t minArg, uint8_t maxArg)
{

    if ((strcmp(strcmd, getArgstring(0)) == 0) && (argCount > minArg) && (argCount <= maxArg))
    {
        return true;
    }
    else
    {
        return false;
    }
}

int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3; // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);                  // wait until SS3 is not busy
    return ADC0_SSFIFO3_R; // get single result from the FIFO
}

int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3; // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);                  // wait until SS3 is not busy
    return ADC1_SSFIFO3_R; // get single result from the FIFO
}

void sendData(uint16_t R)
{
    FSS = 1;
    SSI2_DR_R = R; // write command
    FSS = 0;
}

void calcSine(uint8_t Channel_no, float Freq, float Amp, float Ofs)
{

    uint16_t i = 0;
    phase = 0;
    phase_2 = 0;

    if(alc_mode){

        Amp = ((Amp)/(corrfact));

    }

    if (Channel_no == 1){
        wave = 1;
        deltaphase = 0;
        deltaphase = ((Freq) * (pow(2, 32)) / 100000);
        for (i = 0; i < 4096; i++)
        {


            LUT[0][i] = ((-397.552 * ((sin((2 * PI * i) / 4096) * Amp) + Ofs)) + 1996.04 + 12288);
        }
    }
    else{
        deltaphase_2 = 0;
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
        for (i = 0; i < 4096; i++)
        {


            LUT[1][i] = ((-397.552 * ((sin((2 * PI * i) / 4096) * Amp) + Ofs)) + 1996.04 + 45056);
        }
    }

    initTimer();
}

void calcSquare(uint8_t Channel_no, float Freq, float Amp, float Ofs, float Duty_cycle)
{

    uint16_t i = 0;
    uint16_t dc = 0;
    phase = 0;
    phase_2 = 0;
    dc = ((Duty_cycle/100)*4096);

    if(alc_mode){

            Amp = ((Amp)/(corrfact));

        }
    if (Channel_no == 1)
    {
        wave = 2;
        deltaphase = ((Freq) * (pow(2, 32)) / 100000);

        for (i = 0; i < 4096; i++)
        {
            if (i < dc)
            {
                LUT[0][i] = ((-397.552 * ((Amp) + Ofs)) + 1996.04 + 12288);
            }
            else
            {
                LUT[0][i] = ((-397.552 * ((-Amp) + Ofs)) + 1996.04 + 12288);
            }
        }
    }
    else
    {
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
        for (i = 0; i < 4096; i++)
        {
            if (i < dc)
            {
                LUT[1][i] = ((-397.552 * ((Amp) + Ofs)) + 1996.04 + 45056);
            }
            else
            {
                LUT[1][i] = ((-397.552 * ((-Amp) + Ofs)) + 1996.04 + 45056);
            }
        }
    }

    initTimer();
}

void calcSawtooth(uint8_t Channel_no, float Freq, float Amp, float Ofs)
{

    uint16_t i = 0;
    phase = 0;
    phase_2 = 0;

    if(alc_mode){

            Amp = ((Amp)/(corrfact));

        }
    if (Channel_no == 1)
    {
        deltaphase = ((Freq) * (pow(2, 32)) / 100000);
        wave = 3;
        for (i = 0; i < 4096; i++)
        {


            LUT[0][i] = ((-397.552 * (((Amp / 4096) * i) + Ofs) + 1996.04 + 12288));
        }
    }
    else
    {
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
        for (i = 0; i < 4096; i++)
        {


            LUT[1][i] = ((-397.552 * (((Amp / 4096) * i) + Ofs) + 1996.04 + 45056));
        }
    }

    initTimer();
}

void calcTriangle(uint8_t Channel_no, float Freq, float Amp, float Ofs)
{

    uint16_t i = 0;
    phase = 0;
    phase_2 = 0;

    if(alc_mode){

            Amp = ((Amp)/(corrfact));

        }
    if (Channel_no == 1)
    {
        wave = 4;
        deltaphase = ((Freq) * (pow(2, 32)) / 100000);
        for (i = 0; i < 4096; i++)
        {
            if (i < 2048)
            {
                LUT[0][i] = ((-397.552 * (((Amp / 2048) * i) + Ofs) + 1996.04 + 12288));
            }
            else
            {
                LUT[0][i] = ((-397.552 * (((-Amp / 2048) * i + 2 * Amp) + Ofs) + 1996.04 + 12288));
            }
        }
    }
    else
    {
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
        for (i = 0; i < 4096; i++)
        {
            if (i < 2048)
            {
                LUT[1][i] = ((-397.552 * (((Amp / 2048) * i) + Ofs) + 1996.04 + 45056));
            }
            else
            {
                LUT[1][i] = ((-397.552 * (((-Amp / 2048) * i + 2 * Amp) + Ofs) + 1996.04 + 45056));
            }
        }
    }

    initTimer();
}

void calcdiff(){
    uint16_t i = 0;
    phase_2 = 0;


    if(wave == 1){
        deltaphase_2 = 0;
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
               for (i = 0; i < 4096; i++)
               {


                   LUT[1][i] = ((-397.552 * ((sin((2 * PI * i) / 4096) * -Amp) + Ofs)) + 1996.04 + 45056);
               }

    }
    if(wave == 2){
        deltaphase_2 = 0;
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
               for (i = 0; i < 4096; i++)
               {
                   if (i < 2048)
                   {
                       LUT[1][i] = ((-397.552 * ((-Amp) + Ofs)) + 1996.04 + 45056);
                   }
                   else
                   {
                       LUT[1][i] = ((-397.552 * ((Amp) + Ofs)) + 1996.04 + 45056);
                   }
               }


       }
    if(wave == 3){
        deltaphase_2 = 0;
            deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
            channel_2mode = 1;
            for (i = 0; i < 4096; i++)
                   {
                       LUT[1][i] = ((-397.552 * (((-Amp / 4096) * i) + Ofs) + 1996.04 + 45056));
                   }


           }
    if(wave == 4){
        deltaphase_2 = 0;
                deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
                channel_2mode = 1;
                for (i = 0; i < 4096; i++)
                        {
                            if (i < 2048)
                            {
                                LUT[1][i] = ((-397.552 * (((-Amp / 2048) * i) + Ofs) + 1996.04 + 45056));
                            }
                            else
                            {
                                LUT[1][i] = ((-397.552 * (((Amp / 2048) * i + 2 * -Amp) + Ofs) + 1996.04 + 45056));
                            }
                        }


               }
 //   initTimer();

}


void calchilb(){
    uint16_t i = 0;
    phase_2 = 0;


    if(wave == 1){
        deltaphase_2 = ((Freq) * (pow(2, 32)) / 100000);
        channel_2mode = 1;
               for (i = 0; i < 4096; i++)
               {


                   LUT[1][i] = ((-397.552 * ((cos((2 * PI * i) / 4096) * -Amp) + Ofs)) + 1996.04 + 45056);
               }

    }


}

void calcvoltage1(){
        initA1N0();
        raw = readAdc0Ss3();
        sprintf(temp_value, "raw value:  %4u\r\n", raw);
        putsUart0(temp_value);
        instVoltage = ((raw*3.3)/4096);
        sprintf(temp_value, "Volts:   %4.3f\r\n", instVoltage);
        putsUart0(temp_value);

}

void calcvoltage2(){
        initA1N1changed();
        raw1 = readAdc1Ss3();
        sprintf(temp_value_2, "raw value:  %4u\r\n", raw1);
        putsUart0(temp_value_2);
        instVoltage = ((raw1*3.3)/4096);
        sprintf(temp_value_2, "Volts:  %4.3f\r\n", instVoltage);
        putsUart0(temp_value_2);
}


void calcGain(){

    uint16_t  i, j;
    phase = 0;                  //need to check this or either move this inside for loop
    deltaphase = 0;
    uint8_t out;
    for(j=gainf1; j<gainf2; j*=2){
    deltaphase = 0;
    deltaphase = ((j) * (pow(2, 32)) / 100000);
    for(i=0; i<4096; i++){

        LUT[0][i] = ((-397.552 * ((sin((2 * PI * i) / 4096) *4) + 0)) + 1996.04 + 12288);
        }
    waitMicrosecond(500000);
    waitMicrosecond(500000);
    waitMicrosecond(500000);
    waitMicrosecond(500000);
    calcvoltage1();
    calcvoltage2();
    out = ((raw1)/(raw));
    sprintf(temp_value, "Gain:  %4u\r\n", out);
    }
  //  initTimer();

//    for(j=gainf1; j<gainf2; j*=2){
//       deltaphase = ((j) * (pow(2, 32)) / 100000);

    //    calcvoltage1();
   //     calcvoltage2();

//        raw = readAdc0Ss3();
//        //sprintf(temp_value, "raw value:  %4u\r\n", raw);
//       // putsUart0(temp_value);
//        raw1 = readAdc1Ss3();
//        out = ((raw1)/(raw));
//       // sprintf(temp_value_2, "raw value:  %4u\r\n", raw1);
//      //  putsUart0(temp_value_2);
//      //  sprintf(temp_value, "%4u     %4u\r\n",j,out);
//        sprintf(temp_value, "Gain:  %4.3f\r\n", out);
//        putsUart0(temp_value);
       // delay4Cycles();

   //     waitMicrosecond(500000);
     //   waitMicrosecond(500000);

//   }
//    if(j>gainf2){
//        j = gainf2;
//        deltaphase = ((gainf2) * (pow(2, 32)) / 100000);
//        sprintf(temp_value, "Gain:  %4.3f\r\n", out);
//        putsUart0(temp_value);
//    }

//    initTimer();

}

void calcalc(){

    float test;
    calcvoltage1();
    Res = ((instVoltage)*50)/((b+0.1-instVoltage));
    sprintf(temp_value_2, "Value of R:  %4.3f\r\n", Res);
    putsUart0(temp_value_2);
    test = (Res+50);
    corrfact = ((Res)/(test));
    sprintf(temp_value_2, "Value of Correc fact:  %4.3f\r\n", corrfact);
    putsUart0(temp_value_2);

}
void timer1Isr()
{

    LDAC = 0;
    delay4Cycles();
    LDAC = 1;
    if (Mode)
    {

//        LDAC=0;
//        delay4Cycles();
//        LDAC=1;

        if (CycleMode)
        {

//            LDAC = 0;
//            delay4Cycles();
//            LDAC = 1;
            if (cycles > 0)
            {

                phase += deltaphase;
                val = LUT[0][phase >> 20];
                SSI2_DR_R = val;
                if (channel_2mode == 1)
                {
                    phase_2 += deltaphase_2;
                    val1 = LUT[1][phase_2 >> 20];
                    SSI2_DR_R = val1;
                }
                cycles--;
            }
            if(cycles ==  0){
                phase += deltaphase;
               // phase_2 += deltaphase_2;
                val = 1996.04 + 12288;
               // val1 = 1996.04 + 45056;
                SSI2_DR_R = val;
              if (channel_2mode == 1)
              {
                  phase_2 += deltaphase_2;
                  val1 = 1996.04 + 45056;
                  SSI2_DR_R = val1;
              }
            }



        }
        else
        {
           // temp++;
            phase += deltaphase;
            val = LUT[0][phase >> 20];

            SSI2_DR_R = val;
            if (channel_2mode == 1)
            {
                phase_2 += deltaphase_2;
                val1 = LUT[1][phase_2 >> 20];
                SSI2_DR_R = val1;
            }
        }
    }

    /*
 * Normal Working Without Cycles Test Code *
       if(Mode){

                 LDAC=0;
                 delay4Cycles();
                 LDAC=1;

                 phase += deltaphase;
                 val = LUT[0][phase>>20];
                 val1 = LUT[1][phase>>20];

                 SSI2_DR_R = val;
                 if(channel_2mode == 1){
                 SSI2_DR_R = val1;
                 }

             }

*/

    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // clear interrupt flag
}

void stop()
{

    SSI2_DR_R = 1996.04 + 12288;
    SSI2_DR_R = 1996.04 + 45056;
}

void test()
{
    while (cycles > 0)
    {
        cycles--;
    }
    if (cycles == 0)
    {
        putsUart0("the count decremented to zero \r\n");
    }
}

void empty(){

    uint16_t i =0;
    channel_2mode = 0;
    SSI2_DR_R = 1996.04 + 12288;
                   for (i = 0; i < 4096; i++)
                   {
                       LUT[1][i] = (1996.04 + 45056);
                   }




}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    initSsi();
    //  initTimer();
    //  initA1N0();

    while (1)
    {

        GREEN_LED = 1;
        waitMicrosecond(500000);
        GREEN_LED = 0;
        waitMicrosecond(500000);

        getString(str); //Get string Function
        /*                                              //To debug String Function
        putsUart0(str);
        */
        parseString(str); //Parse string Function

        /*
        uint8_t argCount = 0;                         // getArgstring Function check
        getArgstring(argNo);
        for(i=0; i<argCount; i++)
        {
        putsUart0(getArgstring(i));
        putsUart0("\r\n");

        }
        */

        uint8_t a = 0; /* Flag and parameters extraction variables    */
        uint8_t flag = 0;
        //float b;
        //float Ofs = 0;

        if (isCommand("sine", 3, 5))
        {

            Channel_no = getArgint(1);
            Freq = getArgfloat(2);
            Amp = getArgfloat(3);
            Ofs = getArgfloat(4);

            if (Channel_no != 0 && Freq != 0 && Amp != 0)
            {
                flag = 1;
                //    mode = 2;
                calcSine(Channel_no, Freq, Amp, Ofs);
                putsUart0("valid Sine Command\r\n");
            }
        }
        if (isCommand("dc", 2, 3))
        {
            a = getArgint(1);
            b = getArgfloat(2);
            if (a != 0)
            {
                flag = 1;
                Mode = false;
                //putsUart0("valid Command\r\n");
                if (a == 1)
                {
                    Base = 12288;
                    R = -397.552 * b + 1996.04 + Base;
                    FSS = 0;
                    SSI2_DR_R = R;
                    while (SSI2_SR_R & SSI_SR_BSY)
                        ;
                    FSS = 1;
                    LDAC = 0;
                    delay4Cycles();
                    LDAC = 1;
                    putsUart0("DC Command Running on Channel 1 \r\n");
                }
                if (a == 2)
                {
                    Base = 45056;
                    R = -397.552 * b + 1996.04 + Base;
                    FSS = 0;
                    SSI2_DR_R = R;
                    while (SSI2_SR_R & SSI_SR_BSY)
                        ;
                    FSS = 1;
                    LDAC = 0;
                    delay4Cycles();
                    LDAC = 1;
                    putsUart0("DC Command Running on Channel 2 \r\n");
                }
                if (a <= 0 || a > 2)
                {
                    putsUart0("Please Enter Correct Channel Number Either 1 or 2 \r\n");
                }
            }
        }

        if (isCommand("reset", 0, 1))
        {

            flag = 1;
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            putsUart0("Circuit has been reseted \r\n");
        }

        if (isCommand("voltage", 1, 2))
        {

            a = getArgint(1);
            if (a != 0)
            {
                flag = 1;
                if (a == 1)
                {
                    calcvoltage1();
//                    initA1N0();
//                    raw = readAdc0Ss3();
//                    sprintf(temp_value, "raw value:  %4u\r\n", raw);
//                    putsUart0(temp_value);
//                    instVoltage = ((raw*3.3)/4096);
//                    sprintf(temp_value, "Volts:   %4.3f\r\n", instVoltage);
//                    putsUart0(temp_value);
                }
                else if (a == 2)
                {
                   // initA1N1();
                   // raw1 = readAdc0Ss3();
                    calcvoltage2();
//                   initA1N1changed();
//                   raw1 = readAdc1Ss3();
//                    sprintf(temp_value_2, "raw value:  %4u\r\n", raw1);
//                    putsUart0(temp_value_2);
//                    instVoltage = ((raw1*3.3)/4096);
//                    sprintf(temp_value_2, "Volts:  %4.3f\r\n", instVoltage);
//                    putsUart0(temp_value_2);
                }
                else
                {
                    // flag = 0;
                    putsUart0("Wrong Channel Number");
                }
            }
        }

        if (isCommand("run", 0, 1))
        {

            flag = 1;
            Mode = true;
           // CycleMode = true;
        }

        if (isCommand("stop", 0, 1))
        {

            Mode = false;
            CycleMode = false;
            flag = 1;
            // SSI2_DR_R = 1996.04 + 12288;
            // SSI2_DR_R = 1996.04 + 45056;
            stop();
            putsUart0("Waveform Stops \r\n");
        }

        if (isCommand("cycles", 2, 3))
        {
            Channel_no = getArgint(1);
            N = getArgint(2);
            if ((Channel_no > 0 && Channel_no < 3))
            {
                flag = 1;
                // Mode=true;
                if (N == 0)
                {
                    CycleMode = false;
                }
                else
                {
                    CycleMode = true;
                    cycles = ((100000 / Freq) * N);
                    //test();
                }
            }
        }

        if (isCommand("square", 3, 6))
        {
            Channel_no = getArgint(1);
            Freq = getArgfloat(2);
            Amp = getArgfloat(3);
            Ofs = getArgfloat(4);
            Duty_cycle = getArgfloat(5);
            if (Channel_no != 0 && Freq != 0 && Amp != 0 && Duty_cycle != 0)
            {
                flag = 1;
                calcSquare(Channel_no, Freq, Amp, Ofs,Duty_cycle);
                putsUart0("valid Square Command\r\n");
            }
        }

        if (isCommand("sawtooth", 3, 5))
        {
            Channel_no = getArgint(1);
            Freq = getArgfloat(2);
            Amp = getArgfloat(3);
            Ofs = getArgfloat(4);
            if (Channel_no != 0 && Freq != 0 && Amp != 0)
            {
                flag = 1;
                calcSawtooth(Channel_no, Freq, Amp, Ofs);
                putsUart0("valid Sawtooth Command\r\n");
            }
        }

        if (isCommand("triangle", 3, 5))
        {
            Channel_no = getArgint(1);
            Freq = getArgfloat(2);
            Amp = getArgfloat(3);
            Ofs = getArgfloat(4);
            if (Channel_no != 0 && Freq != 0 && Amp != 0)
            {
                flag = 1;
                calcTriangle(Channel_no, Freq, Amp, Ofs);
                putsUart0("valid Triangle Command\r\n");
            }
        }

        if (isCommand("differential", 1, 2))
            {
            diff_no = getArgint(1);
            if(diff_no !=0){
                flag = 1;
                diff_mode = true;
                calcdiff();
                putsUart0("differential mode turned On\r\n");
            }
            else{
                flag = 1;
                diff_mode = false;
                Mode = false;
                //stop();
                empty();
                putsUart0("differential mode turned off\r\n");
            }


            }
        if(isCommand("hilbert",1,2)){

            hilbert_no = getArgint(1);
            if(hilbert_no !=0){
                flag = 1;
                hilb_mode = true;
                calchilb();
                putsUart0("hilbert mode turned On\r\n");
            }
            else{
                flag = 1;
                hilb_mode = false;
                Mode = false;
                //stop();
                empty();
                putsUart0("hilbert mode turned off\r\n");
            }


        }
        if(isCommand("gain",2,3)){
            gainf1 = getArgfloat(1);
            gainf2 = getArgfloat(2);
            if(gainf1 !=0 && gainf2 !=0){
                flag=1;
                Mode= true;
                initA1N0();
                initA1N1changed();
                putsUart0("Gain mode turned On\r\n");
                putsUart0("Frequency  Gain\r\n");
                calcGain();

            }
        }

        if(isCommand("alc",1,2)){
            alc_no = getArgint(1);
                        if(alc_no !=0){
                            flag = 1;
                            alc_mode = true;
                            calcalc();
                            putsUart0("alc mode turned On\r\n");
                        }
                        else{
                            flag = 1;
                            alc_mode = false;
                            Mode = false;
                            //stop();
                            //empty();
                            putsUart0("alc mode turned off\r\n");
                        }
                }

        if (flag == 0)
        {

            putsUart0("Invalid Command\r\n");
        }
    }

   // return 0;
}
