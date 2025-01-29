#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "adc0.h"
#include "adc1.h"
#include "gpio.h"
#include "nvic.h"
#include "tm4c123gh6pm.h"
#include "rgb_led.h"
#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;
uint16_t maxindex;

#define BUFFER_SIZE 64  // Example size for each buffer A and B
#define TOTAL_SIZE BUFFER_SIZE * 2  // Total size for both buffers

uint16_t ani0[TOTAL_SIZE / 4], ani1[TOTAL_SIZE / 4], ani2[TOTAL_SIZE / 4], ani3[TOTAL_SIZE / 4];
uint16_t index_ani0 = 0, index_ani1 = 0, index_ani2 = 0, index_ani3 = 0;


uint16_t udmaindex;
#define UDMA_CHCTL_XFERSIZE 63
#define ADC_SSDC1_S3DCSEL_S 12
#define ADC_SS1 31
#define MIC_1 PORTE,3
#define MIC_2 PORTE,2
#define MIC_3 PORTE,1
#define MIC_4 PORTD,3
#define THRESHOLD_VALUE 0x000000C8
#define VOLTAGE_REFERENCE_0 0x00000190
#define AC_PIN PORTC,7

#define MAX_LAG 10  // Adjust based on expected max time delay
#define ARRAY_SIZE 128  // Confirm to match your data array sizes

uint16_t activeBuffer;
uint16_t TtimeBuffer;

//volatile int T12 , T23, T13;
uint16_t T12 , T23, T13;
uint16_t maxani1 ,maxani2 ,maxani3;
uint16_t corrResults[2 * MAX_LAG + 1];
uint16_t result[128];

uint32_t AC_flag = 0;


void processBuffer(volatile uint16_t *buffer);
int findMaxCorrelationLag(uint16_t *signal1, uint16_t *signal2, int size);
uint16_t max_index(uint16_t *buffer);
void getsUart0(USER_DATA *data);
uint16_t cross_correlation(uint16_t *x, uint16_t *y, uint16_t n, uint16_t window_size, uint16_t *result);

uint32_t x;
uint32_t y;




// Destination arrays for ping-pong buffering
#pragma DATA_ALIGN (Base_arr, 1024)
volatile uint8_t Base_arr[1024];

volatile uint16_t bufferA[64] = {0};
volatile uint16_t bufferB[64] = {0};

uint16_t angle;

volatile uint32_t *UDMA_PSEP_R;
volatile uint32_t *UDMA_ASEP_R;
volatile uint32_t *UDMA_PDEP_R;
volatile uint32_t *UDMA_ADEP_R;
volatile uint32_t *UDMA_PCW_R;
volatile uint32_t *UDMA_ACW_R;
volatile uint32_t *BAdd;
//volatile uint32_t *SAdd;

uint16_t raw_mic_1 = 0;
uint16_t raw_mic_2 = 0;
uint16_t raw_mic_3 = 0;
uint16_t raw_mic_4 = 0;


void intHW()
{
    initSystemClockTo40Mhz();
    //SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    //enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    selectPinAnalogInput(MIC_1);
    selectPinAnalogInput(MIC_2);
    selectPinAnalogInput(MIC_3);
    selectPinAnalogInput(MIC_4);
    //selectPinAnalogInput(AC_PIN);
    _delay_cycles(3);
}
bool compare_string(char str1[], char str2[]){

    uint8_t i=0, j=0;

    while (str1[i] != '\0' && str2[j] != '\0')
    {
        if (str1[i] != str2[j])
            return false;
        i++;
        j++;
    }

    if(i == j)
        return true;
    else
        return false;
}

bool isCommand(USER_DATA* data, char strCommand[], uint8_t minArguments)
{
    if (data->fieldCount >= minArguments + 1)
    {
        if(compare_string(strCommand, &data->buffer[data->fieldPosition[0]]))
        {
            return true;
        }
    }

    return false;
}

void parseFields(USER_DATA *data)
{
    char PType = 'd';
    data->fieldCount = 0;
    uint8_t i = 0;

    while(true)
    {
        if(data->buffer[i]=='\0'){
            break;
        }
        char currentChar = data->buffer[i];
        char CType = 'd';
        if((currentChar >= 65 && currentChar<= 90)|| (currentChar >= 97 && currentChar <=122)){
            CType = 'a';
        }
        else if (currentChar >= 48 && currentChar <= 57){
            CType = 'n';
        }
        else{
            data->buffer[i] = '\0';
            //CType='d';
        }
        if (CType != PType)
        {
            if (data->fieldCount < MAX_FIELDS && CType != 'd')
            {
                // Record the type and position of the field
                data->fieldType[data->fieldCount] = CType;
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
            }
            PType = CType; // Update previous type

        }
        i++;
    }
}

void intDC()
{
    ADC0_SAC_R |= ADC_SAC_AVG_64X;


   // ADC0_DCRIC_R |= ADC_DCRIC_DCINT0 | ADC_DCRIC_DCINT1 | ADC_DCRIC_DCINT2 | ADC_DCRIC_DCINT3;

    ADC0_SSOP1_R |= ADC_SSOP1_S3DCOP | ADC_SSOP1_S2DCOP | ADC_SSOP1_S1DCOP | ADC_SSOP1_S0DCOP;      //// Sample 3,2,1,0 Digital Comparator
    ADC0_SSDC1_R |= (0x00000000<< ADC_SSDC1_S0DCSEL_S) | (0x00000001<<ADC_SSDC1_S1DCSEL_S) | (0x00000002<<ADC_SSDC1_S2DCSEL_S) | (0x00000003<<ADC_SSDC1_S3DCSEL_S);

    ADC0_DCCTL0_R |= ADC_DCCTL0_CIE | ADC_DCCTL0_CIC_HIGH | ADC_DCCTL1_CIM_ONCE;
    ADC0_DCCTL1_R |= ADC_DCCTL1_CIE | ADC_DCCTL1_CIC_HIGH | ADC_DCCTL1_CIM_ONCE;
    ADC0_DCCTL2_R |= ADC_DCCTL2_CIE | ADC_DCCTL2_CIC_HIGH | ADC_DCCTL2_CIM_ONCE;
    ADC0_DCCTL3_R |= ADC_DCCTL3_CIE | ADC_DCCTL3_CIC_HIGH | ADC_DCCTL3_CIM_ONCE;



    ADC0_DCCMP0_R |= (THRESHOLD_VALUE << 0) | VOLTAGE_REFERENCE_0 <<16;
    ADC0_DCCMP1_R |= (THRESHOLD_VALUE << 0) | VOLTAGE_REFERENCE_0 <<16;
    ADC0_DCCMP2_R |= (THRESHOLD_VALUE << 0) | VOLTAGE_REFERENCE_0 <<16;
    ADC0_DCCMP3_R |= (THRESHOLD_VALUE << 0) | VOLTAGE_REFERENCE_0 <<16;



    ADC0_IM_R |= ADC_IM_DCONSS1;
    enableNvicInterrupt(INT_ADC0SS1);

//    , ADCSSOP1,  ADCSSDC1, ADCDCCTL0,1,2,3 >>
//    ADCDCCMP1 = avg + thresh << 16
//    ADCDCCMP0 = avg-thresh

   // initperiodictimer1(4s)

}

DCisr()
{
    ADC0_DCISC_R |= ADC_DCISC_DCINT0 | ADC_DCISC_DCINT1 | ADC_DCISC_DCINT2 | ADC_DCISC_DCINT3;
    //ADC0_DCISC_R |= 1<<0 | 1<<1 | 1<<2 | 1<<3;
    TtimeBuffer = activeBuffer;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

}

timer1Isr()
{
    //ADC0_DCISC_R |= ADC_DCISC_DCINT0 | ADC_DCISC_DCINT1 | ADC_DCISC_DCINT2 | ADC_DCISC_DCINT3;
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
    UDMA_ENACLR_R |= (1<<25);


     index_ani0 = 0;
     index_ani1 = 0;
     index_ani2 = 0;
     index_ani3 = 0;

    if(TtimeBuffer == 1)
    {
        processBuffer(bufferA);
        processBuffer(bufferB);
    }
    else
    {
        processBuffer(bufferB);
        processBuffer(bufferA);
    }

    maxani1 = max_index(ani1);
    maxani2 = max_index(ani2);
    maxani3 = max_index(ani3);


//    T12 = findMaxCorrelationLag(ani1,ani2 , 10 );
//    T23 = findMaxCorrelationLag(ani2, ani3, 10 );
//    T13 = findMaxCorrelationLag(ani1, ani3, 10 );



    T12 = cross_correlation(ani1, ani2, 32, 5, result);
    T23 = cross_correlation(ani2, ani3, 32, 5, result);
    T13 = cross_correlation(ani1, ani3, 32, 5, result);





    if (T12 > T13) {
            if (T12 > T23) {
                //largest = T12;
                if (T13 > T23) {
                    angle = 120 + 1*T13 + 0.1*T13*T13;
                  //  middle = T13;
                    //smallest = T23;
                } else {
                    angle = 240 + 1*T13 + 0.1*T13*T13;
                    //middle = T23;
                    //smallest = T13;
                }
            } else {
                angle = 0 + 1*T12 + 0.1*T12*T12;
               // largest = T23;
                //middle = T12;
                //smallest = T13;
            }
        } else {
            if (T13 > T23) {
                //largest = T13;
                if (T12 > T23) {
                    angle = 120 + 1*T12 + 0.1*T12*T12;
                   // middle = T12;
                    //smallest = T23;
                } else {
                    angle = 0 + 1*T23 + 0.1*T23*T23;
                   // middle = T23;
                    //smallest = T12;
                }
            } else {
                angle = 240 + 1*T13 + 0.1*T13*T13;
                //largest = T23;
                //middle = T13;
                //smallest = T12;
            }
        }

    char str[80];
    snprintf(str,sizeof(str),"AOA:   %4"PRIu16"\n", angle);
    putsUart0(str);
    UDMA_ENASET_R |= (1<<25);
}
uint16_t cross_correlation(uint16_t *x, uint16_t *y, uint16_t n, uint16_t window_size, uint16_t *result) {


    uint16_t max_lag = n - window_size;
    // Initialize result array to zero
    uint16_t i;
    for (i = 0; i <= max_lag; i++) {
        result[i] = 0;
    }

    // Perform cross-correlation with sliding window
    uint16_t lag;
    for (lag = 0; lag <= max_lag; lag++) {
        uint16_t i;
        for (i = 0; i < window_size; i++) {
            result[lag] += x[i] * y[i + lag];
        }
    }

    // Finding the index of the maximum correlation value
    uint16_t max_index = 0; // Index of the maximum value
    uint16_t max_value = result[0]; // Initialize max_value to the first element of result

    for (i = 0; i <= max_lag; i++) {
        printf("Correlation at lag %d: %d\n", i, result[i]);
        if (result[i] > max_value) {
            max_value = result[i];
            max_index = i;
        }
    }
    printf("Maximum correlation value is %d at index %d.\n", max_value, max_index);

    return max_index;
}

uint16_t max_index(uint16_t *buffer)
{
    uint16_t i=0, buffermax;
    for(i = 0 ; i < 64; i++)
    {
        if(buffer[i] > buffermax )
        {
            buffermax = buffer[i];
            maxindex = i;
        }
    }
    return maxindex;
}

int findMaxCorrelationLag(uint16_t *signal1, uint16_t *signal2, int size) {
    int maxIndex = 0;
    int16_t maxCorr = 0;  // Start with zero since product sums will not be negative
    int lag;
    for (lag = 0; lag <= MAX_LAG; lag++) {  // Start from 0 to only consider positive lags
        int16_t sum = 0;
        uint16_t i;
        for (i = 0; i < size; i++) {
            int j = i + lag;
            if (j < size) {  // Ensure 'j' does not exceed the bounds of 'signal2'
                sum += (int16_t)signal1[i] * (int16_t)signal2[j];
            }
        }
        corrResults[lag] = sum;  // Store correlation value at current lag index

        // Find maximum correlation
        if (sum > maxCorr) {
            maxCorr = sum;
            maxIndex = lag;
        }
    }

    return maxIndex;
}



void processBuffer(volatile uint16_t *buffer)
{
    uint16_t i;
    for (i = 0; i < BUFFER_SIZE; i++)
    {
        switch (i % 4)
        {
            case 0:
                ani0[index_ani0++] = buffer[i]; // Data for ANI0
                break;
            case 1:
                ani1[index_ani1++] = buffer[i]; // Data for ANI1
                break;
            case 2:
                ani2[index_ani2++] = buffer[i]; // Data for ANI2
                break;
            case 3:
                ani3[index_ani3++] = buffer[i]; // Data for ANI3
                break;
        }
    }
}
void intDMA()
{
//    RCGCDMA
    SYSCTL_RCGCDMA_R |= SYSCTL_RCGCDMA_R0;
    _delay_cycles(10);

    UDMA_CFG_R |= UDMA_CFG_MASTEN;



    //UDMA_CHASGN_R = UDMA_CHASGN_SECONDARY;
    UDMA_CHMAP3_R = 1<<4;

    UDMA_PRIOCLR_R = UDMA_PRIOCLR_CLR_M;


    UDMA_ALTCLR_R = UDMA_ALTCLR_CLR_M;
    UDMA_USEBURSTCLR_R = (1<<25);
    UDMA_REQMASKCLR_R = UDMA_REQMASKCLR_CLR_M;


    UDMA_CTLBASE_R = (uint32_t)(&Base_arr[0]);



    UDMA_PSEP_R = ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x190 + 0x00));
    *UDMA_PSEP_R = (uint32_t)(&ADC1_SSFIFO1_R);

    UDMA_PDEP_R = ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x190 + 0x004));
    *UDMA_PDEP_R = (uint32_t)(&bufferA[63]);

    UDMA_PCW_R = ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x190 + 0x008));
    *UDMA_PCW_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_ARBSIZE_2 | UDMA_CHCTL_XFERSIZE<<4 | UDMA_CHCTL_XFERMODE_PINGPONG;



    UDMA_ASEP_R = ((volatile uint32_t *)((UDMA_CTLBASE_R + 0x390 + 0x00)));
    *UDMA_ASEP_R = (uint32_t)(&ADC1_SSFIFO1_R);

    UDMA_ADEP_R = ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x390 + 0x004));
    *UDMA_ADEP_R = (uint32_t)(&bufferB[63]);

    UDMA_ACW_R = ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x390 + 0x008));
    *UDMA_ACW_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_ARBSIZE_2 | UDMA_CHCTL_XFERSIZE<<4 | UDMA_CHCTL_XFERMODE_PINGPONG;


    ADC1_SSCTL1_R |= ADC_SSCTL1_IE3;
    ADC1_IM_R |= ADC_IM_MASK1;
    enableNvicInterrupt(INT_ADC1SS1);
    UDMA_ENASET_R |= (1<<25);

}
void initHwT()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);



    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_16_BIT;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for ONE SHOT MODE
    TIMER1_TAILR_R = 8000;                       // set load value to 40e6 for  92us interrupt rate

    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module

    enableNvicInterrupt(INT_TIMER1A);              // turn-on interrupt 37 (TIMER1A) in NVIC

}
/**
 * main.c
 */

void DMAISR()
{

    UDMA_CHIS_R |= (1<<25);
    ADC1_ISC_R |= ADC_ISC_IN1;

    udmaindex = udmaindex + 4;

    if((*UDMA_PCW_R & UDMA_CHCTL_XFERMODE_M) == 0)
    {
        *UDMA_PCW_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_ARBSIZE_2 | (63<<4) | UDMA_CHCTL_XFERMODE_PINGPONG;
        activeBuffer = 1;
        udmaindex = 0;
    }

    if ((*UDMA_ACW_R & UDMA_CHCTL_XFERMODE_M) == 0)
    {
        *UDMA_ACW_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_ARBSIZE_2 | (63<<4) | UDMA_CHCTL_XFERMODE_PINGPONG;
        activeBuffer = 0;
        UDMA_ENASET_R |= (1<<25);
        udmaindex = 0;
    }


}

int main(void)
{
    intHW();

    initUart0();
    setUart0BaudRate(115200,40e6);

    initAdc0Ss1();
    initAdc1Ss1();

    MY_setAdc0Ss1Mux();
    MY_setAdc1Ss1Mux();
    initHwT();
    intDC();
    intDMA();
    UDMA_ENASET_R |= (1<<25);
    USER_DATA data;

//    putsUart0("enter text:\n");
    // Setup UART0 baudrate
    while (true){
        if (kbhitUart0() == 1){
            getsUart0(&data);
            parseFields(&data);
        if(isCommand(&data,"AOA",0))
        {
            char str[80];
            snprintf(str,sizeof(str),"AOA:   %4"PRIu16"\n", angle);
            putsUart0(str);
        }
        }
    }

}
void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    char c;

    while (true)
    {
        c = getcUart0();

        if (c == 8 || c == 127) //8 or 127 for backspace
        {
            if (count > 0)
            {
                count--;
            }
        }
        else if (c == 13) //13 for enter
        {
            data->buffer[count] = '\0';
            return;
        }
        else if (c >= 32) //32 for character
        {
            if (count < MAX_CHARS)
            {
                data->buffer[count] = c;
                count++;
            }
            else
            {
                data->buffer[count] = '\0';
                return;
            }
        }
    }
}
