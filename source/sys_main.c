
/**
 *
 *  @brief this file contains example command sequences and functions to interface with the bq76PL455A-Q1 from a
 *  microcontroller. The examples provided are described in the bq76PL455A-Q1 Software Design Reference (SLVA617A)
 *  and the sections that correlate to each example are noted in the comments.
 *
 *  This code was written for the TMS570LS04x Launchpad Board, modified to remove R8, allowing use of the SCI1 UART.
 *  A bq76PL455A-Q1 EVM single-ended communication interface is connected to the Boosterpack connectors J1 and J2 as
 *  shown below. Connection of these boards must be made by the user.
 *
 *  J1 pin 1 (+3V3)     -> bq76PL455A-Q1 EVM J3 pin 3 (VIO)
 *  J1 pin 3 (SCI1_RX)  -> bq76PL455A-Q1 EVM J3 pin 5 (TX) **remove R8 from TMS570LS04x Launchpad
 *  J1 pin 4 (SCI1_TX)  -> bq76PL455A-Q1 EVM J3 pin 4 (RX)
 *  J2 pin 1 (GND)      -> bq76PL455A-Q1 EVM J3 pin 1 (DGND)
 *  J2 pin 3 (GIOA0)    -> bq76PL455A-Q1 EVM J3 pin 6 (nWAKE)
 *  J2 pin 4 (GIOA1)    -> bq76PL455A-Q1 EVM J3 pin 2 (nFAULT)
 *
 *
 */



/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"
#include "system.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>


/* USER CODE BEGIN (1) */
#include "gio.h"
#include "sci.h"
#include "rti.h"
#include "sys_vim.h"
#include "swi_util.h"

#include "pl455.h"

#define nDev_ID 0

int UART_RX_RDY = 0;
int RTI_TIMEOUT = 0;


//Number of cells connected in Series and Parallel
#define noSCell 7
#define noPCell 1

#define UV 3.00
#define OV 4.20

// Coeffients for SOC vs OCV
#define p1 -1.061
#define p2 14.05
#define p3 -68.49
#define p4 146.1
#define p5 -115.3


float deltaT=0;
int packState =0;


int T0 = 298;
int beta = 4356;
int R0 = 25000;
int Rf = 10000;
float Vf = 5.30;
float eta = 0.99;
float Q = 2.5*3600;


struct cell{
    float voltage;
    float current;
    float z;
    float temp;
};

void onContactor(void){
    if(packState ==0){
        gioSetBit(gioPORTA, 6, 1); //-ve on
        printf("Negative Contactor  on \n");
        gioSetBit(gioPORTA, 7, 1); //pre-charge on
        printf("Precharge Contactor on \n");
        delayms(1000); //1 sec delay
        gioSetBit(gioPORTA, 4, 1); //+ve on
        printf("Positive Contactor on\n");
        gioSetBit(gioPORTA, 7, 0); //pre-charge off
        printf("Precharge off \n");
        packState = 1;
    }
}

void offContactor(void){
    if(packState ==1){
        gioSetBit(gioPORTA, 4, 0); //+ve off
        printf("Positive Contactor off\n");
        gioSetBit(gioPORTA, 6, 0); //-ve off
        printf("Negative Contactor off\n");
        packState =0;
    }
}

// bframe one byte header then cell voltage, aux channel values, die temperature and vpack.

float getCellVoltage(BYTE *bFrame, struct cell c[]){
    int i, nSent;
    float packV = 0;
    for(i=1; i<=noSCell; i++){
        c[noSCell-i].voltage = (bFrame[i*2-1]<<8|bFrame[i*2]) * 0.000076295;
        packV+=c[noSCell-i].voltage;
// for protection of cell against over and under voltage
        if(c[noSCell-i].voltage < UV || c[noSCell-i].voltage > OV ){
            offContactor();
            nSent = WriteReg(nDev_ID, 12, 0x40, 1, FRMWRT_SGL_NR);  // send out broadcast pwrdown command
        }
    }
    return packV;
}


void showdata(struct cell c[],float packV, float packI, float zAvg){
   int i;
   printf("\n");
   for(i = 0; i < noSCell; i++){
        printf("Cell %d & %d Voltage= %f,\tCell %d & %d SOC= %f,\n",i+1,i+8,c[i].voltage,i+1,i+8,c[i].z*100);
    }
    printf("  \n");
}


float getCellCurrent(BYTE *bFrame, struct cell c[],float iref){
    int i;
    float pack_I;
    pack_I = ((bFrame[15]<<8|bFrame[16])*0.000076295-iref)*40;
    for(i = 0; i<noSCell; i++){
        c[i].current = (pack_I/2);
    }
    return(pack_I);
}

float getCellCurrentRef(BYTE *bFrame){
    int i;
    float pack_Iref;
    pack_Iref = (bFrame[15]<<8|bFrame[16])*0.000076295;
    return(pack_Iref);
}


float getCellTemp(BYTE *bFrame, struct cell c[]){
    int i;
    float v, j;
    float avt = 0;
    for(i = 1; i<=7; i++){
        v  = ((bFrame[15+2*i]<<8|bFrame[16+2*i])*0.000076295);
        j = log(Rf*v/(R0*(Vf-v)));
        c[7-i].temp = T0*beta/(beta + T0*j) - 273.15 ;
        c[0].temp = c[1].temp;
        avt+=c[7-i].temp;
    }
    //printf("Pack Temp. %f",avt/7);
    return avt/noSCell;
}

void printTemp(struct cell c[]){
    int i;
    for(i = 0; i<7; i++){
        printf("Temp.%d = %f ",(i+1), c[i].temp);
    }
    printf("\n");
}

float getInitialSOC(struct cell c[]){
    int i;
    float zAv=0;
    for(i = 0; i< noSCell; i++){
//        c[i].z =  p1*x^4 + p2*x^3 + p3*x^2 + p4*x + p5
        c[i].z = p1*pow(c[i].voltage,4) + p2*pow(c[i].voltage,3) + p3*pow(c[i].voltage,2) + p4*c[i].voltage + p5;
        zAv+=c[i].z;
    }
    return zAv/noSCell;
}


float calculateCellSOC(struct cell c[], float deltaT){
    int i;
    float zAv=0;
    for(i = 0; i< noSCell; i++){
        c[i].z += ((c[i].current*deltaT*eta)/Q);
        zAv += c[i].z;
    }
    return zAv/noSCell;
}

void printSOC(struct cell c[]){
    int i;
    for(i = 0; i<noSCell; i++){
        printf("Z_%d = %f  ",(noSCell-1 - i), c[i].z*100);
    }
    printf("\n");
}


/* USER CODE END */


/* USER CODE BEGIN (2) */
/* USER CODE END */


void main(void)
{
/* USER CODE BEGIN (3) */
    systemInit();
    gioInit();
    sciInit();
    sciSetBaudrate(scilinREG, BAUDRATE);
    rtiInit();
    vimInit();

    _enable_IRQ();

//    WakePL455();

    CommClear();

    CommReset();
    gioSetDirection(gioPORTA, 0xD7); // pin Direction (7,6,4,2,1,0 as output)


// initialize local variables
    int nSent, nRead, nTopFound = 0;
    BYTE  bFrame[132];
    uint32  wTemp = 0;


    struct cell c[7];

    float zAvg;
    float vPack;
    float iPack,avt;
    int i,m,j;
    int cycles = 0;

    // Wake devices ID 0
    // The wake tone will awaken any device that is already in shutdown and the pwrdown will shutdown any device
    // that is already awake. The least number of times to sequence wake and pwrdown will be half the number of
    // boards to cover the worst case combination of boards already awake or shutdown.
    nSent = WriteReg(nDev_ID, 12, 0x40, 1, FRMWRT_SGL_NR);  // send out broadcast pwrdown command
    delayms(5); //~5ms
    WakePL455();
    delayms(5); //~5ms



    // Mask Customer Checksum Fault bit
    nSent = WriteReg(nDev_ID, 107, 0x8000, 2, FRMWRT_SGL_NR); // clear all fault summary flags

    // Clear all faults
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);      // clear all fault summary flags
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR); // clear fault flags in the system status register

    // Set addresses for all boards in daisy-chain (section 1.2.3)
    nSent = WriteReg(nDev_ID, 10, nDev_ID, 1, FRMWRT_SGL_NR); // send address to each board

    // Enable all communication interfaces on all boards in the stack (section 1.2.1)
    nSent = WriteReg(nDev_ID, 16, 0x10F8, 2, FRMWRT_SGL_NR);  // set communications baud rate and enable all interfaces on all boards in stack

    nSent = WriteReg(nDev_ID, 16, 0x1020, 2, FRMWRT_SGL_NR);    // Disable High side receiver on differential (1.2.5)

    nSent = WriteReg(nDev_ID, 16, 0x10C0, 2, FRMWRT_SGL_NR);    // Disable High side receiver on differential (1.2.6)


    // Clear all faults (section 1.2.7)
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_ALL_NR); // clear all fault summary flags
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_ALL_NR); // clear fault flags in the system status register

    delayms(10);

//    // Configure AFE best setting
//    nSent = WriteReg(0, 61, 0x00, 1, FRMWRT_ALL_NR); // set 0 initial delay
//    // Configure cell voltage and internal temp sample period
//    nSent = WriteReg(0, 62, 0xBC, 1, FRMWRT_ALL_NR); // set 60us cell and 100us temp ADC sample period
//    // Configure AUX voltage sample period AUX0-5 are external thermistor, AUX6 is current sense
//    nSent = WriteReg(0, 63, 0x44444444, 4, FRMWRT_ALL_NR); // set 12.6us AUX sample period
//    // Configure the oversampling rate
//    nSent = WriteReg(0, 7, 0x7B, 1, FRMWRT_ALL_NR); // set 8x oversampling, stay on channel for oversample and 12.6us oversample period for cell and AUX
//    // Set AFE_PCTL
//    nSent = WriteReg(0, 15, 0x80, 1, FRMWRT_ALL_NR); // set AFE_PCTL bit to on (only enable AFE when sampling)


        // Configure AFE to fastest setting
        nSent = WriteReg(0, 61, 0x00, 1, FRMWRT_ALL_NR); // set 0 initial delay
        // Configure cell voltage and internal temp sample period
        nSent = WriteReg(0, 62, 0x04, 1, FRMWRT_ALL_NR); // set 4.13us cell and 12.6us temp ADC sample period
        // Configure AUX voltage sample period AUX0-5 are external thermistor, AUX6 is current sense
        nSent = WriteReg(0, 63, 0x44444444, 4, FRMWRT_ALL_NR); // set 12.6us AUX sample period
        // Configure the oversampling rate
        nSent = WriteReg(0, 7, 0x00, 1, FRMWRT_ALL_NR); // set 0 oversampling
        // Set AFE_PCTL
        nSent = WriteReg(0, 15, 0x80, 1, FRMWRT_ALL_NR); // set AFE_PCTL bit to on (only enable AFE when sampling)

    // Clear and check faults (section 2.2.4)
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR); // clear fault flags in the system status register
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR); // clear all fault summary flags

    // Select number of cells and channels to sample (section 2.2.5.1)
    nSent = WriteReg(nDev_ID, 13, 0x07, 1, FRMWRT_SGL_NR); // set number of cells to 7
    nSent = WriteReg(nDev_ID, 3, 0x007FFF42, 4, FRMWRT_SGL_NR); // select 7 cell,8 AUX channels, internal analog die temperature and vpack
    nSent = WriteReg(0, 67, 0x4000, 2, FRMWRT_ALL_NR); // set vm sampling period
    nSent = WriteReg(0, 30, 0x0001, 2, FRMWRT_ALL_NR);//enable vm
    // Set cell over-voltage and cell under-voltage thresholds on a single board (section 2.2.6.1)
    nSent = WriteReg(nDev_ID, 144, 0xD70A, 2, FRMWRT_SGL_NR); // set OV threshold = 4.2000V
    nSent = WriteReg(nDev_ID, 142, 0x8CCD, 2, FRMWRT_SGL_NR); // set UV threshold = 2.75000V

    // Configure GPIO pin direction and set new pin values (section 5.2.1)
    nSent = WriteReg(nDev_ID, 123, 0x00, 1, FRMWRT_SGL_NR); // turn off all GPIO pull downs
    nSent = WriteReg(nDev_ID, 122, 0x00, 1, FRMWRT_SGL_NR); // turn off all GPIO pull ups
    nSent = WriteReg(nDev_ID, 120, 0x3F, 1, FRMWRT_SGL_NR); // set GPIO[5:0] to output direction
    nSent = WriteReg(nDev_ID, 122, 0x3F, 1, FRMWRT_SGL_NR); // turn on all GPIO pull ups
    nSent = WriteReg(nDev_ID, 121, 0x00, 1, FRMWRT_SGL_NR); // set GPIO outputs (as 0)


    nSent = WriteReg(nDev_ID, 2, 0x01, 1, FRMWRT_SGL_R); // send sync sample command
    nSent = WaitRespFrame(bFrame, 37, 0); // 16(8 cell voltage) + 4(temp)  bytes data + packet header + CRC, 0ms timeout

    float iref;
    printf("Battery Health Management System for Lithium-ion battery.\n");
    printf("\n");
    printf("Battery State at No Load:\n");
    printf("\n");

    vPack = getCellVoltage(bFrame,c);
    iref = getCellCurrentRef(bFrame);
    iPack = getCellCurrent(bFrame, c,iref);
    zAvg = getInitialSOC(c);
    printf("Pack Voltage:  %f\n",vPack);
    printf("Pack Current:  %f\n",iPack);
    printf("Pack SOC: %f\n",zAvg*100);
    getCellTemp(bFrame,c);
    printf("\n");

    showdata(c,vPack, iPack, zAvg);
    printTemp(c);

    // printf("Enter no. of cycles for which battery is connected across load");
     printf("Enter no. of cycles for which contactor will be on ");

    scanf("%d",&cycles);

    onContactor();
    delayus(10);
    printf("\n");

    printf("Battery Pack Status: Discharge mode\n\n");
    int g = 0;
    float vavg;


    while(1){
        gioToggleBit(gioPORTA,2);
        WakePL455();
        delayms(5); //~5ms

        nSent = WriteReg(nDev_ID, 20, 0, 2, FRMWRT_SGL_NR); //Balancing
        nSent = WriteReg(nDev_ID, 2, 0x01, 1, FRMWRT_SGL_R); // send sync sample command
        nSent = WaitRespFrame(bFrame, 37, 0); //1 header+  14(7 cell voltage) + 16(AUX)bytes data + 2 Analog die + 2 Vpack + 2 CRC
        vPack = getCellVoltage(bFrame,c);
        vavg=vPack/7;
        iPack = getCellCurrent(bFrame, c,iref);

    int i, j=0,nSent;
    if(iPack > 0.2){
        for(i = 0; i< noSCell; i++){
            if(c[i].z-zAvg > 0.05){ //check % or fractional value
                j = j|1<<i;
        c[i].current = c[i].current - c[i].voltage/75;
            }
        }

       // Mask Customer Checksum Fault bit
        nSent = WriteReg(nDev_ID, 107, 0x8000, 2, FRMWRT_SGL_NR); // clear all fault summary flags

       // Clear all faults
        nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);      // clear all fault summary flags
        nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR); // clear fault flags in the system status register
        nSent = WriteReg(nDev_ID, 20, j, 2, FRMWRT_SGL_NR); //Balancing

    }
        zAvg = calculateCellSOC(c, deltaT);
        printf("Pack Voltage  %f\n",vPack);
        printf("Pack Current  %f\n",iPack);
        // printf("Pack Temp %f",avt);
        printf("Pack SOC %f\n",zAvg*100);
        getCellTemp(bFrame,c);
        printf("\n");
        showdata(c,vPack, iPack, zAvg);
        printTemp(c);

        float vt,atemp,vp;
        vt = (bFrame[31]<<8|bFrame[32]) * 0.000076295;
        atemp=(vt-1.8078)*147.514;
        vp = (((bFrame[33]<<8|bFrame[34]) * 0.000076295)*25);
        printf("vp=%f\n",vp);
        printf("Die Temperature=%f\n",atemp);
        printf("\n");
        printf("\n");
        if(g==cycles){
            offContactor();
            break;
        }
        g++;

        deltaT = 1; // deltaT = 1 sec

    }

/* USER CODE END */
}

/* USER CODE BEGIN (4) */
/* USER CODE END */
