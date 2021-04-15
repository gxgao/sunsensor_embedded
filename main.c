#include <msp430.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/**
 * main.c
 */

volatile unsigned char TXData = 0;
volatile unsigned char TXData_slave = 0;
unsigned char dataBuffer[30];
static const char finalArrayLength = 50;
unsigned char finalArray[50];

volatile unsigned char RXData = 0;
volatile unsigned char slaveCmd = 0;
double READING ;


void init(unsigned char BIT);
void initAll();
void sendZebra1(unsigned char BIT);
void setThreshold(unsigned char bit, unsigned char input);
void setThreshold2(unsigned char bit, unsigned char input);
double determineAngle();
void sendReadings() ;
unsigned char countSetBits(unsigned char byte);

void readAll(unsigned char BIT);
void readXaxis();
void readYaxis();
//Case Parameters are defined here
double slitHeight = 14.7;
float slitWidth = 0;

unsigned char ourBit;

double angleX = 0.0;
double angleY = 0.0;

short MSB = 0;
short LSB = 0;

short numOfBits = 408  ;//(finalarraylength + 1)* 8

typedef enum SPI_ModeEnum{
    IDLE_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
} SPI_Mode;

SPI_Mode state1 = IDLE_MODE;


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //CS pin picking

    //P3.7 : CS1 , P3.5 CS4, P2.4 CS5, P3.1 CS0 , P2.0 CS3, P2.2 CS2
    P3DIR |= BIT7 | BIT5 | BIT1; // Setting the CS pins to output mode by making the corresponding bit 1
    P2DIR |= BIT4 | BIT0 | BIT2; //These are off

    P3OUT |= BIT7 | BIT5 | BIT1; // Deselecting their outputs turns the CS inactive
    P2OUT |= BIT4 | BIT0 | BIT2;

    //P3.0 FR0, P2.1 FR3, P3.2 FR2, P3.6 FR1, P2.5 FR4, P2.3 FR5;
    P3DIR &= ~(BIT0 | BIT2 | BIT6);  // Setting the frame ready ports to input mode)
    P2DIR &= ~(BIT1 | BIT5 | BIT3);
    P3REN &= ~(BIT0 | BIT2 | BIT6);  // disable internal pullup/pulldown resistors
    P2REN &= ~(BIT1 | BIT5 | BIT3);

    P1SEL = BIT5 + BIT6 + BIT7;   // Select pin option to be MISO, MOSI, and SCLK
    P1SEL2 = BIT5 + BIT6 + BIT7;



    UCB0CTL1 = UCSWRST;                       // **Put state machine in reset**
    UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
    UCB0CTL1 |= UCSSEL_2;                     // SMCLK
    UCB0BR0 |= 0x01;                          // /2
    UCB0BR1 = 0;                              //
    //UCB0MCTL = 0;                             // No modulation
    UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCB0RXIE;                          // Enable USCI0 RX interrupt

    // NEED THIS CODE CHECKED info from pg 445 of msp 430g2553  https://www.ti.com/lit/ug/slau144j/slau144j.pdf
/***
    P1SEL = BIT2 + BIT1 + BIT4; // BIT 2 is SIMO , BIT1 is SOMI , bit4 is USCIa0 clk input/output
    P1SEL2 = BIT1 + BIT2 + BIT4;


    UCA0CTL1 = UCSWRST;                       // **Put state machine in reset**
    UCA0CTL0 |= UCCKPL + UCMSB + UCSYNC;  // 3-pin, 8-bit SPI master MSB first big endian , no UCMST so slave mode


    //UCA0MCTL = 0;                             // No modulation
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI0 RX interrupt
***/


    __bis_SR_register(GIE); // Enable interrupts

    //slaveCmd = UCA0RXBUF ;

    initAll();
    __no_operation();
    while(1){
        sendIntegration(4);
        readXaxis();
        __no_operation();
        angleX = determineAngle();
        __no_operation();

        readYaxis();
        __no_operation();
        angleY = determineAngle();
        __no_operation();


    }
   return 0;
}

void init(unsigned char CS){
    state1 = TX_DATA_MODE;
    unsigned char BIT;
    switch(CS){
         case 0:
             BIT = BIT1;
             break;
         case 1:
             BIT = BIT7;
             break;
         case 2:
             BIT = BIT2;
             break;
         case 3:
             BIT = BIT0;
             break;
         case 4:
             BIT = BIT5;
             break;
         case 5:
             BIT = BIT4;
             break;

     }
    if(CS == 3 || CS == 2 || CS == 5){
        P2OUT &= ~BIT;
    }
    else{
        P3OUT &= ~BIT;
    }
    TXData = 0xF0;

    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    if(CS == 3 || CS == 2 || CS == 5){
        P2OUT |= BIT;
    }
    else{
        P3OUT |= BIT; //raises it back high, disabling this sensor
    }
    state1 = IDLE_MODE;
}

void initAll(){
    init(3);
    __no_operation();
    init(4);
    init(5);

    init(0);
    init(1);
    init(2);
}




void sendZebra1(unsigned char BIT){
    state1 = TX_DATA_MODE;
    P3OUT &= ~BIT;  //testing CS1
    TXData = 0xE8;

    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[0] = RXData;
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[1] = RXData;
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[2] = RXData;
    P3OUT |= BIT; //raises it back high, disabling this sensor
    state1 = IDLE_MODE;
}

void sendZebra2(unsigned char BIT){
    state1 = TX_DATA_MODE;
    P3OUT &= ~BIT;  //testing CS1
    TXData = 0xE4;

    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[0] = RXData;
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[1] = RXData;
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[2] = RXData;
    P3OUT |= BIT; //raises it back high, disabling this sensor
    state1 = IDLE_MODE;

}


void sendZebra0(unsigned char BIT){
    state1 = TX_DATA_MODE;
    P3OUT &= ~BIT;  //testing CS1
    TXData = 0xE2;

    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[0] = RXData;
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[1] = RXData;
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    dataBuffer[2] = RXData;
    P3OUT |= BIT; //raises it back high, disabling this sensor
    state1 = IDLE_MODE;

}

void sendIntegration(unsigned char CS){
      state1 = TX_DATA_MODE;
      unsigned char BIT = 0;
      switch(CS){
          case 0:
              BIT = BIT1;
              break;
          case 1:
              BIT = BIT7;
              break ;
          case 2:
              BIT = BIT2;
              break;
          case 3:
              BIT = BIT0;
              break;
          case 4:
              BIT = BIT5;
              break;
          case 5:
              BIT = BIT4;
              break;

      }
      if(CS == 3 || CS == 2 || CS == 5){
          P2OUT &= ~BIT;
      }
      else{
          P3OUT &= ~BIT;  //testing CS1
      }
      TXData = 0xB8; // Command to signify reading

      IE2 |= UCB0TXIE;
      while (!(IFG2 & UCB0TXIFG));
      dataBuffer[0] = RXData;
      TXData = 0xFF;
      IE2 |= UCB0TXIE;
      while (!(IFG2 & UCB0TXIFG));
      dataBuffer[1] = RXData;
      TXData = 0xFF;
      IE2 |= UCB0TXIE;
      while (!(IFG2 & UCB0TXIFG));
      dataBuffer[2] = RXData;
      // Need to wait for FR to go back high
      if(CS == 3 || CS == 2 || CS == 5){
          P2OUT |= BIT; //raises it back high, disabling this sensor
      }
      else{
          P3OUT |= BIT;
      }
      state1 = IDLE_MODE;
}


void readXaxis(){

    setThreshold(3, 16);
    __no_operation();

    //sendZebra1(BIT0);
    sendIntegration(3);
    __no_operation();
    while(1){
        //int toCheck = P2IN & corresponding FR bit;
        //WE want to break loop if P2In is high 1, so when we and it with bit, we get 1.

        __no_operation();
        if (P2IN & BIT1){ // wait for FR to go back high
            __no_operation();
            break;
        }

    }

    __no_operation();

    readAll(3); //buffer is ready
    __no_operation();
    unsigned short count = 0;
    unsigned char i = 9;
    //get rid of permanent 1 bit
    dataBuffer[i] = dataBuffer[i] & 0x7F;
    for(i; i < 27; i++){ //dumping buffer values into our final array 9 - 26 are usuful MSB of index 9 is always 1 LSB of index 26 is always 0
        finalArray[count] = dataBuffer[i];
        count+=1;
    }

    __no_operation();
    //CS4 start here
    __no_operation();
    setThreshold(4, 16);

    sendIntegration(4);
    __no_operation();
    while(1){
       __no_operation();
       if (P2IN & BIT5){ // wait for FR to go back high
           __no_operation();
           break;
       }
    }

    __no_operation();
    readAll(4);
    count = 16; //go back one to account for overlap
    i = 9;
    for(i; i < 27; i++){ //dumping buffer values into our final array
       if(count == 16){

           //get rid of permenant 1;
           unsigned char MSB = finalArray[count] >> 7 & 1;
           unsigned char ourData = dataBuffer[i] ;
           if(!MSB){
                      //if third MSB is 0 we need to set our permenant 1 to 0
                      ourData = ourData & 0x7F;
           }
           finalArray[count] = finalArray[count] | ourData;
           count += 1;

       }
       else if( count == 17){
           finalArray[count] = finalArray[count] | dataBuffer[i]; //Or the overlap
           count+=1;

       }
       else{
           finalArray[count] = dataBuffer[i] ;
           count+=1;
       }

    }

    setThreshold(5, 16);
    sendIntegration(5);
    while(1){
      if (P2IN & BIT3){ // wait for FR to go back high
          break;
      }
    }
    readAll(5);
    // our byte array is 17 long so 16 + 17 - 1 (-1 for offset)
    count = 32;
    i = 9;
    __no_operation();
    for(i; i < 27; i++){ //dumping buffer values into our final array
      if(count == 32){
           unsigned char MSB = finalArray[count] >> 7 & 1;
           unsigned char ourData = dataBuffer[i] ;
           if(!MSB){
                  //if third MSB is 0 we need to set our permenant 1 to 0
                  ourData = ourData & 0x7F;
           }
           finalArray[count] = finalArray[count] | ourData;
           count += 1;

      }
      else if(count == 33){
          finalArray[count] = finalArray[count] | dataBuffer[i]; //Or the overlap
          count += 1;
      }
      else{
          finalArray[count] = dataBuffer[i];
          count+=1;
      }
    }

    __no_operation();

}

void readYaxis(){

    //unsigned char finalArray[50]; //final array with values

    setThreshold(0, 16);
    __no_operation();

    //sendZebra1(BIT0);
    sendIntegration(0);
    __no_operation();
    while(1){
        __no_operation();
        if (P3IN & BIT0){ // wait for FR to go back high
            __no_operation();
            break;
        }
    }

    __no_operation();

    readAll(0); //buffer is ready
    __no_operation();
    unsigned short count = 0;
    unsigned char i = 9;
    for(i; i < 27; i++){ //dumping buffer values into our final array
        finalArray[count] = dataBuffer[i];
        count+=1;
    }

    __no_operation();
    //CS4 start here
    __no_operation();
    setThreshold(1, 16);

    sendIntegration(1);
    __no_operation();
    while(1){
       __no_operation();
       if (P3IN & BIT6){ // wait for FR to go back high
           __no_operation();
           break;
       }
    }

    __no_operation();
    readAll(1); //TODO Note 1 is flipped on the board (to make the wiring work)
    count = 16; //count should end at 17, need to go back one to account for overlap
    i = 26;
    for(i; i >= 9; i--){ //dumping buffer values into our final array
       if(count == 16){
           //get rid of permenant 1;
           unsigned char MSB = finalArray[count] >> 7 & 1;
           unsigned char ourData = dataBuffer[i] ;
           if(!MSB){
                      //if third MSB is 0 we need to set our permenant 1 to 0
                      ourData = ourData & 0x7F;
           }
           finalArray[count] = finalArray[count] | ourData;
           count += 1;

       }
       else if( count == 17){
           finalArray[count] = finalArray[count] | dataBuffer[i]; //Or the overlap
           count+=1;

       }
       else{
           finalArray[count] = dataBuffer[i] ;
           count+=1;
       }

    }
    __no_operation();
    setThreshold(2, 16);

    sendIntegration(2);
    __no_operation();
    while(1){
        __no_operation();
      if (P3IN & BIT2){ // wait for FR to go back high
          __no_operation();
          break;
      }
    }

    __no_operation();
    readAll(2);
    count = 32;
   i = 9;
   for(i; i < 27; i++){ //dumping buffer values into our final array
     if(count == 32){
          unsigned char MSB = finalArray[count] >> 7 & 1;
          unsigned char ourData = dataBuffer[i] ;
          if(!MSB){
                 //if third MSB is 0 we need to set our permenant 1 to 0
                 ourData = ourData & 0x7F;
          }
          finalArray[count] = finalArray[count] | ourData;
          count += 1;

     }
     else if(count == 33){
         finalArray[count] = finalArray[count] | dataBuffer[i]; //Or the overlap
         count += 1;
     }
     else{
         finalArray[count] = dataBuffer[i];
         count+=1;
     }
   }

    __no_operation();

}

unsigned char countSetBits(unsigned char byte){
    unsigned char i = 1 ;
    unsigned char count = 0;
    if( byte & 1 ) count += 1;
    for(i; i < 8; i++) {
       if ((byte >> i ) & 1 ) count += 1;

    }
    return count ;
}

//axis = 0 is x axis = 1 is y
double determineAngle(){
    //go from left, go from right and find the MostSetBit and LeastSetBit

    unsigned char MSBSet = 0;

    unsigned char LSBSet = 0;
    int i = 0;
    int count = 0 ;
    unsigned char* arrayToWorkWith = finalArray;
    unsigned char foundEnough = 7;
    unsigned char clusterCnt = 0;
    short range = 10;
    short startCluster = 0;
    for(i; i < finalArrayLength; i++){
        unsigned char toWorkWith = arrayToWorkWith[i];
        int offSet = 7;
        while(offSet > 0){
            unsigned char bitToWorkWith = toWorkWith >> offSet & 1;

            // if we got a one and we haven't set MSB yet
            if(!MSBSet){
                //If we have a 1 and we hanv'et started a cluster yet
                if(bitToWorkWith && !clusterCnt){
                     clusterCnt += 1;
                     range = count + (10<numOfBits-count?10:numOfBits-count); // we must find a few more ones within this range for it to not be noise
                     startCluster = count;
                }
                else if(bitToWorkWith && clusterCnt && count < range ){
                    clusterCnt += 1;
                }
                if(clusterCnt >= foundEnough){
                    //if we found enough 1's within the range
                    MSBSet = 1;
                    MSB = startCluster;
                    break;
                }
                if(count > range){
                    clusterCnt = 0;
                    range = 0;
                    startCluster = 0;
                }
            }

            offSet -= 1;
            count += 1;
        }
        if(MSBSet){
            break;
        }
    }
    // Look for LSB Now
    count = numOfBits ; // Last pixel should be numOfBits
    i = finalArrayLength;
    clusterCnt = 0;
    range = 10;
    startCluster = numOfBits;
    //THIS can def be optimized later
    for(i; i >=0; i--){
        unsigned char toWorkWith = arrayToWorkWith[i];
        //go backwards now
        if(!LSBSet){
            if (toWorkWith&1 && !clusterCnt){
                startCluster = count;
                clusterCnt += 1;
                range = (count - (10 < count?10:count)); // if we hit the end this will account for it
            }
            else if (toWorkWith&1 && clusterCnt){
               clusterCnt += 1;
           }
           if(clusterCnt >= foundEnough){
                  //if we found enough 1's within the range
                  LSBSet = 1;
                  LSB = startCluster;
                  break;
           }
           if(count < range){
               clusterCnt = 0 ;
               startCluster = numOfBits;
               range = 0;
           }
            count -= 1;
            int offSet = 1;

            while(offSet < 8){

                unsigned char bitToWorkWith = toWorkWith >> offSet & 1;
                // if we got a one and we haven't started our cluster
                if (bitToWorkWith && !clusterCnt){
                    startCluster = count;
                    clusterCnt += 1;
                    range = (count - (10 < count?10:count)); // if we hit the end this will account for it
                }

                else if (bitToWorkWith && clusterCnt){

                    clusterCnt += 1;

                }
                if(clusterCnt >= foundEnough){
                       //if we found enough 1's within the range
                       LSBSet = 1;
                       LSB = startCluster;
                       break;
                }
                if(count < range){
                    clusterCnt = 0 ;
                    startCluster = numOfBits;
                    range = 0;
                }
                offSet+=1;
                count -= 1;
            }
            if(LSBSet){
                break;
            }
        }
     }
    //calculating angle = tan-1(d/h) h and d are presets
    double avgPos = (MSB + LSB)/2.0 ;
    // Sensors are 7.1 MM Optical array. 32 pixel overlap. So 7.1 * 3 - (32/144 * 7.1) = 19.72 mm for total array
    //midpoint is 200 for angle 0
    double d = (avgPos-(numOfBits/2.0)) * 23.216/numOfBits;
    double angle = atan(d/slitHeight);
    READING = angle;
    return angle ;
}


void setThreshold(unsigned char CS, unsigned char input){
    state1 = TX_DATA_MODE;
    unsigned char BIT;
    switch(CS){
         case 0:
             BIT = BIT1;
             break;
         case 1:
             BIT = BIT7;
             break;
         case 2:
             BIT = BIT2;
             break;
         case 3:
             BIT = BIT0;
             break;
         case 4:
             BIT = BIT5;
             break;
         case 5:
             BIT = BIT4;
             break;
     }
    if(CS == 3 || CS == 2 || CS == 5){
        P2OUT &= ~BIT;
    }
    else{
        P3OUT &= ~BIT;  //testing CS of Bit
    }
    TXData = 0xCC;
    //TXData = 0x80;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    TXData = input << 4;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    TXData = 0x00;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));
    if(CS == 3 || CS == 2 || CS == 5){
        P2OUT |= BIT;
    }
    else{
        P3OUT |= BIT; //raises it back high, disabling this sensor
    }
    state1 = IDLE_MODE;

}



void readAll(unsigned char CS){
    state1 = TX_DATA_MODE;
    unsigned char BIT;
    switch(CS){
         case 0:
             BIT = BIT1;
             break;
         case 1:
             BIT = BIT7;
             break;
         case 2:
             BIT = BIT2;
             break;
         case 3:
             BIT = BIT0;
             break;
         case 4:
             BIT = BIT5;
             break;
         case 5:
             BIT = BIT4;
             break;

    }
    if(CS == 3 || CS == 2 || CS == 5){
        P2OUT &= ~BIT;
    }
    else{
        P3OUT &= ~BIT;  //testing CS of BIT
    }

    TXData = 0x9C;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));//{
        //__no_operation();
    //}
    dataBuffer[0] = RXData;
    TXData = 2;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));//{
      //  __no_operation();
    //}
    dataBuffer[1] = RXData;
    TXData = 143;
    IE2 |= UCB0TXIE;
    while (!(IFG2 & UCB0TXIFG));//{
       // __no_operation();
    //}
    dataBuffer[2] = RXData;
    unsigned char i = 3;
    TXData = 0;
    for(i = 3; i < 30; i++){
        IE2 |= UCB0TXIE;
        while (!(IFG2 & UCB0TXIFG));//{
           // __no_operation();
        //}
        dataBuffer[i] = RXData;
    }
    if(CS == 3 || CS == 2 || CS == 5){
        P2OUT |= BIT;
    }
    else{
        P3OUT |= BIT; //raises it back high, disabling this sensor
    }
    state1 = IDLE_MODE;
}


void sendReadings(){
    size_t i ;
    double mask = 0xFF;
    char *bytes = &READING;
    for(i = 0; i < sizeof(double); i++){
        UCA0TXBUF = (char) (bytes[i]);
    }

}



// Tx Interrupt
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCIB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    UCB0TXBUF = TXData;
    IE2 &= ~UCB0TXIE;

}


// Rx Interrupt
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIB0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIB0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    RXData = UCB0RXBUF;
}



///#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
///#pragma vector=USCIAB0RX_VECTOR
///__interrupt void USCBB0RX_ISR (void)
///#elif defined(__GNUC__)
///void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
///#else
///#error Compiler not supported!
///#endif
///{
///  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
///  if(UCA0RXBUF == 1){
///      sendReadings(); //handles handing off the bytes to the TXBUF
///  }
///}

//#pragma vector=USCIAB0TX_VECTOR
//__interrupt void USCIB0TX_ISR(void)
//{
//    UCB0TXBUF = TXData;
//    IE2 &= ~UCB0TXIE;
//}
//__interrupt void USCIA0TX_ISR(void)
//{
//    UCA0TXBUF = TXData_slave;
//    IE2 &= ~UCA0TXIE;
//}
//// Rx Interrupt
//#pragma vector=USCIAB0RX_VECTOR
//__interrupt void USCIB0RX_ISR(void)
//{
//    RXData = UCB0RXBUF;
//}
//__interrupt void USCIA0RX_ISR(void)
//{
//    slaveCmd = UCA0RXBUF;
//}}

/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430G2xx3 Demo - USCI_A0, SPI 3-Wire Slave Data Echo
//
//   Description: SPI slave talks to SPI master using 3-wire mode. Data received
//   from master is echoed back.  USCI RX ISR is used to handle communication,
//   CPU normally in LPM4.  Prior to initial data exchange, master pulses
//   slaves RST for complete reset.
//   ACLK = n/a, MCLK = SMCLK = DCO ~1.2MHz
//
//   Use with SPI Master Incremented Data code example.  If the slave is in
//   debug mode, the reset signal from the master will conflict with slave's
//   JTAG; to work around, use IAR's "Release JTAG on Go" on slave device.  If
//   breakpoints are set in slave RX ISR, master must stopped also to avoid
//   overrunning slave RXBUF.
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          | |             XOUT|-
// Master---+-|RST              |
//            |             P1.2|<- Data Out (UCA0SOMI)
//            |                 |
//            |                 |
//            |             P1.4|<- Serial Clock In (UCA0CLK)
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
//#include <msp430.h>
//#include <stdlib.h>
//#include <stdio.h>
//double READING[1];
//size_t byteI = 0;
//char *txSend[8] ;
//
//int main(void)
//{
//  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer
//  while (!(P1IN & BIT4));                   // If clock sig from mstr stays low,
//                                            // it is not yet in SPI mode
//  READING[0] = 15213.0;
//  P1SEL = BIT1 + BIT2 + BIT4;
//  P1SEL2 = BIT1 + BIT2 + BIT4;
//  UCA0CTL1 = UCSWRST;                       // **Put state machine in reset**
//  UCA0CTL0 |= UCCKPL + UCMSB + UCSYNC;      // 3-pin, 8-bit SPI master
//  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//  //TODO DISABLE THIS DURING CALCULATION
//  IE2 |= UCA0RXIE;                          // Enable USCI0 RX interrupt
//  txSend[0] = 0x06;
//  txSend[1] = 0x05;
//  while(1){
//      __bis_SR_register(LPM0_bits + GIE);       // Enter LPM4, enable interrupts
//  }
//
//  unsigned char slaveCmd = UCA0RXBUF;
//  return 0;
//}
//
//void sendReadings(){
//    size_t i ;
//    double mask = 0xFF;
//    char *bytes = &READING;
//    for(i = 0; i < sizeof(double); i++){
//        UCA0TXBUF = (char) (bytes[i]);
//        while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
//    }
//}
//
//
//
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector=USCIAB0RX_VECTOR
//__interrupt void USCI0RX_ISR (void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
//  //UCA0TXBUF = UCA0RXBUF;
//  //if(UCA0RXBUF == 1){
// // if(byteI == 0){
////      IE2 |= UCA0TXIE;
////  }
//    UCA0TXBUF = (unsigned char)txSend[byteI] ; //*(((char *) READING)+byteI);
//    byteI ++;
//    if(byteI == 1){
////     // IE2 &= ~UCA0TXIE ; //disable it back down
//       byteI = 0;  // 00
//    }
//      //sendReadings(); //handles handing off the bytes to the TXBUF
//  //}
//
//}
////// Echo character
////#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
////#pragma vector=USCIAB0RX_VECTOR
////__interrupt void USCI0RX_ISR (void)
////#elif defined(__GNUC__)
////void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
////#else
////#error Compiler not supported!
////#endif
////{
////  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
////  UCA0TXBUF = UCA0RXBUF;
////}
//
//
//
////           unsigned char toShift = databuffer[i] ;
////           //Get rid of permanent 1 by & with 3rd MSB of our tail End
////           unsigned char thirdMSB = (finalArray[count] >> 5) & 1 ;
////           toShift = toShift >> 2 ; // get our offset
////           if(!thirdMSB){
////               //if third MSB is 0 we need to set our permenant 1 to 0
////               toShift = toShift & 0x1F;
////           }
////           //else it's already 1 so we don't care, now we put back into finalArray
//
//
//
///*** Dead Test code
//     //init(1);
//    __no_operation();
//    //setThreshold(2, 10);
//    //sendZebra1(BIT7);
//    //sendZebra0(bit);
//    //sendIntegration(1);
//    __no_operation();
////    while(1){
////        //int toCheck = P2IN & corresponding FR bit;
////        //WE want to break loop if P2In is high 1, so when we and it with bit, we get 1.
////        //If P2in is low, toCheck returns 0; so we never go into our break
////        if (P3IN & BIT6){ // wait for FR to go back high
////            __no_operation();
////            break;
////        }
////    }
//    //while(~(P2IN & bit)); //wait for frame ready to go back high to signal ready to read
//    __no_operation();
//   // readAll(1);
//    __no_operation();
//
 //***/
//            |             P1.1|-> Data In (UCA0SIMO)
//            |                 |
//            |             P1.4|<- Serial Clock In (UCA0CLK)
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
///#include <msp430.h>
///#include <stdlib.h>
///#include <stdio.h>
///double READING[1];
///size_t byteI = 0;
///char *txSend[8] ;
///
///int main(void)
///{
///  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer
///  while (!(P1IN & BIT4));                   // If clock sig from mstr stays low,
///                                            // it is not yet in SPI mode
///  READING[0] = 15213.0;
///  P1SEL = BIT1 + BIT2 + BIT4;
///  P1SEL2 = BIT1 + BIT2 + BIT4;
///  UCA0CTL1 = UCSWRST;                       // **Put state machine in reset**
///  UCA0CTL0 |= UCCKPL + UCMSB + UCSYNC;      // 3-pin, 8-bit SPI master
///  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
/// //TODO DISABLE THIS DURING CALCULATION
//   IE2 |= UCA0RXIE;                          // Enable USCI0 RX interrupt
//  txSend[0] = 0x06;
//  txSend[1] = 0x05;;
///  while(1){
///      __bis_SR_register(LPM0_bits + GIE);       // Enter LPM4, enable interrupts
///  }
///
///  unsigned char slaveCmd = UCA0RXBUF;
///  return 0;
///}
///
///void sendReadings(){
///    size_t i ;
///    double mask = 0xFF;
///    char *bytes = &READING;
///    for(i = 0; i < sizeof(double); i++){
///        UCA0TXBUF = (char) (bytes[i]);
///        while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
///    }
///}
///
///
///
///#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
///#pragma vector=USCIAB0RX_VECTOR
///__interrupt void USCI0RX_ISR (void)
///#elif defined(__GNUC__)
///void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
///#else
///#error Compiler not supported!
///#endif
///{
///  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
/// ///UCA0TXBUF = UCA0RXBUF;
///  //if(UCA0RXBUF == 1){
/// // if(byteI == 0){
/////      IE2 |= UCA0TXIE;
/////
//     UCA0TXBUF =(unsigned char)txSend[byteI]] ; //*(((char *) READING)+byteI);
//     byteI ++;
//     if(byteI ==88){
/////     // IE2 &= ~UCA0TXIE ; //disable it back down
//        byteI = 0;  // 00
//     }
///      //sendReadings(); //handles handing off the bytes to the TXBUF
///  //}
///}
// Echo character
///#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
///#pragma vector=USCIAB0RX_VECTOR
///__interrupt void USCI0RX_ISR (void)
///#elif defined(__GNUC__)
///void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
///#else
///#error Compiler not supported!
///#endif
///{
///  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
///  UCA0TXBUF = UCA0RXBUF;
///}
///
///
///
/////           unsigned char toShift = databuffer[i] ;
/////           //Get rid of permanent 1 by & with 3rd MSB of our tail End
/////           unsigned char thirdMSB = (finalArray[count] >> 5) & 1 ;
/////           toShift = toShift >> 2 ; // get our offset
/////           if(!thirdMSB){
/////               //if third MSB is 0 we need to set our permenant 1 to 0
/////               toShift = toShift & 0x1F;
/////           }
/////           //else it's already 1 so we don't care, now we put back into finalArray
///
///
///
////*** Dead Test code
///     //init(1);
///    __no_operation();
///    //setThreshold(2, 10);
///    //sendZebra1(BIT7);
///    //sendZebra0(bit);
///    //sendIntegration(1);
///    __no_operation();
/////    while(1){
/////        //int toCheck = P2IN & corresponding FR bit;
/////        //WE want to break loop if P2In is high 1, so when we and it with bit, we get 1.
/////        //If P2in is low, toCheck returns 0; so we never go into our break
/////        if (P3IN & BIT6){ // wait for FR to go back high
/////            __no_operation();
/////            break;
/////        }
/////    }
///    //while(~(P2IN & bit)); //wait for frame ready to go back high to signal ready to read
///    __no_operation();
///   // readAll(1);
///    __no_operation();
///
///***/
