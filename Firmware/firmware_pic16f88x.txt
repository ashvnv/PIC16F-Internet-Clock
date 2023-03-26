/*
  *     Internet Clock Firmware for PIC16F88x Series Microcontroller
  *     Author: Ashwin Vallaban     https://github.com/ashvnv
  *     Date: Feb 2023
*/

//------------------Time Variables---------------------
#define SECONDS_OVERFLOW_VAL 0xC4
unsigned bcd, lower_nib;
unsigned timer_overflow_seconds = SECONDS_OVERFLOW_VAL;
//-----------------------------------------------------

//------Variables used during manual time set operation------
unsigned char time_set_digit_addr = 1;//Address (MAX7219) of the digit being updated
unsigned char rtc_hour_set = 0; //store hour
unsigned char rtc_min_set = 0; //store min
unsigned char rtc_temp_set = 0; //store the current digit value
//-----------------------------------------------------------

void SPI_init(void);
void SPI_tx(unsigned char, unsigned char);
void SPI_update(void);

void I2C_init(void);
void I2C_update_rtc(unsigned char);
void I2C_update_time(void);


void main() {
     //------------Init MSSP peripheral (SPI & I2C)---------------
     PORTC = 0x00;//Clear the Port
     TRISC.F5 = 0;  //configured as O/P for SDO

     ANS4_bit = 0; //Digital IO
     TRISA.F5 = 0; //configure as LOAD pin to MAX7219
     PORTA.F5 = 1; //Pin is default HIGH

     CKE_bit = 1; //Transmit data on rising edge of CLK (For SPI)
     SSPADD = 0x15;//Clock speed of approx. 45kHz (For I2C BRG)
     //------------------------------------------------

     //--------------DS1307 Initialization-------------
     I2C_init();
     delay_ms(20); //Wait for RTC clock to stabilize

     SEN_bit = 1;    //Enable a start condition
     while(SEN_bit); //wait for start condition to complete

     SSPBUF = 0xD0;  //RTC add with write command
     while(SSPSTAT & 0x04); //Wait till transfer is complete

     SSPBUF = 0x07;  //RTC SQWOut register address
     while(SSPSTAT & 0x04); //Wait till transfer is complete

     SSPBUF = 0x90;  //Enable 1Hz clock
     while(SSPSTAT & 0x04); //Wait till transfer is complete

     PEN_bit = 1;    //Enable a stop condition
     while(PEN_bit); //wait for stop condition to complete
     //--------------------------------------------------
     I2C_update_time();
     //-------------Configure the 60 Seconds counter--------------------
     PORTC.F0 = 0; //Clear the T1CKI port pin
     TRISC.F0 = 1; //Configure T1CKI as an Input pin

     TMR1CS_bit = 1;//Use external clock. Configured as a Counter
     T1SYNC_bit = 1;//Do not synchronize the external clock

     TMR1IE_bit = 1;//Enable overflow interrupt
     PEIE_bit = 1;//Enable Peripheral Interrupt

     //Load TMR1H:TMR1L with  which makes the Counter overflow
     //     at the  rising edge of external clock
     TMR1H = 0xFF;
     TMR1L = timer_overflow_seconds;

     TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
     TMR1ON_bit = 1;//Turn on the Timer


     //--------------MAX7219 Initialization------------
     SPI_init();

     SPI_tx(0x09, 0xFF);  //Code B Dcoding enabled
     SPI_tx(0x0B, 0x03); //scan limit to 4 digits

     SPI_update();

     SPI_tx(0x0C, 0x01); //disable sleep
     //------------------------------------------------

     //-----------------ADC Configure----------------------
     TRISA.F0 = 1;
     PORTA.F0 = 1;
     ADCS0_bit = 1;//Set the clock to Fosc/8

     //Set the brightness
     ADON_bit = 1; //turn on ADC module
     delay_ms(1);//A/D Acquisition time
     ADCON0 = ADCON0 | 0x02;                     //start conversion
     while(ADCON0 & 0x02); //wait for conversion
     SPI_tx(0x0A,ADRESH * 15 / 128); //set the intensity. SPI_init() called above.
     ADON_bit = 0; //turn offf ADC module
     //----------------------------------------------------

     //----------------SLEEP Indicator LED-----------------
     ANS1_bit = 0; //Digital IO
     TRISA.F1 = 0;//Configure as Output
     PORTA.F1 = 1;//LED is ON

     //----------------UART Configure-----------------
     SPBRG = 25; //for 4MHz OSC 9600 baud with 0.16% error
     BRGH_bit = 1;//Enable high speed mode
     TXEN_bit = 1; //enable transmission
     CREN_bit = 1; //enable reception

     //-----------Manual Time Set Switches--------

     ANS12_bit = 0; //Digital IO
     ANS10_bit = 0; //Digital IO
     PORTB.F0 = 0;  //Clear Port Pin
     TRISB.F0 = 1;  //Configure as an input pin
     PORTB.F1 = 0;  //Clear Port Pin
     TRISB.F1 = 1;  //Configure as an input pin
     WPUB = 0x03; //enable only RB0 and RB1 pullup
     OPTION_REG = OPTION_REG & 0x7F; //Enable pull ups

     INTEDG_bit = 0; //Interrupt on falling edge of INT pin
     INTF_bit = 0;//Clear the external interrupt flag
     INTE_bit = 1;//Enable external interrupt

     while(1) {
              PORTA.F1 = ~PORTA.F1;//Toggle MODE LED

              asm sleep;
              delay_ms(20);//Debounce

              INTF_bit = 0;//Clear the INT flag

              PORTA.F1 = ~PORTA.F1;//Toggle MODE LED

              //Check if 60 seconds counter interrupted
              if (TMR1IF_bit) {
                 TMR1ON_bit = 0; //To prevent write contention turn off the timer

                 //Load TMR1H:TMR1L with 0xFFC4 which makes the Counter overflow
                 //     at 60th rising edge of external clock
                 TMR1H = 0xFF;
                 TMR1L = SECONDS_OVERFLOW_VAL;

                 I2C_update_time();//Update the global registers
                 SPI_update(); //Update the MAX7219 driver

                 TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
                 TMR1ON_bit = 1;//Turn on the Timer
              }
              //Check if SW1 switch was clicked while SW0 switch is clicked
              else if (PORTB.F1) {
                 //In time set mode

                  //Disable 60 seconds overflow counter
                  TMR1ON_bit = 0;//Turn off the timer
                  //Reset the timer registers
                  TMR1H = 0xFF;
                  TMR1L = SECONDS_OVERFLOW_VAL;
                  TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt

                  //Initialize the display for setting time
                  SPI_init();
                  SPI_tx(0x01,0x00);//Digit 0 will display 0
                  //Other digits will display a dash (-)
                  SPI_tx(0x02,0x0A);
                  SPI_tx(0x03,0x0A);
                  SPI_tx(0x04,0x0A);

                  //Clear the global time variables
                  rtc_hour_set = 0;
                  rtc_min_set = 0;

                  while(1) {
                  //Set Time Loop

                        asm sleep; //sleep till switch is clicked
                        delay_ms(20);//Debounce
                        INTF_bit = 0;//Clear the INT flag

                        if (~PORTB.F1){
                           //SW0 is clicked, SW1 is clicked
                           //Move to next digit
                           ++time_set_digit_addr;
                           if (time_set_digit_addr > 4){
                              //Update DS1307 registers with seconds register reset
                              I2C_update_rtc(0);
                              TMR1ON_bit = 1;//Turn on the 60 seconds overflow

                              //Reset the digit address and clear temp register
                              time_set_digit_addr = 1;
                              rtc_temp_set = 0;

                              break; //Time Set Complete
                           }
                           //Clear the temporary register value
                           rtc_temp_set = 0;
                           //Current digit will display 0
                           SPI_tx(time_set_digit_addr,0x00);//Digit 0 will display 0


                        } else {
                           //SW0 is clicked, SW1 is not clicked
                           //Incriment current digit time

                           switch(time_set_digit_addr) {
                              //--------Hour Digit (0 and 1)---------
                              //Hour variable rtc_hour_set
                              case 1:  if (++rtc_temp_set > 2)
                                          rtc_temp_set = 0;//Hour MSB [0 - 2]
                                       rtc_hour_set = rtc_temp_set << 4;
                                   break;
                              case 2:  ++rtc_temp_set;
                                       if ((rtc_hour_set >> 4) == 2 && rtc_temp_set > 3)
                                          rtc_temp_set = 0;//Hour can be max 23
                                       else if (rtc_temp_set > 9)
                                          rtc_temp_set = 0;//Hour LSB [0 - 9]
                                       rtc_hour_set &= 0xF0;
                                       rtc_hour_set |= rtc_temp_set;
                                   break;
                              //------------------------------------
                              //-------- Minute Digit (2 & 3)-------
                              //Minute variable rtc_min_set
                              case 3:  if (++rtc_temp_set > 5)
                                          rtc_temp_set = 0;//Min MSB [0 - 5]
                                       rtc_min_set = rtc_temp_set << 4;
                                   break;
                              case 4:  if (++rtc_temp_set > 9)
                                          rtc_temp_set = 0;//Min LSB [0 - 9]
                                       rtc_min_set &= 0xF0;
                                       rtc_min_set |= rtc_temp_set;
                                   break;
                              //------------------------------------
                           }
                           //update the current digit
                           SPI_tx(time_set_digit_addr,rtc_temp_set);
                        }
                  }
              }
              else {
                   //Brightness adjust and getting NTP time
                   /*While the microcontroller is in this mode the brightness can be
                         adjusted. MAX7219 will display - 8 8 -
                         For syncing the time with
                         the Internet, SW1 & SW0 button has to be clicked simultaneously
                         while in this mode. The Microcontroller will then go into NTP
                         Sync mode. MAX7219 displays - - - - and once the time is obtained
                         from ESP8266 via UART DS1307 registers are updated and the clock
                         goes into normal mode. At any time for going back to the normal
                         mode SW0 interrupt button is to be clicked. */

                   //Disable 60 seconds overflow counter
                    TMR1ON_bit = 0;//Turn off the timer
                    TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt

                    //Initialize the display to - 8 8 - (Brightness Adj Mode)
                    SPI_init();
                    SPI_tx(0x01,0x0A);
                    SPI_tx(0x02,0x08);
                    SPI_tx(0x03,0x08);
                    SPI_tx(0x04,0x0A);

                    ADON_bit = 1; //turn on ADC module

                    while(~INTF_bit){ //Wait till interrupt
                         delay_ms(1);//A/D Acquisition time
                         ADCON0 = ADCON0 | 0x02;                     //start conversion
                         while(ADCON0 & 0x02); //wait for conversion
                         SPI_tx(0x0A,ADRESH * 15 / 128); //set the intensity. SPI_init() called above.
                    }
                    //Button interrupt
                    delay_ms(20); //Debounce
                    INTF_bit = 0;//Clear INT flag

                    //Turn off the ADC module
                    ADON_bit = 0;

                    //Check if the INC button was clicked or not. If not return to
                    //      normal mode else go to NTP Sync mode.
                    if (~PORTB.F1) {
                       //INC button was clicked. In NTP SYNC mode
                          //Initialize the display to - - - - (NTP SYNC Mode)
                           SPI_init();
                           SPI_tx(0x01,0x0A);
                           SPI_tx(0x02,0x0A);
                           SPI_tx(0x03,0x0A);
                           SPI_tx(0x04,0x0A);

                           SPEN_bit = 1;//enable uart

                           TXREG = 1;//Send time update request
                           while(!TRMT_bit); //wait for transmission to complete

                           INTF_bit = 0; //Clear external INT flag (Button INT)

                           //Wait for ESP to send the time information. If the external INT
                           //     button is clicked, cancel the time update option
                           while(!RCIF_bit && ~INTF_bit); //wait for reception (HOUR)

                           if (INTF_bit){
                              //Cancel the time sync operation as the INT button was clicked
                              delay_ms(20);//Debounce wait
                              I2C_update_time(); //Sync microcontroller time from DS1307
                              INTF_bit = 0;//Clear INT flag

                           } else {
                           //Receive the time information from ESP
                              rtc_hour_set = RCREG;

                              while(!RCIF_bit); //wait for reception (MIN)
                              rtc_min_set = RCREG;

                              while(!RCIF_bit); //wait for reception (SEC)
                              bcd = RCREG & 0x7F; //CH bit in DS1307 should be 0
                              lower_nib = bcd & 0x0F;
                              timer_overflow_seconds = (bcd >> 4)*10;
                              timer_overflow_seconds += lower_nib;
                              timer_overflow_seconds = 0xFF - (60 - timer_overflow_seconds);

                              I2C_update_rtc(bcd);//Update DS1307 time registers with seconds
                           }


                    } else {
                      //Go to normal mode.
                      I2C_update_time(); //Sync microcontroller time from DS1307
                    }

                    //Reset the timer registers
                    TMR1H = 0xFF;
                    TMR1L = timer_overflow_seconds;
                    TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
                    TMR1ON_bit = 1;//Turn on the 60 seconds overflow

                    SPI_update();//Update the display
              }
     }
}

void SPI_init(void){
     SSPEN_bit = 0;//Turn off the I2C mode
     TRISC.F3 = 0;  //configured as O/P SCK
     SMP_bit = 1; //Input data sampled at end of data output time
     SSPCON = 0x22; //Enable SPI with Fosc/64 Clock
}

void SPI_tx(unsigned char addr,unsigned char dat)
{
     PORTA.F5 = 0;
     SSPBUF = addr;
     while(~BF_bit);
     SSPBUF = dat;
     while(~BF_bit);
     PORTA.F5 = 1;
}

void SPI_update() {
     SPI_init();
     //------------Set the display------------
     SPI_tx(0x01,rtc_hour_set >> 4);//Digit 0 Hour MSB
     SPI_tx(0x02,rtc_hour_set & 0x0F);//Digit 1 Hour LSB
     SPI_tx(0x03,rtc_min_set >> 4);//Digit 2 Minute MSB
     SPI_tx(0x04,rtc_min_set & 0x0F);//Digit 3 Minute LSB
     //---------------------------------------
}



void I2C_init(){
     SSPEN_bit = 0;  //Clear MSSP
     TRISC.F3 = 1;  //configured as I/P SCL
     SMP_bit = 0;
     SSPCON = 0x28;   //Configure in I2C master mode
}

void I2C_update_rtc(unsigned char sec){
     I2C_init();

     SEN_bit = 1; //enable start condition on I2C Bus
     while(SEN_bit); //wait for start condition to complete

     SSPBUF = 0xD0; //RTC address with Write command
     while(SSPSTAT & 0x04); //Wait for transfer to complete

     SSPBUF = 0x00; //RTC seconds register address
     while(SSPSTAT & 0x04); //Wait for transfer to complete

     //-----------------Update Seconds Register of DS1307---------------
     SSPBUF = sec;//Seconds register
     while(SSPSTAT & 0x04); //Wait for transfer to complete

     //-----------------Update Minutes Register of DS1307---------------
     SSPBUF = rtc_min_set; //Update the minutes register
     while(SSPSTAT & 0x04); //Wait for transfer to complete

     //-----------------Update Hour Register of DS1307---------------
     SSPBUF = rtc_hour_set; //Update the hour register
     while(SSPSTAT & 0x04); //Wait for transfer to complete


     PEN_bit = 1;//Enable stop condition
     while(PEN_bit); //Wait for stop condition to complete
}

void I2C_update_time(){
     I2C_init();

     SEN_bit = 1; //enable start condition on I2C Bus
     while(SEN_bit); //wait for start condition to complete

     SSPBUF = 0xD0; //RTC address with Write command
     while(SSPSTAT & 0x04); //Wait for transfer to complete

     SSPBUF = 0x00; //RTC seconds register address
     while(SSPSTAT & 0x04); //Wait for transfer to complete

     RSEN_bit = 1; //Repeat start
     while(RSEN_bit);//Wait for Repeat Start condition to complete

     SSPBUF = 0xD1; //RTC add with read command
     while(SSPSTAT & 0x04);//Wait for transfer to complete

     //-----------------Read Seconds Register of DS1307---------------
     RCEN_bit = 1; //Receive enable
     while(RCEN_bit);//wait till the data is received

     ACKDT_bit = 0;
     ACKEN_bit = 1;//Send ACKDT ack
     while(ACKEN_bit); //ACK

     //--------BCD to decimal and Timer overflow conversion----
     bcd = SSPBUF;
     lower_nib = bcd & 0x0F;
     timer_overflow_seconds = (bcd >> 4)*10;
     timer_overflow_seconds += lower_nib;
     timer_overflow_seconds = 0xFF - (60 - timer_overflow_seconds);
     //--------------------------------------------------------

     //-----------------Read Minutes Register of DS1307---------------
     RCEN_bit = 1; //Receive enable
     while(RCEN_bit);//wait till the data is received

     ACKDT_bit = 0;
     ACKEN_bit = 1;//Send ACKDT ack
     while(ACKEN_bit); //ACK

     rtc_min_set = SSPBUF; //Update the minutes global variable

     //-----------------Read Hour Register of DS1307---------------
     RCEN_bit = 1; //Receive enable
     while(RCEN_bit);//wait till the data is received

     ACKDT_bit = 1;//No more registers to read so send NACK
     ACKEN_bit = 1;//Send the ACKDT to slave
     while(ACKEN_bit); //Wait for NACK to completely transfer

     rtc_hour_set = SSPBUF; //Update the hour global variable

     PEN_bit = 1;//Enable stop condition
     while(PEN_bit); //Wait for stop condition to complete
}