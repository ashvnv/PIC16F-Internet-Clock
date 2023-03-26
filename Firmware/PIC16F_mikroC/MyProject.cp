#line 1 "D:/MEGA/Experience/Selec Controls/Project/FILES/Firmware/PIC16F_mikroC/MyProject.c"
#line 9 "D:/MEGA/Experience/Selec Controls/Project/FILES/Firmware/PIC16F_mikroC/MyProject.c"
unsigned bcd, lower_nib;
unsigned timer_overflow_seconds =  0xC4 ;



unsigned char time_set_digit_addr = 1;
unsigned char rtc_hour_set = 0;
unsigned char rtc_min_set = 0;
unsigned char rtc_temp_set = 0;


void SPI_init(void);
void SPI_tx(unsigned char, unsigned char);
void SPI_update(void);

void I2C_init(void);
void I2C_update_rtc(unsigned char);
void I2C_update_time(void);


void main() {

 PORTC = 0x00;
 TRISC.F5 = 0;

 ANS4_bit = 0;
 TRISA.F5 = 0;
 PORTA.F5 = 1;

 CKE_bit = 1;
 SSPADD = 0x15;



 I2C_init();
 delay_ms(20);

 SEN_bit = 1;
 while(SEN_bit);

 SSPBUF = 0xD0;
 while(SSPSTAT & 0x04);

 SSPBUF = 0x07;
 while(SSPSTAT & 0x04);

 SSPBUF = 0x90;
 while(SSPSTAT & 0x04);

 PEN_bit = 1;
 while(PEN_bit);

 I2C_update_time();

 PORTC.F0 = 0;
 TRISC.F0 = 1;

 TMR1CS_bit = 1;
 T1SYNC_bit = 1;

 TMR1IE_bit = 1;
 PEIE_bit = 1;



 TMR1H = 0xFF;
 TMR1L = timer_overflow_seconds;

 TMR1IF_bit = 0;
 TMR1ON_bit = 1;



 SPI_init();

 SPI_tx(0x09, 0xFF);
 SPI_tx(0x0B, 0x03);

 SPI_update();

 SPI_tx(0x0C, 0x01);



 TRISA.F0 = 1;
 PORTA.F0 = 1;
 ADCS0_bit = 1;


 ADON_bit = 1;
 delay_ms(1);
 ADCON0 = ADCON0 | 0x02;
 while(ADCON0 & 0x02);
 SPI_tx(0x0A,ADRESH * 15 / 128);
 ADON_bit = 0;



 ANS1_bit = 0;
 TRISA.F1 = 0;
 PORTA.F1 = 1;


 SPBRG = 25;
 BRGH_bit = 1;
 TXEN_bit = 1;
 CREN_bit = 1;



 ANS12_bit = 0;
 ANS10_bit = 0;
 PORTB.F0 = 0;
 TRISB.F0 = 1;
 PORTB.F1 = 0;
 TRISB.F1 = 1;
 WPUB = 0x03;
 OPTION_REG = OPTION_REG & 0x7F;

 INTEDG_bit = 0;
 INTF_bit = 0;
 INTE_bit = 1;

 while(1) {
 PORTA.F1 = ~PORTA.F1;

 asm sleep;
 delay_ms(20);

 INTF_bit = 0;

 PORTA.F1 = ~PORTA.F1;


 if (TMR1IF_bit) {
 TMR1ON_bit = 0;



 TMR1H = 0xFF;
 TMR1L =  0xC4 ;

 I2C_update_time();
 SPI_update();

 TMR1IF_bit = 0;
 TMR1ON_bit = 1;
 }

 else if (PORTB.F1) {



 TMR1ON_bit = 0;

 TMR1H = 0xFF;
 TMR1L =  0xC4 ;
 TMR1IF_bit = 0;


 SPI_init();
 SPI_tx(0x01,0x00);

 SPI_tx(0x02,0x0A);
 SPI_tx(0x03,0x0A);
 SPI_tx(0x04,0x0A);


 rtc_hour_set = 0;
 rtc_min_set = 0;

 while(1) {


 asm sleep;
 delay_ms(20);
 INTF_bit = 0;

 if (~PORTB.F1){


 ++time_set_digit_addr;
 if (time_set_digit_addr > 4){

 I2C_update_rtc(0);
 TMR1ON_bit = 1;


 time_set_digit_addr = 1;
 rtc_temp_set = 0;

 break;
 }

 rtc_temp_set = 0;

 SPI_tx(time_set_digit_addr,0x00);


 } else {



 switch(time_set_digit_addr) {


 case 1: if (++rtc_temp_set > 2)
 rtc_temp_set = 0;
 rtc_hour_set = rtc_temp_set << 4;
 break;
 case 2: ++rtc_temp_set;
 if ((rtc_hour_set >> 4) == 2 && rtc_temp_set > 3)
 rtc_temp_set = 0;
 else if (rtc_temp_set > 9)
 rtc_temp_set = 0;
 rtc_hour_set &= 0xF0;
 rtc_hour_set |= rtc_temp_set;
 break;



 case 3: if (++rtc_temp_set > 5)
 rtc_temp_set = 0;
 rtc_min_set = rtc_temp_set << 4;
 break;
 case 4: if (++rtc_temp_set > 9)
 rtc_temp_set = 0;
 rtc_min_set &= 0xF0;
 rtc_min_set |= rtc_temp_set;
 break;

 }

 SPI_tx(time_set_digit_addr,rtc_temp_set);
 }
 }
 }
 else {
#line 259 "D:/MEGA/Experience/Selec Controls/Project/FILES/Firmware/PIC16F_mikroC/MyProject.c"
 TMR1ON_bit = 0;
 TMR1IF_bit = 0;


 SPI_init();
 SPI_tx(0x01,0x0A);
 SPI_tx(0x02,0x08);
 SPI_tx(0x03,0x08);
 SPI_tx(0x04,0x0A);

 ADON_bit = 1;

 while(~INTF_bit){
 delay_ms(1);
 ADCON0 = ADCON0 | 0x02;
 while(ADCON0 & 0x02);
 SPI_tx(0x0A,ADRESH * 15 / 128);
 }

 delay_ms(20);
 INTF_bit = 0;


 ADON_bit = 0;



 if (~PORTB.F1) {


 SPI_init();
 SPI_tx(0x01,0x0A);
 SPI_tx(0x02,0x0A);
 SPI_tx(0x03,0x0A);
 SPI_tx(0x04,0x0A);

 SPEN_bit = 1;

 TXREG = 1;
 while(!TRMT_bit);

 INTF_bit = 0;



 while(!RCIF_bit && ~INTF_bit);

 if (INTF_bit){

 delay_ms(20);
 I2C_update_time();
 INTF_bit = 0;

 } else {

 rtc_hour_set = RCREG;

 while(!RCIF_bit);
 rtc_min_set = RCREG;

 while(!RCIF_bit);
 bcd = RCREG & 0x7F;
 lower_nib = bcd & 0x0F;
 timer_overflow_seconds = (bcd >> 4)*10;
 timer_overflow_seconds += lower_nib;
 timer_overflow_seconds = 0xFF - (60 - timer_overflow_seconds);

 I2C_update_rtc(bcd);
 }


 } else {

 I2C_update_time();
 }


 TMR1H = 0xFF;
 TMR1L = timer_overflow_seconds;
 TMR1IF_bit = 0;
 TMR1ON_bit = 1;

 SPI_update();
 }
 }
}

void SPI_init(void){
 SSPEN_bit = 0;
 TRISC.F3 = 0;
 SMP_bit = 1;
 SSPCON = 0x22;
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

 SPI_tx(0x01,rtc_hour_set >> 4);
 SPI_tx(0x02,rtc_hour_set & 0x0F);
 SPI_tx(0x03,rtc_min_set >> 4);
 SPI_tx(0x04,rtc_min_set & 0x0F);

}



void I2C_init(){
 SSPEN_bit = 0;
 TRISC.F3 = 1;
 SMP_bit = 0;
 SSPCON = 0x28;
}

void I2C_update_rtc(unsigned char sec){
 I2C_init();

 SEN_bit = 1;
 while(SEN_bit);

 SSPBUF = 0xD0;
 while(SSPSTAT & 0x04);

 SSPBUF = 0x00;
 while(SSPSTAT & 0x04);


 SSPBUF = sec;
 while(SSPSTAT & 0x04);


 SSPBUF = rtc_min_set;
 while(SSPSTAT & 0x04);


 SSPBUF = rtc_hour_set;
 while(SSPSTAT & 0x04);


 PEN_bit = 1;
 while(PEN_bit);
}

void I2C_update_time(){
 I2C_init();

 SEN_bit = 1;
 while(SEN_bit);

 SSPBUF = 0xD0;
 while(SSPSTAT & 0x04);

 SSPBUF = 0x00;
 while(SSPSTAT & 0x04);

 RSEN_bit = 1;
 while(RSEN_bit);

 SSPBUF = 0xD1;
 while(SSPSTAT & 0x04);


 RCEN_bit = 1;
 while(RCEN_bit);

 ACKDT_bit = 0;
 ACKEN_bit = 1;
 while(ACKEN_bit);


 bcd = SSPBUF;
 lower_nib = bcd & 0x0F;
 timer_overflow_seconds = (bcd >> 4)*10;
 timer_overflow_seconds += lower_nib;
 timer_overflow_seconds = 0xFF - (60 - timer_overflow_seconds);



 RCEN_bit = 1;
 while(RCEN_bit);

 ACKDT_bit = 0;
 ACKEN_bit = 1;
 while(ACKEN_bit);

 rtc_min_set = SSPBUF;


 RCEN_bit = 1;
 while(RCEN_bit);

 ACKDT_bit = 1;
 ACKEN_bit = 1;
 while(ACKEN_bit);

 rtc_hour_set = SSPBUF;

 PEN_bit = 1;
 while(PEN_bit);
}
