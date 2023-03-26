
_main:

;MyProject.c,29 :: 		void main() {
;MyProject.c,31 :: 		PORTC = 0x00;//Clear the Port
	CLRF       PORTC+0
;MyProject.c,32 :: 		TRISC.F5 = 0;  //configured as O/P for SDO
	BCF        TRISC+0, 5
;MyProject.c,34 :: 		ANS4_bit = 0; //Digital IO
	BCF        ANS4_bit+0, BitPos(ANS4_bit+0)
;MyProject.c,35 :: 		TRISA.F5 = 0; //configure as LOAD pin to MAX7219
	BCF        TRISA+0, 5
;MyProject.c,36 :: 		PORTA.F5 = 1; //Pin is default HIGH
	BSF        PORTA+0, 5
;MyProject.c,38 :: 		CKE_bit = 1; //Transmit data on rising edge of CLK (For SPI)
	BSF        CKE_bit+0, BitPos(CKE_bit+0)
;MyProject.c,39 :: 		SSPADD = 0x15;//Clock speed of approx. 45kHz (For I2C BRG)
	MOVLW      21
	MOVWF      SSPADD+0
;MyProject.c,43 :: 		I2C_init();
	CALL       _I2C_init+0
;MyProject.c,44 :: 		delay_ms(20); //Wait for RTC clock to stabilize
	MOVLW      26
	MOVWF      R12+0
	MOVLW      248
	MOVWF      R13+0
L_main0:
	DECFSZ     R13+0, 1
	GOTO       L_main0
	DECFSZ     R12+0, 1
	GOTO       L_main0
	NOP
;MyProject.c,46 :: 		SEN_bit = 1;    //Enable a start condition
	BSF        SEN_bit+0, BitPos(SEN_bit+0)
;MyProject.c,47 :: 		while(SEN_bit); //wait for start condition to complete
L_main1:
	BTFSS      SEN_bit+0, BitPos(SEN_bit+0)
	GOTO       L_main2
	GOTO       L_main1
L_main2:
;MyProject.c,49 :: 		SSPBUF = 0xD0;  //RTC add with write command
	MOVLW      208
	MOVWF      SSPBUF+0
;MyProject.c,50 :: 		while(SSPSTAT & 0x04); //Wait till transfer is complete
L_main3:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_main4
	GOTO       L_main3
L_main4:
;MyProject.c,52 :: 		SSPBUF = 0x07;  //RTC SQWOut register address
	MOVLW      7
	MOVWF      SSPBUF+0
;MyProject.c,53 :: 		while(SSPSTAT & 0x04); //Wait till transfer is complete
L_main5:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_main6
	GOTO       L_main5
L_main6:
;MyProject.c,55 :: 		SSPBUF = 0x90;  //Enable 1Hz clock
	MOVLW      144
	MOVWF      SSPBUF+0
;MyProject.c,56 :: 		while(SSPSTAT & 0x04); //Wait till transfer is complete
L_main7:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_main8
	GOTO       L_main7
L_main8:
;MyProject.c,58 :: 		PEN_bit = 1;    //Enable a stop condition
	BSF        PEN_bit+0, BitPos(PEN_bit+0)
;MyProject.c,59 :: 		while(PEN_bit); //wait for stop condition to complete
L_main9:
	BTFSS      PEN_bit+0, BitPos(PEN_bit+0)
	GOTO       L_main10
	GOTO       L_main9
L_main10:
;MyProject.c,61 :: 		I2C_update_time();
	CALL       _I2C_update_time+0
;MyProject.c,63 :: 		PORTC.F0 = 0; //Clear the T1CKI port pin
	BCF        PORTC+0, 0
;MyProject.c,64 :: 		TRISC.F0 = 1; //Configure T1CKI as an Input pin
	BSF        TRISC+0, 0
;MyProject.c,66 :: 		TMR1CS_bit = 1;//Use external clock. Configured as a Counter
	BSF        TMR1CS_bit+0, BitPos(TMR1CS_bit+0)
;MyProject.c,67 :: 		T1SYNC_bit = 1;//Do not synchronize the external clock
	BSF        T1SYNC_bit+0, BitPos(T1SYNC_bit+0)
;MyProject.c,69 :: 		TMR1IE_bit = 1;//Enable overflow interrupt
	BSF        TMR1IE_bit+0, BitPos(TMR1IE_bit+0)
;MyProject.c,70 :: 		PEIE_bit = 1;//Enable Peripheral Interrupt
	BSF        PEIE_bit+0, BitPos(PEIE_bit+0)
;MyProject.c,74 :: 		TMR1H = 0xFF;
	MOVLW      255
	MOVWF      TMR1H+0
;MyProject.c,75 :: 		TMR1L = timer_overflow_seconds;
	MOVF       _timer_overflow_seconds+0, 0
	MOVWF      TMR1L+0
;MyProject.c,77 :: 		TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
	BCF        TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
;MyProject.c,78 :: 		TMR1ON_bit = 1;//Turn on the Timer
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,82 :: 		SPI_init();
	CALL       _SPI_init+0
;MyProject.c,84 :: 		SPI_tx(0x09, 0xFF);  //Code B Dcoding enabled
	MOVLW      9
	MOVWF      FARG_SPI_tx+0
	MOVLW      255
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,85 :: 		SPI_tx(0x0B, 0x03); //scan limit to 4 digits
	MOVLW      11
	MOVWF      FARG_SPI_tx+0
	MOVLW      3
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,87 :: 		SPI_update();
	CALL       _SPI_update+0
;MyProject.c,89 :: 		SPI_tx(0x0C, 0x01); //disable sleep
	MOVLW      12
	MOVWF      FARG_SPI_tx+0
	MOVLW      1
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,93 :: 		TRISA.F0 = 1;
	BSF        TRISA+0, 0
;MyProject.c,94 :: 		PORTA.F0 = 1;
	BSF        PORTA+0, 0
;MyProject.c,95 :: 		ADCS0_bit = 1;//Set the clock to Fosc/8
	BSF        ADCS0_bit+0, BitPos(ADCS0_bit+0)
;MyProject.c,98 :: 		ADON_bit = 1; //turn on ADC module
	BSF        ADON_bit+0, BitPos(ADON_bit+0)
;MyProject.c,99 :: 		delay_ms(1);//A/D Acquisition time
	MOVLW      2
	MOVWF      R12+0
	MOVLW      75
	MOVWF      R13+0
L_main11:
	DECFSZ     R13+0, 1
	GOTO       L_main11
	DECFSZ     R12+0, 1
	GOTO       L_main11
;MyProject.c,100 :: 		ADCON0 = ADCON0 | 0x02;                     //start conversion
	BSF        ADCON0+0, 1
;MyProject.c,101 :: 		while(ADCON0 & 0x02); //wait for conversion
L_main12:
	BTFSS      ADCON0+0, 1
	GOTO       L_main13
	GOTO       L_main12
L_main13:
;MyProject.c,102 :: 		SPI_tx(0x0A,ADRESH * 15 / 128); //set the intensity. SPI_init() called above.
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	MOVF       ADRESH+0, 0
	MOVWF      R0+0
	MOVLW      15
	MOVWF      R4+0
	CALL       _Mul_8X8_U+0
	MOVLW      128
	MOVWF      R4+0
	CLRF       R4+1
	CALL       _Div_16x16_S+0
	MOVF       R0+0, 0
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,103 :: 		ADON_bit = 0; //turn offf ADC module
	BCF        ADON_bit+0, BitPos(ADON_bit+0)
;MyProject.c,107 :: 		ANS1_bit = 0; //Digital IO
	BCF        ANS1_bit+0, BitPos(ANS1_bit+0)
;MyProject.c,108 :: 		TRISA.F1 = 0;//Configure as Output
	BCF        TRISA+0, 1
;MyProject.c,109 :: 		PORTA.F1 = 1;//LED is ON
	BSF        PORTA+0, 1
;MyProject.c,112 :: 		SPBRG = 25; //for 4MHz OSC 9600 baud with 0.16% error
	MOVLW      25
	MOVWF      SPBRG+0
;MyProject.c,113 :: 		BRGH_bit = 1;//Enable high speed mode
	BSF        BRGH_bit+0, BitPos(BRGH_bit+0)
;MyProject.c,114 :: 		TXEN_bit = 1; //enable transmission
	BSF        TXEN_bit+0, BitPos(TXEN_bit+0)
;MyProject.c,115 :: 		CREN_bit = 1; //enable reception
	BSF        CREN_bit+0, BitPos(CREN_bit+0)
;MyProject.c,119 :: 		ANS12_bit = 0; //Digital IO
	BCF        ANS12_bit+0, BitPos(ANS12_bit+0)
;MyProject.c,120 :: 		ANS10_bit = 0; //Digital IO
	BCF        ANS10_bit+0, BitPos(ANS10_bit+0)
;MyProject.c,121 :: 		PORTB.F0 = 0;  //Clear Port Pin
	BCF        PORTB+0, 0
;MyProject.c,122 :: 		TRISB.F0 = 1;  //Configure as an input pin
	BSF        TRISB+0, 0
;MyProject.c,123 :: 		PORTB.F1 = 0;  //Clear Port Pin
	BCF        PORTB+0, 1
;MyProject.c,124 :: 		TRISB.F1 = 1;  //Configure as an input pin
	BSF        TRISB+0, 1
;MyProject.c,125 :: 		WPUB = 0x03; //enable only RB0 and RB1 pullup
	MOVLW      3
	MOVWF      WPUB+0
;MyProject.c,126 :: 		OPTION_REG = OPTION_REG & 0x7F; //Enable pull ups
	MOVLW      127
	ANDWF      OPTION_REG+0, 1
;MyProject.c,128 :: 		INTEDG_bit = 0; //Interrupt on falling edge of INT pin
	BCF        INTEDG_bit+0, BitPos(INTEDG_bit+0)
;MyProject.c,129 :: 		INTF_bit = 0;//Clear the external interrupt flag
	BCF        INTF_bit+0, BitPos(INTF_bit+0)
;MyProject.c,130 :: 		INTE_bit = 1;//Enable external interrupt
	BSF        INTE_bit+0, BitPos(INTE_bit+0)
;MyProject.c,132 :: 		while(1) {
L_main14:
;MyProject.c,133 :: 		PORTA.F1 = ~PORTA.F1;//Toggle MODE LED
	MOVLW      2
	XORWF      PORTA+0, 1
;MyProject.c,135 :: 		asm sleep;
	SLEEP
;MyProject.c,136 :: 		delay_ms(20);//Debounce
	MOVLW      26
	MOVWF      R12+0
	MOVLW      248
	MOVWF      R13+0
L_main16:
	DECFSZ     R13+0, 1
	GOTO       L_main16
	DECFSZ     R12+0, 1
	GOTO       L_main16
	NOP
;MyProject.c,138 :: 		INTF_bit = 0;//Clear the INT flag
	BCF        INTF_bit+0, BitPos(INTF_bit+0)
;MyProject.c,140 :: 		PORTA.F1 = ~PORTA.F1;//Toggle MODE LED
	MOVLW      2
	XORWF      PORTA+0, 1
;MyProject.c,143 :: 		if (TMR1IF_bit) {
	BTFSS      TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
	GOTO       L_main17
;MyProject.c,144 :: 		TMR1ON_bit = 0; //To prevent write contention turn off the timer
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,148 :: 		TMR1H = 0xFF;
	MOVLW      255
	MOVWF      TMR1H+0
;MyProject.c,149 :: 		TMR1L = SECONDS_OVERFLOW_VAL;
	MOVLW      196
	MOVWF      TMR1L+0
;MyProject.c,151 :: 		I2C_update_time();//Update the global registers
	CALL       _I2C_update_time+0
;MyProject.c,152 :: 		SPI_update(); //Update the MAX7219 driver
	CALL       _SPI_update+0
;MyProject.c,154 :: 		TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
	BCF        TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
;MyProject.c,155 :: 		TMR1ON_bit = 1;//Turn on the Timer
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,156 :: 		}
	GOTO       L_main18
L_main17:
;MyProject.c,158 :: 		else if (PORTB.F1) {
	BTFSS      PORTB+0, 1
	GOTO       L_main19
;MyProject.c,162 :: 		TMR1ON_bit = 0;//Turn off the timer
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,164 :: 		TMR1H = 0xFF;
	MOVLW      255
	MOVWF      TMR1H+0
;MyProject.c,165 :: 		TMR1L = SECONDS_OVERFLOW_VAL;
	MOVLW      196
	MOVWF      TMR1L+0
;MyProject.c,166 :: 		TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
	BCF        TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
;MyProject.c,169 :: 		SPI_init();
	CALL       _SPI_init+0
;MyProject.c,170 :: 		SPI_tx(0x01,0x00);//Digit 0 will display 0
	MOVLW      1
	MOVWF      FARG_SPI_tx+0
	CLRF       FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,172 :: 		SPI_tx(0x02,0x0A);
	MOVLW      2
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,173 :: 		SPI_tx(0x03,0x0A);
	MOVLW      3
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,174 :: 		SPI_tx(0x04,0x0A);
	MOVLW      4
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,177 :: 		rtc_hour_set = 0;
	CLRF       _rtc_hour_set+0
;MyProject.c,178 :: 		rtc_min_set = 0;
	CLRF       _rtc_min_set+0
;MyProject.c,180 :: 		while(1) {
L_main20:
;MyProject.c,183 :: 		asm sleep; //sleep till switch is clicked
	SLEEP
;MyProject.c,184 :: 		delay_ms(20);//Debounce
	MOVLW      26
	MOVWF      R12+0
	MOVLW      248
	MOVWF      R13+0
L_main22:
	DECFSZ     R13+0, 1
	GOTO       L_main22
	DECFSZ     R12+0, 1
	GOTO       L_main22
	NOP
;MyProject.c,185 :: 		INTF_bit = 0;//Clear the INT flag
	BCF        INTF_bit+0, BitPos(INTF_bit+0)
;MyProject.c,187 :: 		if (~PORTB.F1){
	BTFSC      PORTB+0, 1
	GOTO       L__main107
	BSF        3, 0
	GOTO       L__main108
L__main107:
	BCF        3, 0
L__main108:
	BTFSS      3, 0
	GOTO       L_main23
;MyProject.c,190 :: 		++time_set_digit_addr;
	INCF       _time_set_digit_addr+0, 1
;MyProject.c,191 :: 		if (time_set_digit_addr > 4){
	MOVF       _time_set_digit_addr+0, 0
	SUBLW      4
	BTFSC      STATUS+0, 0
	GOTO       L_main24
;MyProject.c,193 :: 		I2C_update_rtc(0);
	CLRF       FARG_I2C_update_rtc+0
	CALL       _I2C_update_rtc+0
;MyProject.c,194 :: 		TMR1ON_bit = 1;//Turn on the 60 seconds overflow
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,197 :: 		time_set_digit_addr = 1;
	MOVLW      1
	MOVWF      _time_set_digit_addr+0
;MyProject.c,198 :: 		rtc_temp_set = 0;
	CLRF       _rtc_temp_set+0
;MyProject.c,200 :: 		break; //Time Set Complete
	GOTO       L_main21
;MyProject.c,201 :: 		}
L_main24:
;MyProject.c,203 :: 		rtc_temp_set = 0;
	CLRF       _rtc_temp_set+0
;MyProject.c,205 :: 		SPI_tx(time_set_digit_addr,0x00);//Digit 0 will display 0
	MOVF       _time_set_digit_addr+0, 0
	MOVWF      FARG_SPI_tx+0
	CLRF       FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,208 :: 		} else {
	GOTO       L_main25
L_main23:
;MyProject.c,212 :: 		switch(time_set_digit_addr) {
	GOTO       L_main26
;MyProject.c,215 :: 		case 1:  if (++rtc_temp_set > 2)
L_main28:
	INCF       _rtc_temp_set+0, 1
	MOVF       _rtc_temp_set+0, 0
	SUBLW      2
	BTFSC      STATUS+0, 0
	GOTO       L_main29
;MyProject.c,216 :: 		rtc_temp_set = 0;//Hour MSB [0 - 2]
	CLRF       _rtc_temp_set+0
L_main29:
;MyProject.c,217 :: 		rtc_hour_set = rtc_temp_set << 4;
	MOVF       _rtc_temp_set+0, 0
	MOVWF      _rtc_hour_set+0
	RLF        _rtc_hour_set+0, 1
	BCF        _rtc_hour_set+0, 0
	RLF        _rtc_hour_set+0, 1
	BCF        _rtc_hour_set+0, 0
	RLF        _rtc_hour_set+0, 1
	BCF        _rtc_hour_set+0, 0
	RLF        _rtc_hour_set+0, 1
	BCF        _rtc_hour_set+0, 0
;MyProject.c,218 :: 		break;
	GOTO       L_main27
;MyProject.c,219 :: 		case 2:  ++rtc_temp_set;
L_main30:
	INCF       _rtc_temp_set+0, 1
;MyProject.c,220 :: 		if ((rtc_hour_set >> 4) == 2 && rtc_temp_set > 3)
	MOVF       _rtc_hour_set+0, 0
	MOVWF      R1+0
	RRF        R1+0, 1
	BCF        R1+0, 7
	RRF        R1+0, 1
	BCF        R1+0, 7
	RRF        R1+0, 1
	BCF        R1+0, 7
	RRF        R1+0, 1
	BCF        R1+0, 7
	MOVF       R1+0, 0
	XORLW      2
	BTFSS      STATUS+0, 2
	GOTO       L_main33
	MOVF       _rtc_temp_set+0, 0
	SUBLW      3
	BTFSC      STATUS+0, 0
	GOTO       L_main33
L__main105:
;MyProject.c,221 :: 		rtc_temp_set = 0;//Hour can be max 23
	CLRF       _rtc_temp_set+0
	GOTO       L_main34
L_main33:
;MyProject.c,222 :: 		else if (rtc_temp_set > 9)
	MOVF       _rtc_temp_set+0, 0
	SUBLW      9
	BTFSC      STATUS+0, 0
	GOTO       L_main35
;MyProject.c,223 :: 		rtc_temp_set = 0;//Hour LSB [0 - 9]
	CLRF       _rtc_temp_set+0
L_main35:
L_main34:
;MyProject.c,224 :: 		rtc_hour_set &= 0xF0;
	MOVLW      240
	ANDWF      _rtc_hour_set+0, 1
;MyProject.c,225 :: 		rtc_hour_set |= rtc_temp_set;
	MOVF       _rtc_temp_set+0, 0
	IORWF      _rtc_hour_set+0, 1
;MyProject.c,226 :: 		break;
	GOTO       L_main27
;MyProject.c,230 :: 		case 3:  if (++rtc_temp_set > 5)
L_main36:
	INCF       _rtc_temp_set+0, 1
	MOVF       _rtc_temp_set+0, 0
	SUBLW      5
	BTFSC      STATUS+0, 0
	GOTO       L_main37
;MyProject.c,231 :: 		rtc_temp_set = 0;//Min MSB [0 - 5]
	CLRF       _rtc_temp_set+0
L_main37:
;MyProject.c,232 :: 		rtc_min_set = rtc_temp_set << 4;
	MOVF       _rtc_temp_set+0, 0
	MOVWF      _rtc_min_set+0
	RLF        _rtc_min_set+0, 1
	BCF        _rtc_min_set+0, 0
	RLF        _rtc_min_set+0, 1
	BCF        _rtc_min_set+0, 0
	RLF        _rtc_min_set+0, 1
	BCF        _rtc_min_set+0, 0
	RLF        _rtc_min_set+0, 1
	BCF        _rtc_min_set+0, 0
;MyProject.c,233 :: 		break;
	GOTO       L_main27
;MyProject.c,234 :: 		case 4:  if (++rtc_temp_set > 9)
L_main38:
	INCF       _rtc_temp_set+0, 1
	MOVF       _rtc_temp_set+0, 0
	SUBLW      9
	BTFSC      STATUS+0, 0
	GOTO       L_main39
;MyProject.c,235 :: 		rtc_temp_set = 0;//Min LSB [0 - 9]
	CLRF       _rtc_temp_set+0
L_main39:
;MyProject.c,236 :: 		rtc_min_set &= 0xF0;
	MOVLW      240
	ANDWF      _rtc_min_set+0, 1
;MyProject.c,237 :: 		rtc_min_set |= rtc_temp_set;
	MOVF       _rtc_temp_set+0, 0
	IORWF      _rtc_min_set+0, 1
;MyProject.c,238 :: 		break;
	GOTO       L_main27
;MyProject.c,240 :: 		}
L_main26:
	MOVF       _time_set_digit_addr+0, 0
	XORLW      1
	BTFSC      STATUS+0, 2
	GOTO       L_main28
	MOVF       _time_set_digit_addr+0, 0
	XORLW      2
	BTFSC      STATUS+0, 2
	GOTO       L_main30
	MOVF       _time_set_digit_addr+0, 0
	XORLW      3
	BTFSC      STATUS+0, 2
	GOTO       L_main36
	MOVF       _time_set_digit_addr+0, 0
	XORLW      4
	BTFSC      STATUS+0, 2
	GOTO       L_main38
L_main27:
;MyProject.c,242 :: 		SPI_tx(time_set_digit_addr,rtc_temp_set);
	MOVF       _time_set_digit_addr+0, 0
	MOVWF      FARG_SPI_tx+0
	MOVF       _rtc_temp_set+0, 0
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,243 :: 		}
L_main25:
;MyProject.c,244 :: 		}
	GOTO       L_main20
L_main21:
;MyProject.c,245 :: 		}
	GOTO       L_main40
L_main19:
;MyProject.c,259 :: 		TMR1ON_bit = 0;//Turn off the timer
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,260 :: 		TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
	BCF        TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
;MyProject.c,263 :: 		SPI_init();
	CALL       _SPI_init+0
;MyProject.c,264 :: 		SPI_tx(0x01,0x0A);
	MOVLW      1
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,265 :: 		SPI_tx(0x02,0x08);
	MOVLW      2
	MOVWF      FARG_SPI_tx+0
	MOVLW      8
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,266 :: 		SPI_tx(0x03,0x08);
	MOVLW      3
	MOVWF      FARG_SPI_tx+0
	MOVLW      8
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,267 :: 		SPI_tx(0x04,0x0A);
	MOVLW      4
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,269 :: 		ADON_bit = 1; //turn on ADC module
	BSF        ADON_bit+0, BitPos(ADON_bit+0)
;MyProject.c,271 :: 		while(~INTF_bit){ //Wait till interrupt
L_main41:
	BTFSC      INTF_bit+0, BitPos(INTF_bit+0)
	GOTO       L__main109
	BSF        3, 0
	GOTO       L__main110
L__main109:
	BCF        3, 0
L__main110:
	BTFSS      3, 0
	GOTO       L_main42
;MyProject.c,272 :: 		delay_ms(1);//A/D Acquisition time
	MOVLW      2
	MOVWF      R12+0
	MOVLW      75
	MOVWF      R13+0
L_main43:
	DECFSZ     R13+0, 1
	GOTO       L_main43
	DECFSZ     R12+0, 1
	GOTO       L_main43
;MyProject.c,273 :: 		ADCON0 = ADCON0 | 0x02;                     //start conversion
	BSF        ADCON0+0, 1
;MyProject.c,274 :: 		while(ADCON0 & 0x02); //wait for conversion
L_main44:
	BTFSS      ADCON0+0, 1
	GOTO       L_main45
	GOTO       L_main44
L_main45:
;MyProject.c,275 :: 		SPI_tx(0x0A,ADRESH * 15 / 128); //set the intensity. SPI_init() called above.
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	MOVF       ADRESH+0, 0
	MOVWF      R0+0
	MOVLW      15
	MOVWF      R4+0
	CALL       _Mul_8X8_U+0
	MOVLW      128
	MOVWF      R4+0
	CLRF       R4+1
	CALL       _Div_16x16_S+0
	MOVF       R0+0, 0
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,276 :: 		}
	GOTO       L_main41
L_main42:
;MyProject.c,278 :: 		delay_ms(20); //Debounce
	MOVLW      26
	MOVWF      R12+0
	MOVLW      248
	MOVWF      R13+0
L_main46:
	DECFSZ     R13+0, 1
	GOTO       L_main46
	DECFSZ     R12+0, 1
	GOTO       L_main46
	NOP
;MyProject.c,279 :: 		INTF_bit = 0;//Clear INT flag
	BCF        INTF_bit+0, BitPos(INTF_bit+0)
;MyProject.c,282 :: 		ADON_bit = 0;
	BCF        ADON_bit+0, BitPos(ADON_bit+0)
;MyProject.c,286 :: 		if (~PORTB.F1) {
	BTFSC      PORTB+0, 1
	GOTO       L__main111
	BSF        3, 0
	GOTO       L__main112
L__main111:
	BCF        3, 0
L__main112:
	BTFSS      3, 0
	GOTO       L_main47
;MyProject.c,289 :: 		SPI_init();
	CALL       _SPI_init+0
;MyProject.c,290 :: 		SPI_tx(0x01,0x0A);
	MOVLW      1
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,291 :: 		SPI_tx(0x02,0x0A);
	MOVLW      2
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,292 :: 		SPI_tx(0x03,0x0A);
	MOVLW      3
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,293 :: 		SPI_tx(0x04,0x0A);
	MOVLW      4
	MOVWF      FARG_SPI_tx+0
	MOVLW      10
	MOVWF      FARG_SPI_tx+0
	CALL       _SPI_tx+0
;MyProject.c,295 :: 		SPEN_bit = 1;//enable uart
	BSF        SPEN_bit+0, BitPos(SPEN_bit+0)
;MyProject.c,297 :: 		TXREG = 1;//Send time update request
	MOVLW      1
	MOVWF      TXREG+0
;MyProject.c,298 :: 		while(!TRMT_bit); //wait for transmission to complete
L_main48:
	BTFSC      TRMT_bit+0, BitPos(TRMT_bit+0)
	GOTO       L_main49
	GOTO       L_main48
L_main49:
;MyProject.c,300 :: 		INTF_bit = 0; //Clear external INT flag (Button INT)
	BCF        INTF_bit+0, BitPos(INTF_bit+0)
;MyProject.c,304 :: 		while(!RCIF_bit && ~INTF_bit); //wait for reception (HOUR)
L_main50:
	BTFSC      RCIF_bit+0, BitPos(RCIF_bit+0)
	GOTO       L_main51
	BTFSC      INTF_bit+0, BitPos(INTF_bit+0)
	GOTO       L__main113
	BSF        3, 0
	GOTO       L__main114
L__main113:
	BCF        3, 0
L__main114:
	BTFSS      3, 0
	GOTO       L_main51
L__main104:
	GOTO       L_main50
L_main51:
;MyProject.c,306 :: 		if (INTF_bit){
	BTFSS      INTF_bit+0, BitPos(INTF_bit+0)
	GOTO       L_main54
;MyProject.c,308 :: 		delay_ms(20);//Debounce wait
	MOVLW      26
	MOVWF      R12+0
	MOVLW      248
	MOVWF      R13+0
L_main55:
	DECFSZ     R13+0, 1
	GOTO       L_main55
	DECFSZ     R12+0, 1
	GOTO       L_main55
	NOP
;MyProject.c,309 :: 		I2C_update_time(); //Sync microcontroller time from DS1307
	CALL       _I2C_update_time+0
;MyProject.c,310 :: 		INTF_bit = 0;//Clear INT flag
	BCF        INTF_bit+0, BitPos(INTF_bit+0)
;MyProject.c,312 :: 		} else {
	GOTO       L_main56
L_main54:
;MyProject.c,314 :: 		rtc_hour_set = RCREG;
	MOVF       RCREG+0, 0
	MOVWF      _rtc_hour_set+0
;MyProject.c,316 :: 		while(!RCIF_bit); //wait for reception (MIN)
L_main57:
	BTFSC      RCIF_bit+0, BitPos(RCIF_bit+0)
	GOTO       L_main58
	GOTO       L_main57
L_main58:
;MyProject.c,317 :: 		rtc_min_set = RCREG;
	MOVF       RCREG+0, 0
	MOVWF      _rtc_min_set+0
;MyProject.c,319 :: 		while(!RCIF_bit); //wait for reception (SEC)
L_main59:
	BTFSC      RCIF_bit+0, BitPos(RCIF_bit+0)
	GOTO       L_main60
	GOTO       L_main59
L_main60:
;MyProject.c,320 :: 		bcd = RCREG & 0x7F; //CH bit in DS1307 should be 0
	MOVLW      127
	ANDWF      RCREG+0, 0
	MOVWF      _bcd+0
	CLRF       _bcd+1
	MOVLW      0
	ANDWF      _bcd+1, 1
	MOVLW      0
	MOVWF      _bcd+1
;MyProject.c,321 :: 		lower_nib = bcd & 0x0F;
	MOVLW      15
	ANDWF      _bcd+0, 0
	MOVWF      FLOC__main+0
	MOVF       _bcd+1, 0
	MOVWF      FLOC__main+1
	MOVLW      0
	ANDWF      FLOC__main+1, 1
	MOVF       FLOC__main+0, 0
	MOVWF      _lower_nib+0
	MOVF       FLOC__main+1, 0
	MOVWF      _lower_nib+1
;MyProject.c,322 :: 		timer_overflow_seconds = (bcd >> 4)*10;
	MOVF       _bcd+0, 0
	MOVWF      R0+0
	MOVF       _bcd+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVF       R0+0, 0
	MOVWF      _timer_overflow_seconds+0
	MOVF       R0+1, 0
	MOVWF      _timer_overflow_seconds+1
;MyProject.c,323 :: 		timer_overflow_seconds += lower_nib;
	MOVF       FLOC__main+0, 0
	ADDWF      R0+0, 1
	MOVF       FLOC__main+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _timer_overflow_seconds+0
	MOVF       R0+1, 0
	MOVWF      _timer_overflow_seconds+1
;MyProject.c,324 :: 		timer_overflow_seconds = 0xFF - (60 - timer_overflow_seconds);
	MOVF       R0+0, 0
	SUBLW      60
	MOVWF      R0+0
	MOVF       R0+1, 0
	BTFSS      STATUS+0, 0
	ADDLW      1
	CLRF       R0+1
	SUBWF      R0+1, 1
	MOVF       R0+0, 0
	SUBLW      255
	MOVWF      _timer_overflow_seconds+0
	MOVF       R0+1, 0
	BTFSS      STATUS+0, 0
	ADDLW      1
	CLRF       _timer_overflow_seconds+1
	SUBWF      _timer_overflow_seconds+1, 1
;MyProject.c,326 :: 		I2C_update_rtc(bcd);//Update DS1307 time registers with seconds
	MOVF       _bcd+0, 0
	MOVWF      FARG_I2C_update_rtc+0
	CALL       _I2C_update_rtc+0
;MyProject.c,327 :: 		}
L_main56:
;MyProject.c,330 :: 		} else {
	GOTO       L_main61
L_main47:
;MyProject.c,332 :: 		I2C_update_time(); //Sync microcontroller time from DS1307
	CALL       _I2C_update_time+0
;MyProject.c,333 :: 		}
L_main61:
;MyProject.c,336 :: 		TMR1H = 0xFF;
	MOVLW      255
	MOVWF      TMR1H+0
;MyProject.c,337 :: 		TMR1L = timer_overflow_seconds;
	MOVF       _timer_overflow_seconds+0, 0
	MOVWF      TMR1L+0
;MyProject.c,338 :: 		TMR1IF_bit = 0;//Clear Timer1/Counter1 overflow interrupt
	BCF        TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
;MyProject.c,339 :: 		TMR1ON_bit = 1;//Turn on the 60 seconds overflow
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;MyProject.c,341 :: 		SPI_update();//Update the display
	CALL       _SPI_update+0
;MyProject.c,342 :: 		}
L_main40:
L_main18:
;MyProject.c,343 :: 		}
	GOTO       L_main14
;MyProject.c,344 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_SPI_init:

;MyProject.c,346 :: 		void SPI_init(void){
;MyProject.c,347 :: 		SSPEN_bit = 0;//Turn off the I2C mode
	BCF        SSPEN_bit+0, BitPos(SSPEN_bit+0)
;MyProject.c,348 :: 		TRISC.F3 = 0;  //configured as O/P SCK
	BCF        TRISC+0, 3
;MyProject.c,349 :: 		SMP_bit = 1; //Input data sampled at end of data output time
	BSF        SMP_bit+0, BitPos(SMP_bit+0)
;MyProject.c,350 :: 		SSPCON = 0x22; //Enable SPI with Fosc/64 Clock
	MOVLW      34
	MOVWF      SSPCON+0
;MyProject.c,351 :: 		}
L_end_SPI_init:
	RETURN
; end of _SPI_init

_SPI_tx:

;MyProject.c,353 :: 		void SPI_tx(unsigned char addr,unsigned char dat)
;MyProject.c,355 :: 		PORTA.F5 = 0;
	BCF        PORTA+0, 5
;MyProject.c,356 :: 		SSPBUF = addr;
	MOVF       FARG_SPI_tx_addr+0, 0
	MOVWF      SSPBUF+0
;MyProject.c,357 :: 		while(~BF_bit);
L_SPI_tx62:
	BTFSC      BF_bit+0, BitPos(BF_bit+0)
	GOTO       L__SPI_tx117
	BSF        3, 0
	GOTO       L__SPI_tx118
L__SPI_tx117:
	BCF        3, 0
L__SPI_tx118:
	BTFSS      3, 0
	GOTO       L_SPI_tx63
	GOTO       L_SPI_tx62
L_SPI_tx63:
;MyProject.c,358 :: 		SSPBUF = dat;
	MOVF       FARG_SPI_tx_dat+0, 0
	MOVWF      SSPBUF+0
;MyProject.c,359 :: 		while(~BF_bit);
L_SPI_tx64:
	BTFSC      BF_bit+0, BitPos(BF_bit+0)
	GOTO       L__SPI_tx119
	BSF        3, 0
	GOTO       L__SPI_tx120
L__SPI_tx119:
	BCF        3, 0
L__SPI_tx120:
	BTFSS      3, 0
	GOTO       L_SPI_tx65
	GOTO       L_SPI_tx64
L_SPI_tx65:
;MyProject.c,360 :: 		PORTA.F5 = 1;
	BSF        PORTA+0, 5
;MyProject.c,361 :: 		}
L_end_SPI_tx:
	RETURN
; end of _SPI_tx

_SPI_update:

;MyProject.c,363 :: 		void SPI_update() {
;MyProject.c,364 :: 		SPI_init();
	CALL       _SPI_init+0
;MyProject.c,366 :: 		SPI_tx(0x01,rtc_hour_set >> 4);//Digit 0 Hour MSB
	MOVLW      1
	MOVWF      FARG_SPI_tx_addr+0
	MOVF       _rtc_hour_set+0, 0
	MOVWF      FARG_SPI_tx_dat+0
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	CALL       _SPI_tx+0
;MyProject.c,367 :: 		SPI_tx(0x02,rtc_hour_set & 0x0F);//Digit 1 Hour LSB
	MOVLW      2
	MOVWF      FARG_SPI_tx_addr+0
	MOVLW      15
	ANDWF      _rtc_hour_set+0, 0
	MOVWF      FARG_SPI_tx_dat+0
	CALL       _SPI_tx+0
;MyProject.c,368 :: 		SPI_tx(0x03,rtc_min_set >> 4);//Digit 2 Minute MSB
	MOVLW      3
	MOVWF      FARG_SPI_tx_addr+0
	MOVF       _rtc_min_set+0, 0
	MOVWF      FARG_SPI_tx_dat+0
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	RRF        FARG_SPI_tx_dat+0, 1
	BCF        FARG_SPI_tx_dat+0, 7
	CALL       _SPI_tx+0
;MyProject.c,369 :: 		SPI_tx(0x04,rtc_min_set & 0x0F);//Digit 3 Minute LSB
	MOVLW      4
	MOVWF      FARG_SPI_tx_addr+0
	MOVLW      15
	ANDWF      _rtc_min_set+0, 0
	MOVWF      FARG_SPI_tx_dat+0
	CALL       _SPI_tx+0
;MyProject.c,371 :: 		}
L_end_SPI_update:
	RETURN
; end of _SPI_update

_I2C_init:

;MyProject.c,375 :: 		void I2C_init(){
;MyProject.c,376 :: 		SSPEN_bit = 0;  //Clear MSSP
	BCF        SSPEN_bit+0, BitPos(SSPEN_bit+0)
;MyProject.c,377 :: 		TRISC.F3 = 1;  //configured as I/P SCL
	BSF        TRISC+0, 3
;MyProject.c,378 :: 		SMP_bit = 0;
	BCF        SMP_bit+0, BitPos(SMP_bit+0)
;MyProject.c,379 :: 		SSPCON = 0x28;   //Configure in I2C master mode
	MOVLW      40
	MOVWF      SSPCON+0
;MyProject.c,380 :: 		}
L_end_I2C_init:
	RETURN
; end of _I2C_init

_I2C_update_rtc:

;MyProject.c,382 :: 		void I2C_update_rtc(unsigned char sec){
;MyProject.c,383 :: 		I2C_init();
	CALL       _I2C_init+0
;MyProject.c,385 :: 		SEN_bit = 1; //enable start condition on I2C Bus
	BSF        SEN_bit+0, BitPos(SEN_bit+0)
;MyProject.c,386 :: 		while(SEN_bit); //wait for start condition to complete
L_I2C_update_rtc66:
	BTFSS      SEN_bit+0, BitPos(SEN_bit+0)
	GOTO       L_I2C_update_rtc67
	GOTO       L_I2C_update_rtc66
L_I2C_update_rtc67:
;MyProject.c,388 :: 		SSPBUF = 0xD0; //RTC address with Write command
	MOVLW      208
	MOVWF      SSPBUF+0
;MyProject.c,389 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_rtc68:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_rtc69
	GOTO       L_I2C_update_rtc68
L_I2C_update_rtc69:
;MyProject.c,391 :: 		SSPBUF = 0x00; //RTC seconds register address
	CLRF       SSPBUF+0
;MyProject.c,392 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_rtc70:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_rtc71
	GOTO       L_I2C_update_rtc70
L_I2C_update_rtc71:
;MyProject.c,395 :: 		SSPBUF = sec;//Seconds register
	MOVF       FARG_I2C_update_rtc_sec+0, 0
	MOVWF      SSPBUF+0
;MyProject.c,396 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_rtc72:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_rtc73
	GOTO       L_I2C_update_rtc72
L_I2C_update_rtc73:
;MyProject.c,399 :: 		SSPBUF = rtc_min_set; //Update the minutes register
	MOVF       _rtc_min_set+0, 0
	MOVWF      SSPBUF+0
;MyProject.c,400 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_rtc74:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_rtc75
	GOTO       L_I2C_update_rtc74
L_I2C_update_rtc75:
;MyProject.c,403 :: 		SSPBUF = rtc_hour_set; //Update the hour register
	MOVF       _rtc_hour_set+0, 0
	MOVWF      SSPBUF+0
;MyProject.c,404 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_rtc76:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_rtc77
	GOTO       L_I2C_update_rtc76
L_I2C_update_rtc77:
;MyProject.c,407 :: 		PEN_bit = 1;//Enable stop condition
	BSF        PEN_bit+0, BitPos(PEN_bit+0)
;MyProject.c,408 :: 		while(PEN_bit); //Wait for stop condition to complete
L_I2C_update_rtc78:
	BTFSS      PEN_bit+0, BitPos(PEN_bit+0)
	GOTO       L_I2C_update_rtc79
	GOTO       L_I2C_update_rtc78
L_I2C_update_rtc79:
;MyProject.c,409 :: 		}
L_end_I2C_update_rtc:
	RETURN
; end of _I2C_update_rtc

_I2C_update_time:

;MyProject.c,411 :: 		void I2C_update_time(){
;MyProject.c,412 :: 		I2C_init();
	CALL       _I2C_init+0
;MyProject.c,414 :: 		SEN_bit = 1; //enable start condition on I2C Bus
	BSF        SEN_bit+0, BitPos(SEN_bit+0)
;MyProject.c,415 :: 		while(SEN_bit); //wait for start condition to complete
L_I2C_update_time80:
	BTFSS      SEN_bit+0, BitPos(SEN_bit+0)
	GOTO       L_I2C_update_time81
	GOTO       L_I2C_update_time80
L_I2C_update_time81:
;MyProject.c,417 :: 		SSPBUF = 0xD0; //RTC address with Write command
	MOVLW      208
	MOVWF      SSPBUF+0
;MyProject.c,418 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_time82:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_time83
	GOTO       L_I2C_update_time82
L_I2C_update_time83:
;MyProject.c,420 :: 		SSPBUF = 0x00; //RTC seconds register address
	CLRF       SSPBUF+0
;MyProject.c,421 :: 		while(SSPSTAT & 0x04); //Wait for transfer to complete
L_I2C_update_time84:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_time85
	GOTO       L_I2C_update_time84
L_I2C_update_time85:
;MyProject.c,423 :: 		RSEN_bit = 1; //Repeat start
	BSF        RSEN_bit+0, BitPos(RSEN_bit+0)
;MyProject.c,424 :: 		while(RSEN_bit);//Wait for Repeat Start condition to complete
L_I2C_update_time86:
	BTFSS      RSEN_bit+0, BitPos(RSEN_bit+0)
	GOTO       L_I2C_update_time87
	GOTO       L_I2C_update_time86
L_I2C_update_time87:
;MyProject.c,426 :: 		SSPBUF = 0xD1; //RTC add with read command
	MOVLW      209
	MOVWF      SSPBUF+0
;MyProject.c,427 :: 		while(SSPSTAT & 0x04);//Wait for transfer to complete
L_I2C_update_time88:
	BTFSS      SSPSTAT+0, 2
	GOTO       L_I2C_update_time89
	GOTO       L_I2C_update_time88
L_I2C_update_time89:
;MyProject.c,430 :: 		RCEN_bit = 1; //Receive enable
	BSF        RCEN_bit+0, BitPos(RCEN_bit+0)
;MyProject.c,431 :: 		while(RCEN_bit);//wait till the data is received
L_I2C_update_time90:
	BTFSS      RCEN_bit+0, BitPos(RCEN_bit+0)
	GOTO       L_I2C_update_time91
	GOTO       L_I2C_update_time90
L_I2C_update_time91:
;MyProject.c,433 :: 		ACKDT_bit = 0;
	BCF        ACKDT_bit+0, BitPos(ACKDT_bit+0)
;MyProject.c,434 :: 		ACKEN_bit = 1;//Send ACKDT ack
	BSF        ACKEN_bit+0, BitPos(ACKEN_bit+0)
;MyProject.c,435 :: 		while(ACKEN_bit); //ACK
L_I2C_update_time92:
	BTFSS      ACKEN_bit+0, BitPos(ACKEN_bit+0)
	GOTO       L_I2C_update_time93
	GOTO       L_I2C_update_time92
L_I2C_update_time93:
;MyProject.c,438 :: 		bcd = SSPBUF;
	MOVF       SSPBUF+0, 0
	MOVWF      _bcd+0
	CLRF       _bcd+1
;MyProject.c,439 :: 		lower_nib = bcd & 0x0F;
	MOVLW      15
	ANDWF      _bcd+0, 0
	MOVWF      FLOC__I2C_update_time+0
	MOVF       _bcd+1, 0
	MOVWF      FLOC__I2C_update_time+1
	MOVLW      0
	ANDWF      FLOC__I2C_update_time+1, 1
	MOVF       FLOC__I2C_update_time+0, 0
	MOVWF      _lower_nib+0
	MOVF       FLOC__I2C_update_time+1, 0
	MOVWF      _lower_nib+1
;MyProject.c,440 :: 		timer_overflow_seconds = (bcd >> 4)*10;
	MOVF       _bcd+0, 0
	MOVWF      R0+0
	MOVF       _bcd+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVF       R0+0, 0
	MOVWF      _timer_overflow_seconds+0
	MOVF       R0+1, 0
	MOVWF      _timer_overflow_seconds+1
;MyProject.c,441 :: 		timer_overflow_seconds += lower_nib;
	MOVF       FLOC__I2C_update_time+0, 0
	ADDWF      R0+0, 1
	MOVF       FLOC__I2C_update_time+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _timer_overflow_seconds+0
	MOVF       R0+1, 0
	MOVWF      _timer_overflow_seconds+1
;MyProject.c,442 :: 		timer_overflow_seconds = 0xFF - (60 - timer_overflow_seconds);
	MOVF       R0+0, 0
	SUBLW      60
	MOVWF      R0+0
	MOVF       R0+1, 0
	BTFSS      STATUS+0, 0
	ADDLW      1
	CLRF       R0+1
	SUBWF      R0+1, 1
	MOVF       R0+0, 0
	SUBLW      255
	MOVWF      _timer_overflow_seconds+0
	MOVF       R0+1, 0
	BTFSS      STATUS+0, 0
	ADDLW      1
	CLRF       _timer_overflow_seconds+1
	SUBWF      _timer_overflow_seconds+1, 1
;MyProject.c,446 :: 		RCEN_bit = 1; //Receive enable
	BSF        RCEN_bit+0, BitPos(RCEN_bit+0)
;MyProject.c,447 :: 		while(RCEN_bit);//wait till the data is received
L_I2C_update_time94:
	BTFSS      RCEN_bit+0, BitPos(RCEN_bit+0)
	GOTO       L_I2C_update_time95
	GOTO       L_I2C_update_time94
L_I2C_update_time95:
;MyProject.c,449 :: 		ACKDT_bit = 0;
	BCF        ACKDT_bit+0, BitPos(ACKDT_bit+0)
;MyProject.c,450 :: 		ACKEN_bit = 1;//Send ACKDT ack
	BSF        ACKEN_bit+0, BitPos(ACKEN_bit+0)
;MyProject.c,451 :: 		while(ACKEN_bit); //ACK
L_I2C_update_time96:
	BTFSS      ACKEN_bit+0, BitPos(ACKEN_bit+0)
	GOTO       L_I2C_update_time97
	GOTO       L_I2C_update_time96
L_I2C_update_time97:
;MyProject.c,453 :: 		rtc_min_set = SSPBUF; //Update the minutes global variable
	MOVF       SSPBUF+0, 0
	MOVWF      _rtc_min_set+0
;MyProject.c,456 :: 		RCEN_bit = 1; //Receive enable
	BSF        RCEN_bit+0, BitPos(RCEN_bit+0)
;MyProject.c,457 :: 		while(RCEN_bit);//wait till the data is received
L_I2C_update_time98:
	BTFSS      RCEN_bit+0, BitPos(RCEN_bit+0)
	GOTO       L_I2C_update_time99
	GOTO       L_I2C_update_time98
L_I2C_update_time99:
;MyProject.c,459 :: 		ACKDT_bit = 1;//No more registers to read so send NACK
	BSF        ACKDT_bit+0, BitPos(ACKDT_bit+0)
;MyProject.c,460 :: 		ACKEN_bit = 1;//Send the ACKDT to slave
	BSF        ACKEN_bit+0, BitPos(ACKEN_bit+0)
;MyProject.c,461 :: 		while(ACKEN_bit); //Wait for NACK to completely transfer
L_I2C_update_time100:
	BTFSS      ACKEN_bit+0, BitPos(ACKEN_bit+0)
	GOTO       L_I2C_update_time101
	GOTO       L_I2C_update_time100
L_I2C_update_time101:
;MyProject.c,463 :: 		rtc_hour_set = SSPBUF; //Update the hour global variable
	MOVF       SSPBUF+0, 0
	MOVWF      _rtc_hour_set+0
;MyProject.c,465 :: 		PEN_bit = 1;//Enable stop condition
	BSF        PEN_bit+0, BitPos(PEN_bit+0)
;MyProject.c,466 :: 		while(PEN_bit); //Wait for stop condition to complete
L_I2C_update_time102:
	BTFSS      PEN_bit+0, BitPos(PEN_bit+0)
	GOTO       L_I2C_update_time103
	GOTO       L_I2C_update_time102
L_I2C_update_time103:
;MyProject.c,467 :: 		}
L_end_I2C_update_time:
	RETURN
; end of _I2C_update_time
