#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"


#define RS 1    /* BIT0 mask */
#define RW 2    /* BIT1 mask */
#define EN 4    /* BIT2 mask */
void keypad_init(void);
void keypad_disable(void);
char keypad_getkey(void);
void LCD_nibble_write(unsigned char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void delayMs(int n);
char hexKeys[] = {0x0,'A','3','2','1','B','6','5','4','C','9','8','7','D','#','0','*'};
char keypressed;
bool modeSelect;
unsigned char key;
void delayUs(int n);

void UART0_init(void);
void UART0_Transmit_Poll(uint8_t);
uint8_t UART0_Receive_Poll(void);

//mode1
void ADC0_init(void);
short int result1;
short int result2;
bool alt = true;

//mode2/5
void LED_init(void);
void Trigger_Timer2_init(void);
void Capture_Timer2_init(void);
void PWM_init25(void);
char buffer5[7] = {"0900\r\n"};
volatile uint32_t pulse_width;
volatile float distance;
int tmpInt1;
float tmpFrac;
int tmpInt2;
volatile uint32_t cont[2];
short int result;
char buffer[30];
#define mod 14999



//mode 3/4
void UART0Mode34_init(void);
void UART0Tx(char c);
void UART0_puts(char* s);
void PWM_init(void);
short int result;
short int prevresult = -1;
char buffer4[7] = {"0000\r\n"};
char buffer3[4] = {"1\r\n"};
int mode;
int number = 0;
int i = 0;


int main(void) {

    BOARD_InitBootClocks();

    __disable_irq();
    UART0_init();
    keypad_init();
    PWM_init();
    LCD_init();
    __enable_irq();

    short int i = 0;
    bool up = true;

    while(1) {
    	if(mode == 1){
    		if(hexKeys[keypad_getkey()] == '#'){
				UART0_init();
			    NVIC_DisableIRQ(15);		//Disable IRQ 15
			}

    	}
    	if(mode == 2){
    		if(hexKeys[keypad_getkey()] == '#'){
				UART0_init();
			}
    	}
    	if(mode == 3){

    		if(hexKeys[keypad_getkey()] == '#'){
				UART0_init();
			}

    		result = atoi(buffer3);        /* read conversion result and clear COCO flag */

    		if(prevresult != result){
				UART0_Transmit_Poll('S');
				UART0_Transmit_Poll('p');
				UART0_Transmit_Poll('e');
				UART0_Transmit_Poll('e');
				UART0_Transmit_Poll('d');
				UART0_Transmit_Poll(':');
				UART0_Transmit_Poll(' ');
				UART0_puts(buffer3);
    		}
    		prevresult = result;

    		if(result > 0 && result <= 4)
    			delayMs(7/result);
    		else if(result <= 0)
    			delayMs(7/1);
    		else if(result > 4)
    			delayMs(7/4);

    		//servo angle
    		TPM0->CONTROLS[1].CnV = 1620 + i*(5940/1800);

    		if(up){
    			if(i >= 1800){
    				up = false;
    			}
    			i++;
    		}else{
    			if(i <= 0){
    				up = true;
    			}
    			i--;
		  }
    	}
    	if(mode == 4){
    		if(hexKeys[keypad_getkey()] == '#'){
    			UART0_init();
    		}
    		result = atoi(buffer4);        /* read conversion result and clear COCO flag */

    		if(prevresult != result){
				UART0_Transmit_Poll('A');
				UART0_Transmit_Poll('n');
				UART0_Transmit_Poll('g');
				UART0_Transmit_Poll('l');
				UART0_Transmit_Poll('e');
				UART0_Transmit_Poll(':');
				UART0_Transmit_Poll(' ');

				UART0_puts(buffer4);
    		}
    		prevresult = result;
    		delayMs(1000);
    		//servo control
    		TPM0->CONTROLS[1].CnV = 1620 + (result)*(5940/1800);
		}
    	if(mode == 5){
    		if(hexKeys[keypad_getkey()] == '#'){
				UART0_init();
			}

    		result = atoi(buffer3);        /* read conversion result and clear COCO flag */

			if(prevresult != result){
				UART0_Transmit_Poll('S');
				UART0_Transmit_Poll('p');
				UART0_Transmit_Poll('e');
				UART0_Transmit_Poll('e');
				UART0_Transmit_Poll('d');
				UART0_Transmit_Poll(':');
				UART0_Transmit_Poll(' ');

				UART0_puts(buffer3);
			}

			prevresult = result;
			delayMs(1000);

			if(distance < (float)12.0)
			 TPM0->CONTROLS[3].CnV = 0;
			else
			TPM0->CONTROLS[3].CnV = result*14000;  /* Set up channel value between 0% - 93%*/
    	}
    }
    return 0;
}

//keypad interrupt
void PORTD_IRQHandler(void){
	key = keypad_getkey();

	if(key != 0){
		keypressed = hexKeys[key];

		if(keypressed == '#'){
			LCD_command(1);
			LCD_command(0x84);
			LCD_data('M');
			LCD_data('o');
			LCD_data('d');
			LCD_data('e');
			LCD_data(':');
			LCD_data(' ');
			UART0_Transmit_Poll('M');
			UART0_Transmit_Poll('o');
			UART0_Transmit_Poll('d');
			UART0_Transmit_Poll('e');
			UART0_Transmit_Poll(':');
			UART0_Transmit_Poll(' ');
			modeSelect = true;
		}

		if(keypressed != '#' && modeSelect){
			if(keypressed == '1'){
				mode = 1;
				LCD_data(keypressed);
				UART0_Transmit_Poll(keypressed);
				LCD_command(0xC3);
				LCD_data('A');
				LCD_data('/');
				LCD_data('D');
				LCD_data(' ');
				LCD_data('S');
				LCD_data('E');
				LCD_data('R');
				LCD_data('V');
				LCD_data('O');

				ADC0_init();                    /* Configure ADC0 */
				ADC0->SC1[0] = 0x40;
				PORTD->ISFR |= 0x00000020;
			}
			if(keypressed == '2'){
				mode = 2;
				LCD_data(keypressed);
				UART0_Transmit_Poll(keypressed);
				LCD_command(0xC2);
				LCD_data('M');
				LCD_data('O');
				LCD_data('T');
				LCD_data('O');
				LCD_data('R');
				LCD_data(' ');
				LCD_data('S');
				LCD_data('E');
				LCD_data('N');
				LCD_data('S');
				LCD_data('O');
				LCD_data('R');

				LED_init();
			    ADC0_init();                    /* Configure ADC0 */
			    PWM_init25();                     /* Configure PWM on TPM0_CH3 */
			    Trigger_Timer2_init();          /* Configure PWM on TPM2_CH0 */
			    Capture_Timer2_init();
			    ADC0->SC1[0] = 0x40;			//start conversion on channel 0
				PORTD->ISFR |= 0x00000020;
			}
			if(keypressed == '3'){
				mode = 3;
				LCD_data(keypressed);
				UART0_Transmit_Poll(keypressed);
				LCD_command(0xC2);
				LCD_data('S');
				LCD_data('E');
				LCD_data('R');
				LCD_data('V');
				LCD_data('O');
				LCD_data(' ');
				LCD_data('P');
				LCD_data('U');
				LCD_data('T');
				LCD_data('T');
				LCD_data('Y');
				LCD_data('3');

				UART0Mode34_init();
				PORTD->ISFR |= 0x00000020;

			}
			if(keypressed == '4'){
				mode = 4;
				LCD_data(keypressed);
				UART0_Transmit_Poll(keypressed);
				LCD_command(0xC2);
				LCD_data('S');
				LCD_data('E');
				LCD_data('R');
				LCD_data('V');
				LCD_data('O');
				LCD_data(' ');
				LCD_data('P');
				LCD_data('U');
				LCD_data('T');
				LCD_data('T');
				LCD_data('Y');
				LCD_data('4');

				UART0Mode34_init();
				PORTD->ISFR |= 0x00000020;
			}
			if(keypressed == '5'){
				mode = 5;
				LCD_data(keypressed);
				UART0_Transmit_Poll(keypressed);
				LCD_command(0xC1);
				LCD_data('M');
				LCD_data('O');
				LCD_data('T');
				LCD_data('O');
				LCD_data('R');
				LCD_data(' ');
				LCD_data('W');
				LCD_data('/');
				LCD_data(' ');
				LCD_data('P');
				LCD_data('U');
				LCD_data('T');
				LCD_data('T');
				LCD_data('Y');

				LED_init();                     /* Configure LEDs */
				UART0Mode34_init();                   /* initialize UART0 for output */
				PWM_init25();
				Trigger_Timer2_init();
				Capture_Timer2_init();
				PORTD->ISFR |= 0x00000020;
			}else{}

			modeSelect = false;

		}

		delayMs(300);
	}
}


void UART0_IRQHandler(void) {
    char c;
    c = UART0->D;           /* read the char received */
    if(mode == 4){
    	buffer4[i%4] = c;
    	i++;
    }else if(mode == 3 || mode == 5){
    	buffer3[0] = c;
    }

}

void ADC0_IRQHandler(void)
{
	if(mode == 1){
		if (alt){
			result1 = ADC0->R[0];        /* read conversion result and clear COCO flag */
		}else if (!alt ){
			result2 = ADC0->R[0];
		}

		if (result1 < 1500 || result1 > 2485){
			TPM0->CONTROLS[1].CnV = 0;			//potentiometer loses control of servo
		}else{
			TPM0->CONTROLS[1].CnV = 1500 +result2*3/2;		//potentiometer controls servo
		}

		if(alt){
			ADC0->SC1[0] = 0x40;        /* start conversion on channel 0 */

		}else{
			ADC0->SC1[0] = 0x44;		//start conversion on channel 4
		}

		alt = !alt;
	}
	if(mode == 2){
		result1 = ADC0->R[0];        		/* read conversion result and clear COCO flag */
		if(distance < (float)12.0){
			TPM0->CONTROLS[3].CnV = 0;
		}else{
			TPM0->CONTROLS[3].CnV = result1*14;  /* For DC Motor: Set up channel value between 0% - 93%*/

		}
		ADC0->SC1[0] = 0x40;
	}else{
		result1 = ADC0->R[0];
	}

}

void TPM0_IRQHandler()
{
	TPM0->CONTROLS[3].CnSC |= 0x80;				//Clear CHF
	TPM0->CONTROLS[3].CnV = 0;
	ADC0->SC1[0] &= ~0x1F;			//start conversion on channel 0
	TPM0->SC |= 0x80;			//clear TOF
}

void TPM2_IRQHandler(void) {
	while(!(TPM2->CONTROLS[1].CnSC & 0x80)) { } /* wait until the CHF is set */
	cont[i%2] = TPM2->CONTROLS[1].CnV;
	if(i%2 == 1){

		if(cont[1] > cont[0] ){
			pulse_width = cont[1] - cont[0];
		}
		    else {
		    pulse_width = cont[1] - cont[0] + mod + 1;
		}

		if(mode == 5){
			distance = (float)(pulse_width*2/3)*0.0343;

			tmpInt1 = distance;
			tmpFrac = distance - tmpInt1;
			tmpInt2 = (tmpFrac * 10000);


			distance = (float)(distance/2.54);

			tmpInt1 = distance;
			tmpFrac = distance - tmpInt1;
			tmpInt2 = (tmpFrac * 10000);
		}

		if(mode == 2){
			sprintf(buffer, "Pulse width %d \r\n", pulse_width); /* convert to string */
			UART0_puts(buffer);
			distance = (float)(pulse_width*2/3)*0.0343;

			tmpInt1 = distance;
			tmpFrac = distance - tmpInt1;
			tmpInt2 = (tmpFrac * 10000);

			sprintf(buffer, "Distance %d.%04d cm\r\n", tmpInt1, tmpInt2); /* convert to string */
			UART0_puts(buffer);

			distance = (float)(distance/2.54);

			tmpInt1 = distance;
			tmpFrac = distance - tmpInt1;
			tmpInt2 = (tmpFrac * 10000);

			sprintf(buffer, "Distance %d.%04d inches\r\n", tmpInt1, tmpInt2); /* convert to string */
			UART0_puts(buffer);
			ADC0->SC1[0] = 0x40;
		}

    	if(distance < (float)12.0)
    	    PTB->PCOR |= 0x40000;       /* Clear PTB18 to turn on red LED */
    		//insert code for DC Motor stops until object out of range
    	else
    	    PTB->PSOR |= 0x40000;       /* Set PTB18 to turn off red LED */

    	if(distance < (float)24.0 && distance > (float)12.0)
    		{
    		PTD->PCOR |= 0x2;		/* Clear PTB18 to turn on blue LED */
    		}
    	else
    		PTD->PSOR |= 0x2;       /* Set PTB18 to turn off blue LED */

	}
	i++;
/*---------------------------------------------------------------------*/
    TPM2->CONTROLS[1].CnSC |= 0x80;    /* clear CHF */
    TPM2->SC |= 0x80;
}

void PWM_init(void)
{
    SIM->SCGC5 |= 0x800;       		   	   /* enable clock to Port C*/
    PORTC->PCR[2] = 0x0400;     		   /* PTC2 used by TPM0 */
    SIM->SCGC6 |= 0x01000000;   		   /* enable clock to TPM0 */
    SIM->SOPT2 |= 0x01000000;   		   /* use MCGFLLCLK as timer counter clock */

    TPM0->SC = 0;               		  /* disable timer */
    TPM0->CONTROLS[1].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
    TPM0->MOD = 60000;          		  /* Set up modulo register for 50 Hz - 48.00 MHz */
    TPM0->CONTROLS[1].CnV = 1500;  		  /* Set up channel value for 2.5% duty-cycle */
    TPM0->SC |= 0x0C;            		  /* enable TPM0 with pre-scaler /16 */
}

void keypad_init(void)
{
    SIM->SCGC5   |= 0x01000;       /* enable clock to Port D */
    PORTD->PCR[0] = 0x103;      /* make PTD0 pin as GPIO and enable pullup*/
    PORTD->PCR[1] = 0x103;      /* make PTD1 pin as GPIO and enable pullup*/
    PORTD->PCR[2] = 0x103;		/* make PTD2 pin as GPIO and enable pullup*/
    PORTD->PCR[3] = 0x103;      /* make PTD3 pin as GPIO and enable pullup*/
    PORTD->PCR[4] = 0x103;      /* make PTD4 pin as GPIO and enable pullup and enable interrupt*/
    PORTD->PCR[5] = 0xA0103;      /* make PTD5 pin as GPIO and enable pullup and enable interrupt*/
    PORTD->PCR[6] = 0x103;      /* make PTD6 pin as GPIO and enable pullup and enable interrupt*/
    PORTD->PCR[7] = 0x103;      /* make PTD7 pin as GPIO and enable pullup and enable interrupt*/
    PTD->PDDR     = 0x0F;       /* make PTD7-4 as input pins, PTD3-0 as outputs */
    NVIC_EnableIRQ(31); 		//enable portD interrupt
}

void keypad_disable(void)
{
    SIM->SCGC5   |= 0x01000;       /* enable clock to Port D */
    PORTD->PCR[4] = 0x103;      /* make PTD4 pin as GPIO and enable pullup and enable interrupt*/
    PORTD->PCR[5] = 0x103;      /* make PTD5 pin as GPIO and enable pullup and enable interrupt*/
    PORTD->PCR[6] = 0x103;      /* make PTD6 pin as GPIO and enable pullup and enable interrupt*/
    PORTD->PCR[7] = 0x103;      /* make PTD7 pin as GPIO and enable pullup and enable interrupt*/
    PTD->PDDR     = 0x0F;       /* make PTD7-4 as input pins, PTD3-0 as outputs */
}

char keypad_getkey(void)
{
    int row, col;
    const char row_select[] = {0x01, 0x02, 0x04, 0x08}; /* one row is active */

    /* check to see any key pressed */
    PTD->PDDR |= 0x0F;          /* enable all rows */
    PTD->PCOR = 0x0F;
    delayUs(2);                 /* wait for signal return */
    col = PTD->PDIR & 0xF0;     /* read all columns */
    PTD->PDDR =0;              /* disable all rows */
    if (col == 0xF0)
        return 0;               /* no key pressed */

    /* If a key is pressed, it gets here to find out which key.
     * It activates one row at a time and read the input to see
     * which column is active. */
    for (row = 0; row < 4; row++)
    {
        PTD->PDDR = 0;                  /* disable all rows */
        PTD->PDDR |= row_select[row];   /* enable one row */
        PTD->PCOR = row_select[row];    /* drive the active row low */
        delayUs(2);                     /* wait for signal to settle */
        col = PTD->PDIR & 0xF0;         /* read all columns */
        if (col != 0xF0) break;         /* if one of the input is low, some key is pressed. */
    }
    PTD->PDDR = 0;                      /* disable all rows */
    if (row == 4)
        return 0;                       /* if we get here, no key is pressed */

    /* gets here when one of the rows has key pressed, check which column it is */
    if (col == 0xE0) return row * 4 + 1;    /* key in column 0 */
    if (col == 0xD0) return row * 4 + 2;    /* key in column 1 */
    if (col == 0xB0) return row * 4 + 3;    /* key in column 2 */
    if (col == 0x70) return row * 4 + 4;    /* key in column 3 */


    return 0;   /* just to be safe */
}

void LCD_init(void)
{
    SIM->SCGC5 |= 0x800;       /* enable clock to Port C */
    PORTC->PCR[8] = 0x100;      /* make PTC8 pin as GPIO */
    PORTC->PCR[9] = 0x100;      /* make PTC9 pin as GPIO */
    PORTC->PCR[10] = 0x100;      /* make PTC10 pin as GPIO */
    PORTC->PCR[11] = 0x100;      /* make PTC11 pin as GPIO */
    PORTC->PCR[12] = 0x100;      /* make PTC12 pin as GPIO */
    PORTC->PCR[13] = 0x100;      /* make PTC13 pin as GPIO */

    PTC->PDDR |= 0x3F00;          /* make PTC 13-8 as output pins */

    delayMs(30);                /* initialization sequence */
    LCD_nibble_write(0x30, 0);
    delayMs(10);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x20, 0);  /* use 4-bit data mode */
    delayMs(1);

    LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          /* move cursor right */
    LCD_command(0x01);          /* clear screen, move cursor to home */
    LCD_command(0x0F);          /* turn on display, cursor blinking */

    LCD_command(0x83);
	LCD_data('H');
	LCD_data('o');
	LCD_data('l');
	LCD_data('d');
	LCD_data(' ');
	LCD_data('#');
	LCD_data(' ');
	LCD_data('T');
	LCD_data('o');
	LCD_command(0xC2);
	LCD_data('S');
	LCD_data('e');
	LCD_data('l');
	LCD_data('e');
	LCD_data('c');
	LCD_data('t');
	LCD_data(' ');
	LCD_data('M');
	LCD_data('o');
	LCD_data('d');
	LCD_data('e');
}

void LCD_nibble_write(unsigned char data, unsigned char control)
{
    data &= 0xF0;       /* clear lower nibble for control */
    control &= 0x0F;    /* clear upper nibble for data */
    PTC->PDOR = ((data | control| EN) & 0xF0) << 6 | ((data|control|EN) & 0x04) << 7 | ((data|control|EN) & 0x01) << 8;       /* RS = 0, R/W = 0 */
    PTC->PDOR = data | control | EN;  /* pulse E */
    delayMs(0);
    PTC->PDOR = (data & 0xF0) << 6;
    PTC->PDOR = 0;
}

void LCD_command(unsigned char command)
{
    LCD_nibble_write(command & 0xF0, 0);   /* upper nibble first */
    LCD_nibble_write(command << 4, 0);     /* then lower nibble */

    if (command < 4)
        delayMs(4);         /* commands 1 and 2 need up to 1.64ms */
    else
        delayMs(1);         /* all others 40 us */
}

void LCD_data(unsigned char data)
{
    LCD_nibble_write(data & 0xF0, RS);    /* upper nibble first */
    LCD_nibble_write(data << 4, RS);      /* then lower nibble  */

    delayMs(1);
}

//clock speed 48 MHZ and 115200 baud
void UART0_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART0(1);            //0x0400;    /* enable clock for UART0 */
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);         //0x04000000 /* use FLL output for UART Baud rate generator */
    UART0->C2   = 0;                             //0x00       /* turn off UART0 while changing configurations */
    UART0->BDH  = UART0_BDH_SBR(0);              //0x00;      /* SBR12:SBR8 = 0b00000    - 115200 Baud
    UART0->C4   = UART0_C4_OSR(15);              //0x0F;      /* Over Sampling Ratio (15+1) */
//	UART0->BDL  = UART0_BDL_SBR(137);            //0x89;      /* SBR7:SBR0  = 0b10001001 -   9600 Baud - Clock = 20.97 MHz*/
//	    UART0->BDL  = UART0_BDL_SBR(12);             //0x0C;      /* SBR7:SBR0  = 0b00001100 - 115200 Baud - Clock = 20.97 MHz*/
    UART0->BDL  = UART0_BDL_SBR(26); 			 //0x1A;      /* SBR7:SBR0  = 0b00011010 - 115200 Baud - Clock = 48.00 MHz */
    UART0->C1   = UART0_C1_M(0);                 //0x00;      /* 8-bit data */
    UART0->C2   = UART0_C2_TE(1)|UART0_C2_RE(1); //|=0x0C;    /* enable transmit & Receive*/
    NVIC->ISER[0] &= ~0x00001000;				//disable interrupt
    SIM->SCGC5     |= SIM_SCGC5_PORTA(1);         //|= 0x0200; /* enable clock for PORTA */
    PORTA->PCR[2]  = PORT_PCR_MUX(2);            //0x0200;    /* make PTA2 UART0_Tx pin */
    PORTA->PCR[1]  = PORT_PCR_MUX(2);            //0x0200;    /* make PTA1 UART0_Rx pin */

    SIM->SCGC6 |= 0x01000000;   		   /* enable clock to TPM0 */
	SIM->SOPT2 |= 0x01000000;   		   /* use MCGFLLCLK as timer counter clock */
	TPM0->SC = 0;
	TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK;
    TPM0->CONTROLS[3].CnV = 0;
    NVIC_DisableIRQ(15);
    NVIC_DisableIRQ(17);
}

void ADC0_init(void)
{
	uint16_t calibration;

    SIM->SCGC5 |= 0x2000;       /* clock to PORTE */
    PORTE->PCR[20] = 0;         /* PTE20 analog input */
    PORTE->PCR[21] = 0;			//PTE21 analog input
    SIM->SCGC6 |= 0x8000000;    /* clock to ADC0 */
    ADC0->SC2 &= ~0x40;         /* software trigger */
    ADC0->SC1[0] |= 0x40;

    /* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;

    //Start Calibration
    ADC0->SC3 |= ADC_SC3_CAL_MASK;
	while (ADC0->SC3 & ADC_SC3_CAL_MASK) {
	// Wait for calibration to complete
	}
	// Finish off the calibration
	// Initialize a 16-bit variable in RAM
	calibration = 0x0;
	// Add the plus-side calibration results to the variable
	calibration += ADC0->CLP0;
	calibration += ADC0->CLP1;
	calibration += ADC0->CLP2;
	calibration += ADC0->CLP3;
	calibration += ADC0->CLP4;
	calibration += ADC0->CLPS;
	// Divide by two
	calibration /= 2;
	// Set the MSB of the variable
	calibration |= 0x8000;
	// Store the value in the plus-side gain calibration register
	ADC0->PG = calibration;
	// Repeat the procedure for the minus-side calibration value
	calibration = 0x0000;
	calibration += ADC0->CLM0;
	calibration += ADC0->CLM1;
	calibration += ADC0->CLM2;
	calibration += ADC0->CLM3;
	calibration += ADC0->CLM4;
	calibration += ADC0->CLMS;
	calibration /= 2;
	calibration |= 0x8000;
	ADC0->MG = calibration;
    //Done Calibration

	/* Reconfigure ADC0*/
    /* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
    NVIC_EnableIRQ(15);		//Enable IRQ 15
}

void UART0Mode34_init(void) {
	SIM->SCGC4 |= 0x0400;    /* enable clock for UART0 */
	SIM->SOPT2 |= 0x04000000;   /* use FLL output for UART Baud rate generator */
	UART0->C2 = 0;          /* turn off UART0 while changing configurations */
	UART0->BDH = 0x00;
	UART0->BDL  = UART0_BDL_SBR(26); 		      /* 115200 Baud - Using 48 MHz clock*/
	UART0->C4 = 0x0F;       /* Over Sampling Ratio 16 */
	UART0->C1 = 0x00;       /* 8-bit data */
	UART0->C2 = 0x2C;       /* enable receive, transmit and receive interrupt*/
	NVIC->ISER[0] |= 0x00001000;    /* enable INT12 (bit 12 of ISER[0]) */
	SIM->SCGC5 |= 0x0200;    /* enable clock for PORTA */
	PORTA->PCR[1] = 0x0200; /* make PTA1 UART0_Rx pin */
	PORTA->PCR[2] = 0x0200; /* make PTA1 UART0_Tx pin */
}

void LED_init(void) {
    SIM->SCGC5 |= 0x400;        /* enable clock to Port B */
    SIM->SCGC5 |= 0x1000;       /* enable clock to Port D */
    PORTB->PCR[18] = 0x100;     /* make PTB18 pin as GPIO */
    PTB->PDDR |= 0x40000;       /* make PTB18 as output pin */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PTD->PDDR |= 0x02;          /* make PTD1 as output pin */
}

void PWM_init25(void)
{
    SIM->SCGC5 |= 0x800;       		   	   /* enable clock to Port C*/
    PORTC->PCR[4] = 0x0400;     		   /* PTC2 used by TPM0 */
    SIM->SCGC6 |= 0x01000000;   		   /* enable clock to TPM0 */
    SIM->SOPT2 |= 0x01000000;   		   /* use MCGFLLCLK as timer counter clock */

    TPM0->SC = 0;               		  /* disable timer */
    TPM0->CONTROLS[3].CnSC |= TPM_CnSC_MSB_MASK |TPM_CnSC_ELSB_MASK; //Enable TPM0_CH1 as edge-aligned PWM
    TPM0->MOD = 60000;          		  /* Set up modulo register for 50 Hz - 48.00 MHz */
    TPM0->CONTROLS[3].CnV = 1500;  		  /* Set up channel value for 2.5% duty-cycle */
    TPM0->SC |= 0x0C;            		  /* enable TPM0 with pre-scaler /16 */

    if(mode == 2)
    	NVIC_EnableIRQ(17);					  //Enable IRQ 17
}

void Trigger_Timer2_init(void)
{
    SIM->SCGC5 |= 0x02000;        			//enable clock to Port E
    SIM->SCGC6 |= 0x04000000;    			//enable clock to TPM2
    SIM->SOPT2 |= 0x01000000;    			//use MCGFLLCLK as timer counter clock
    //PORTC->PCR[2] = 0x0400;    		 	/* PTC2 used by TPM0_CH1
    PORTE->PCR[22] = 0x0300;					//PTE22 used by TPM2_CH0

    TPM2->SC = 0;                			//disable timer
    TPM2->CONTROLS[0].CnSC  = 0x80;   	    //clear CHF  for Channel 2
    TPM2->CONTROLS[0].CnSC |= 0x20|0x08;    //edge-aligned, pulse high MSB:MSA=10, ELSB:ELSA=10
    TPM2->CONTROLS[0].CnV   = 8;  		    //Set up channel value for >10 us
	TPM2->SC |= 0x06;           		    //set timer with prescaler /64
    TPM2->MOD = mod;            		    //Set up modulo register = (44),14999
//*************************PRE-Scaler settings **************************
	TPM2->SC |= 0x08;           	        //enable timer
//***********************************************************************
}

void Capture_Timer2_init(void)  // Also enables the TPM2_CH1 interrupt
{
    SIM->SCGC5 |= 0x2000;       /* enable clock to Port E*/
    SIM->SCGC6 |= 0x04000000;   /* enable clock to TPM2 */
    PORTE->PCR[23] = 0x0300;    /* PTE23 used by TPM2_CH1 */
    SIM->SOPT2 |= 0x01000000;   /* use MCGFLLCLK as timer counter clock */
    TPM2->SC = 0;               /* disable timer */

    TPM2->CONTROLS[1].CnSC = 0x80;   	/* clear CHF  for Channel 1*/
//  MSB:MSA=00, ELSB:ELSA=11 and set interrupt*/
/*  capture on both edges, MSB:MSA=00, ELSB:ELSA=11 and set interrupt*/
    TPM2->CONTROLS[1].CnSC |= TPM_CnSC_CHIE_MASK|TPM_CnSC_ELSB_MASK|TPM_CnSC_ELSA_MASK;
    TPM2->MOD = mod;            		  /* Set up modulo register = 44999*/
    TPM2->CONTROLS[1].CnV = (mod+1)/2 -1; /* Set up 50% dutycycle */

	TPM2->SC |= 0x80;           /* clear TOF */
	TPM2->SC |= 0x06;           /* enable timer with prescaler /2^6 = 64 */
//*************************PRE-Scaler settings *********************************************
	TPM2->SC |= 0x08;           /* enable timer             */
//******************************************************************************************
    NVIC_EnableIRQ(TPM2_IRQn);  /* enable Timer2 interrupt in NVIC */
}

void UART0_Transmit_Poll(uint8_t data) {
	while (!(UART0->S1 & UART_S1_TDRE_MASK)); // wait until transmit data register is empty TDRE_Mask is 0x80
	UART0->D = data;
}
uint8_t UART0_Receive_Poll(void) {
	while (!(UART0->S1 & UART_S1_RDRF_MASK)); // wait until receive data register is full RDRF_Mask is 0x20
	return UART0->D;
}

void UART0Tx(char c) {
    while(!(UART0->S1 & 0x80)) {
    }   /* wait for transmit buffer empty */
    UART0->D = c; /* send a char */
}

void UART0_puts(char* s) {
    while (*s != 0)         /* if not end of string */
        UART0Tx(*s++);      /* send the character through UART0 */
}

void delayMs(int n) {
	    int i, j;
	    for(i = 0 ; i < n; i++)
	        for (j = 0; j < 3500; j++) {}
}

void delayUs(int n){
    int i; int j;
    for(i = 0 ; i < n; i++) {
        for(j = 0; j < 5; j++) ;
    }
}
