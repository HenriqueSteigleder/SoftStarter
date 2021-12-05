/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/*  PINO D7 DO LCD -> Pino PB2
 *  PINO D6 DO LCD -> Pino PB1
 *  PINO D5 DO LCD -> Pino PB15
 *  PINO D4 DO LCD -> Pino PB14
 *
 *  PINO LCD_EN ->  Pino PB13
 *  PINO LCD_RS ->  Pino PB5
 *
 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


void tempo_us(uint16_t t)
{
	for(uint16_t i=0; i<=t*11; i++)
	{
		__NOP();
		__NOP();
	}
}

//**** PULSO DE ENABLE  *****

void enable() {
	HAL_GPIO_WritePin(Habilita_GPIO_Port,Habilita_Pin,1);
	tempo_us(4);
	HAL_GPIO_WritePin(Habilita_GPIO_Port,Habilita_Pin,0);
}

//****  ESCRITA NO LCD EM 04 BITS  ******

void lcd_write(unsigned char c) {
	unsigned char d = (c >> 4) & 0x0F;
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, (d & 0x01));
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, (d & 0x02));
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, (d & 0x04));
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, (d & 0x08));
	enable();

	d = (c & 0x0F);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, (d & 0x01));
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, (d & 0x02));
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, (d & 0x04));
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, (d & 0x08));
	enable();

	tempo_us(40);
}

//**** ESCRITA DE UMA STRING (NOME)  ****

void lcd_puts(const char *s) {
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,1);
	while (*s) {
		lcd_write(*s++);
	}
}

//**** ESCRITA DE UMA VARIÁVEL NUMÉRICA DECIMAL (Número)  ****

void lcd_printd(uint16_t c){
	char buf[10];
	sprintf(buf,"%d",c);
	lcd_puts(buf);
}

/*
void lcd_printf(float c){
	char buf[10];
	sprintf(buf,"%f",c);
	lcd_puts(buf);
}
*/

//*** ESCRITA DE UM CARACTERE  ****

void lcd_putc(char c) {
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,1);
	lcd_write(c);
}

//**** POSICIONAMENTO DO CURSOR  *****

void lcd_goto(unsigned char pos) {
	HAL_GPIO_WritePin(Habilita_GPIO_Port,RS_Pin,0);
	lcd_write(pos);
}

//*** LIMPEZA DA MEMÓRIA DDRAM (Tela) *****

void lcd_clear() {
	HAL_GPIO_WritePin(Habilita_GPIO_Port,RS_Pin,0);
	lcd_write(0x01);
	HAL_Delay(2);
}
void lcd_init() {

	HAL_Delay(15);
	HAL_GPIO_WritePin(D0_GPIO_Port,
			D0_Pin | D1_Pin | D2_Pin | D3_Pin | RS_Pin | Habilita_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin | D1_Pin, GPIO_PIN_SET);

	enable();
	HAL_Delay(5);

	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin | D1_Pin, GPIO_PIN_SET);
	enable();
	tempo_us(100);

	HAL_GPIO_WritePin(D0_GPIO_Port,
			D0_Pin | D1_Pin | D2_Pin | D3_Pin | RS_Pin | Habilita_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
	enable();
	tempo_us(40);

	lcd_write(0x28);
	lcd_write(0x06);
	lcd_write(0x0C);
	lcd_clear();
}

