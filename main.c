/*
 * main.c
 *
 *  Created on: 19.10.2016
 *      Author: Myszkowski
 */

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "HD44780/HD44780.h"
#include "DS18B20/ds18b20.h"
#include "RS232/uart.h"

#define UART_BAUD_RATE      9600	//prêdkoœc komunikacji przez RS232

#define ENK_A			(1 << PB4)	//pin kana³u A enkodera
#define ENK_B			(1 << PB5)	//pin kana³u B enkodera
#define DOL				(1 << PC3)	//pin przycisku w dó³
#define GORA			(1 << PC4)	//pin przycisku w górê (przycisk enkodera)
#define V_SET			(1 << PB1)	//pin steruj¹cy napiêciem wyjœciowym
#define I_SET			(1 << PB2)	//pin steruj¹cy ograniczeniem pr¹dowym wyjœciowym
#define V_OUT			(1 << PC0)	//pin mierz¹cy napiêcie wyjœciow
#define I_OUT			(1 << PC1)	//pin mierz¹cy pr¹d wyjœciowy
#define UZWOJENIE		(1 << PC5)	//pin steruj¹cy uzwojeniem
#define WENT			(1 << PB3)	//pin steruj¹cy wentylatorem
#define WYJ				(1 << PC2)	//pin w³¹czaj¹cy (1) lub wy³¹czaj¹cy stabilizator (0)

#define L_WIERSZY		4			//liczba wierszy w wyœwietlaczu HD44780
#define V_MAX			25			//wartoœc maksymalnego napiêcia wyjœciowego
#define I_MAX			3			//wartoœc maksymalnej wartoœci ograniczenia pr¹dowego
#define TEMP_MAX		100			//maksymalna ustawiana temperatura przy której wiatrak zmienia stan
#define TEMP_MIN		40			//minimalna ustawiana temperatura przy której wiatrak zmienia stan
#define L_POMIAROW_ADC	1000			//liczba pomiarów wykonywanych przez ADC, nastêpnie uœrednionych
#define V_UZW			12			//napiêcie wyjœciowy, przy którym prze³¹czane jest uzwojenie

int8_t menu_pos=0;					//zmienna okreœlaj¹ca edytowan¹ funkcjê
uint8_t menu_pos_max=3;				//liczba funkcji + 1

uint8_t stan_enk=0;					//stan kana³u A i B enkodera
uint8_t stan_enk_stare=0;			//stary stan kana³u A i B enkodera

//wartosci
double Vset=0;						//wartoœc napiêcia nastawiana
double Iset=3.0;					//wartoœc ograniczenia pr¹dowego nastawiana
double Vout=0;						//wartoœc napiêcia mierzona
double Iout=0;						//wartoœc ograniczenia pr¹dowego mierzona
uint64_t suma_Vout=0;				//suma zebranych pomiarów napiêcia
uint64_t suma_Iout=0;				//suma zebranych pomiarów pr¹du

uint8_t l_uzwojen=2;				//liczba za³¹czonych uzwojeñ
uint8_t wentylator=0;				//stan wentylatora

uint16_t licznik_pomiarow=0;			//ile pomiarów zosta³o zsumowanych
uint8_t licznik_wyswietlacza=0;

uint8_t temp=0;						//temperatura badana
uint8_t temp_set=50;				//temperatura przy jakiej za³¹cza siê wentylator (histereza= +-5stopni celsjusza)
uint8_t wyjscie=1;					//stan stabilizatora (0-wy³¹czony, 1-w³¹czony)

uint16_t OCR1Aset=0;				//Vset*10;
uint16_t OCR1Bset=167;				//237-Iset*70/3;

unsigned int RS232_znak;			//pojedynczy odbierany znak przez rs232
char write_bufor[100];				//bufor zapisu RS232
char read_bufor[100];				//bufor odczytu RS232
uint8_t data_read=0; 				//rodzaj wysy³anych danych przez RS232 0-nic,1-napiêcie,2-pr¹d
uint8_t char_count=0;

char bufor[100];					//bufor konwersji tablicy charów

/*Konfiguracja enkodera*/
void Encoder_Inicjalizacja()
{
	/* ustawia PB4(ENK.A), PB5(ENK.B), PC3(MENU_BUTT), PC4(ENK.BUTT) jako wejœcie */
		DDRB &=~ (ENK_A);
		DDRB &=~ (ENK_B);
		PORTB |= (ENK_A | ENK_B);   /* podci¹gniecie pinów PB4 and PB5 do 5V   */

		DDRC &=~ (GORA);				/* PC3 and PC4 as input */
		DDRC &=~ (DOL);
		PORTC |= (GORA | DOL);   /* podci¹gniecie pinów PC3 and PC4 do 5V  */

		PCICR |= (1<<PCIE0);
		PCMSK0|= ((1<<PCINT5) | (1<<PCINT4));

		PCICR |= (1<<PCIE1);
		PCMSK1|=((1<<PCINT11)|(1<<PCINT12));
}

/*Konfiguracja regulacji pr¹du i napiêcie - PWM*/
void DAC_Inicjalizacja()
{
	DDRB |= V_SET | I_SET;
	TCCR1A |= (1<<WGM10);                      // Fast PWM 8bit
	TCCR1B |= (1<<WGM12);
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1) ;        //Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM
	TCCR1B |= (1<<CS10);             // Preksaler = 1  fpwm = 64khz
	//ICR1 = 250;			//max wartosc w rejestrze
	TIMSK1 |= (1<<TOIE0);	//Overflow Interrupt Enable
}

/*Konfiguracja odczytu napiêcia i pr¹du - przetwornik ADC*/
void ADC_Inicjalizacja()
{
	DDRC&=~(V_OUT|I_OUT);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC

	ADCSRA |= (1 << ADEN);  // w³¹czenie ADC
	ADCSRA |= (1 << ADSC);  // uruchomienie konwercji analogowo-cyfrowej
}

/*Konfiguracja prze³¹czania zwojów*/
void Uzwojenie_Inicjalizacja()
{
	DDRC |= UZWOJENIE;
	PORTC &= ~UZWOJENIE;
}

/*Konfiguracja sterowania wentylatorem*/
void Wentylator_Inicjalizacja()
{
	DDRB |= WENT;
	PORTB &= ~WENT;
}

/*Konfiguracja sterowania w³¹czaniem/wy³¹czaniem stabilizatora*/
void Wyjscie_Inicjalizacja()
{
	DDRC |= WYJ;
	PORTC |= WYJ;
}

/*Konfiguracja pomiaru temperatury*/
void Temperatura_Inicjalizacja()
{
		ds18b20_reset(); //reset
		ds18b20_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
		ds18b20_writebyte(DS18B20_CMD_CONVERTTEMP); //start temperature conversion
}

/*Funkcja wyœwietlaj¹ca HD44780*/
void Menu_Wyswietl(uint8_t pos)
{
	LCD_Clear();
	LCD_Home();
	LCD_WriteText(">");
	for(int i=0;i<L_WIERSZY;i++)
	{
		LCD_GoTo(1+(i/2)*20, i);
		switch ((i+pos)%(menu_pos_max+1))
		{
		case 0:
			LCD_WriteText("Vout=");
			dtostrf(Vout, 5, 2, bufor);
			LCD_WriteText(bufor);
			LCD_WriteText("V(");
			dtostrf(Vset, 5, 2, bufor);
			LCD_WriteText(bufor);
			LCD_WriteText("V)");
			break;
		case 1:
			LCD_WriteText("Iout=");
			dtostrf(Iout, 5, 2, bufor);
			LCD_WriteText(bufor);
			LCD_WriteText("A(");
			dtostrf(Iset, 5, 2, bufor);
			LCD_WriteText(bufor);
			LCD_WriteText("A)");
			break;
		case 2:
			LCD_WriteText("Temp=");
			itoa(temp, bufor, 10);
			LCD_WriteText(bufor);
			LCD_WriteData(223);
			LCD_WriteText("C (");
			itoa(temp_set, bufor, 10);
			LCD_WriteText(bufor);
			LCD_WriteData(223);
			LCD_WriteText("C)");
			break;
		default:
			if(wyjscie==1)LCD_WriteText("Wyjsci=ON");
			else LCD_WriteText("Wyjsci=OFF");
			break;
		}
	}
}

/*Odczyt napiêcia i pr¹du wyjœciowego*/
void Odczyt_Napiecia()
{
	while((ADCSRA &(1<<ADSC)))
	{}
		cli();

		if(bit_is_clear(ADMUX, MUX0))
		{
			suma_Vout+=(ADCL | (ADCH << 8));
			ADMUX|=(1<<MUX0);
		}
		else
		{
			suma_Iout+=(ADCL | (ADCH << 8));
			ADMUX &= ~(1<<MUX0);
			licznik_pomiarow++;
		}

		if(licznik_pomiarow==L_POMIAROW_ADC)
		{
			Vout=suma_Vout*5/(0.18*1023.0*L_POMIAROW_ADC)*0.985 ;
			Iout=(suma_Iout*5.0/(1023.0*L_POMIAROW_ADC)-2.5)/0.125;
			suma_Iout=0;
			suma_Vout=0;
			licznik_pomiarow=0;
		}
		ADCSRA|=(1<<ADSC);
		sei();

}

/*Odczyt temperatury*/
void Odczyt_Temperatury()
{
	if(ds18b20_readbit())
	{
		uint8_t temperature_l;
		uint8_t temperature_h;

		ds18b20_reset(); //reset
		ds18b20_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
		ds18b20_writebyte(DS18B20_CMD_RSCRATCHPAD); //read scratchpad

		//read 2 byte from scratchpad
		temperature_l = ds18b20_readbyte();
		temperature_h = ds18b20_readbyte();
		temp = ( ( temperature_h << 8 ) + temperature_l ) * 0.0625;

		ds18b20_reset(); //reset
		ds18b20_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
		ds18b20_writebyte(DS18B20_CMD_CONVERTTEMP); //start temperature conversion
	}
}

/*Prze³¹czanie zwojów*/
void Przelacz_Uzwojenie()
{
	if(l_uzwojen==2 && Vout<=V_UZW-1 && Vset<=V_UZW-1)
	{
		l_uzwojen=1;
		PORTC |= UZWOJENIE;
	}
	else if(l_uzwojen==1 && Vout>=V_UZW+1 && Vset>=V_UZW+1)
	{
		l_uzwojen=2;
		PORTC &= ~UZWOJENIE;
	}
}

/*W³¹czanie i wy³¹czanie wentylatora*/
void Przelacz_Wentylator()
{
	if(wentylator==0 && temp>=temp_set+5)
	{
		wentylator=1;
		PORTB |= WENT;
	}
	else if(wentylator==1 && temp<=temp_set-5)
	{
		wentylator=0;
		PORTB &= ~WENT;
	}
}

/*Komunikacja przez RS232*/
void RS232()
{
			RS232_znak = uart_getc();

	        if ((RS232_znak & UART_NO_DATA )) {}
	        else
	        {
	            if(data_read>0 && ((RS232_znak>=48 && RS232_znak<=57)|| RS232_znak==46) )
				{
					read_bufor[char_count]=RS232_znak;
					char_count++;
				}
	            else if(data_read>0)
	            {
	            	if(RS232_znak=='\n')
					{
	            		double t;
	            		if(data_read==1)
						{
	            			t=atof(read_bufor);
	            			if(t>25.0)Vset=25.0;
	            			else if (t<0.0)Vset=0.0;
	            			else Vset=t;
	            			OCR1Aset=Vset*10.23;//*10;
						}
						else if(data_read==2)
						{
							t=atof(read_bufor);
							if(t>3.0)Iset=3.0;
							else if (t<0.0)Iset=0.0;
							else Iset=t;
							OCR1Bset=247-Iset*70/3;
						}
					}
	            	else
	            	{
	            		uart_puts("Bledne dane");
	            	}
	            	memset(read_bufor, 0, sizeof read_bufor);
	            	char_count=0;
	            	data_read=0;
	            }
	            else if(RS232_znak=='?')
	            {
	            	uart_puts("Vout=");
	            	dtostrf(Vout, 5, 2, write_bufor);
	            	uart_puts(write_bufor);
	            	uart_puts(", Vset=");
					dtostrf(Vset, 5, 2, write_bufor);
					uart_puts(write_bufor);
	            	uart_puts(", Iout=");
					dtostrf(Iout, 5, 2, write_bufor);
					uart_puts(write_bufor);
					uart_puts(", Iset=");
					dtostrf(Iset, 5, 2, write_bufor);
					uart_puts(write_bufor);
	            }
	            else if(RS232_znak=='V')
	            {
	            	 data_read=1;
	            }
	            else if(RS232_znak=='I')
	            {
	            	data_read=2;
	            }
	            else
	            	uart_puts("Bledne dane");

	        }
}

//przerwanie enkodera
ISR(PCINT0_vect )
{
	stan_enk=0;
	if(!bit_is_clear(PINB, PB4))
		stan_enk |= (1<<1);

	if(!bit_is_clear(PINB, PB5))
		stan_enk |= (1<<0);

		if((stan_enk==3 && stan_enk_stare==1))
		{
			switch(menu_pos)
			{
			case 0:
				if(Vset<V_MAX-0.001)Vset+=0.1;
				OCR1Aset=Vset*10.23;//*10;
				break;
			case 1:
				if(Iset<I_MAX-0.001)Iset+=0.1;
				OCR1Bset=247-Iset*70/3;
				break;
			case 2:
				if(temp_set<TEMP_MAX)temp_set+=1;
				break;
			default:
				if(wyjscie==1)
				{
					wyjscie=0;
					PORTC &= ~(1<<PC2);
				}
				else
				{
					wyjscie=1;
					PORTC |= (1<<PC2);
				}
				break;
			}
		}

		else if(stan_enk==0 && stan_enk_stare==1)
		{
			switch(menu_pos)
			{
			case 0:
				if(Vset>0.001)Vset-=0.1;
				OCR1Aset=Vset*10.23;//*10;
				break;
			case 1:
				if(Iset>0.001)Iset-=0.1;
				OCR1Bset=247-Iset*70/3;
				break;
			case 2:
				if(temp_set>TEMP_MIN)temp_set-=1;
				break;
			default:
				if(wyjscie==1)wyjscie=0;
				else wyjscie=1;
				break;
			}
		}
	if(menu_pos<0)
		menu_pos=menu_pos_max;
	if(menu_pos>menu_pos_max)
		menu_pos=0;

	stan_enk_stare = stan_enk;

}

//przerwanie przycisków
ISR(PCINT1_vect )
{
	if(bit_is_clear(PINC, PC3))
	{
		if(menu_pos==menu_pos_max)
			menu_pos=0;
		else
			menu_pos+=1;
	}

	if(bit_is_clear(PINC, PC4))
	{
		if(menu_pos==0)
			menu_pos=menu_pos_max;
		else
			menu_pos-=1;
	}
}

//przerwanie Vset, Iset
ISR(TIMER1_OVF_vect)
{
   OCR1A=OCR1Aset;
   OCR1B=OCR1Bset;
}

int main(void)
{
	Uzwojenie_Inicjalizacja();
	LCD_Initalize();
	Encoder_Inicjalizacja();
	DAC_Inicjalizacja();
	ADC_Inicjalizacja();
	Wentylator_Inicjalizacja();
	Temperatura_Inicjalizacja();
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	Wyjscie_Inicjalizacja();
	Menu_Wyswietl(0);

	sei();   // globalne uruchomienie przerwan

	while(1)
	{
		for(int i=0; i<2000;i++)
		{
			Odczyt_Napiecia();			//odczyt napiecia i pr¹du wyjœciowego
			Odczyt_Temperatury();		//odczyt temperatury
			Przelacz_Wentylator();		//sprawdza czy nale¿y prze³¹czyc wentylator
			Przelacz_Uzwojenie();		//sprawdza czy nale¿y prze³¹czyc uzwojenie
			RS232();					//sprawdza czy konieczna jest komunkacja przez RS232
		}
		Menu_Wyswietl(menu_pos);		//wyœwietla aktualne parametry zasilacza
	}
  return 0;
}



