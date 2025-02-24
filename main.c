#include "MKL05Z4.h"
#include "DAC.h"
#include "ADC.h"
#include "tsi.h"
#include "klaw.h"
#include "frdm_bsp.h"
#include "lcd1602.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#define MASKA_10BIT 0x03FF //1023
int DIV_CORE=8192;  // Przerwanie co 0.120ms
volatile uint8_t S2_press = 0; // "1" - klawisz zosta� wci�ni�ty "0" - klawisz "skonsumowany"
volatile uint8_t S3_press = 0; // "1" - klawisz zosta� wci�ni�ty "0" - klawisz "skonsumowany"
volatile uint8_t S4_press = 0; // "1" - klawisz zosta� wci�ni�ty "0" - klawisz "skonsumowany"
volatile uint16_t faza, mod,df; //do syntezy DDS
volatile uint8_t wynik_ok = 0;
volatile uint16_t temp;
volatile float wynik; 				// wynik z ADC
volatile uint16_t slider = 1; // pozycja suwaka dotykowego
volatile uint16_t dac;				// warto�� wyjsciowa dla DAC
volatile int16_t Sinus[1024];
volatile uint16_t ksztalt = 0; // zmienna kszta�t ustawiaj�ca kszta�t przebiegu

volatile uint8_t octave = 4; // domyslna oktawa -  C4
uint32_t sum = 0;            // suma pr�bek
uint16_t sample_count = 0;   // licznik pr�bek

// mapowanie zakres�w ADC na nuty pianina
const uint16_t frequency_map[14] = {0, 315, 630, 945, 1260, 1575, 1890, 2205, 2520, 2835, 3150, 3465, 3795, 4095};
const char *notes[13] = {"C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "H ", "C'"};
const float base_frequencies[13] = {261.6, 277.2, 293.7, 311.1, 329.6, 349.2, 370.0, 392.0, 415.3, 440.0, 466.2, 493.9, 523.3};


uint32_t Trojkat(uint16_t faza) {
    if (faza < 512) {
        return (faza * 8);  // Rosn�ca: 0 do 4095
    } else {
        return (4095 - ((faza - 512) * 8));  // Malej�ca: 4095 do 0
    }
}

uint32_t Pila(uint16_t faza)	//funkcja generuj�ca przebieg pi�okszta�tny
{
    return (faza * 4.0);  // przebieg rosn�cy od 0 do 4095
}

void SysTick_Handler(void) // Podprogram obs�ugi przerwania od SysTick'a
{
    dac = 0;

    switch (ksztalt)
    {
    case 0: // Przebieg sinusoidalny
        dac = (Sinus[faza] / 100) * slider + 0x0800;
        break;
    case 1: // Przebieg tr�jk�tny
        dac = (Trojkat(faza) * slider / 100) + 0x0800;
        break;
    case 2: // Przebieg pi�okszta�tny
        dac = (Pila(faza) * slider / 100) + 0x0800;
        break;
    }

    DAC_Load_Trig(dac);			 // za�adowanie warto�ci do DAC
    faza += mod;             // faza - generator cyfrowej fazy
    faza &= MASKA_10BIT;     // rejestr sterujacy przetwornikiem, liczony modulo 1024 (N=10 bit�w)
}

void ADC0_IRQHandler()
{
    temp = ADC0->R[0];										// Odczyt danej
		if(!wynik_ok)													// sprawd�, czy wynik skonsumowany przez p�tle g��wn�
		{
			wynik = temp;												// wy�lij now� dan� do petli gl�wnej
			wynik_ok=1;
		}
}

void PORTA_IRQHandler(void) // podprogram obs�ugi przerwania od klawiszy S2, S3 i S4
{
    uint32_t buf;
    buf = PORTA->ISFR & (S2_MASK | S3_MASK | S4_MASK); 

    switch (buf)
    {
    case S2_MASK:
        DELAY(100);
        if (!(PTA->PDIR & S2_MASK)) // minimalizacja drga� zestyk�w
        {
            DELAY(100);
            if (!(PTA->PDIR & S2_MASK))
            {
                if (!S2_press)
                {
                    S2_press = 1;
                }
            }
        }
        break;
    case S3_MASK:
        DELAY(100);
        if (!(PTA->PDIR & S3_MASK)) // minimalizacja drga� zestyk�w
        {
            DELAY(100);
            if (!(PTA->PDIR & S3_MASK))
            {
                if (!S3_press)
                {
                    S3_press = 1;
                }
            }
        }
        break;
    case S4_MASK:
        DELAY(100);
        if (!(PTA->PDIR & S4_MASK)) // minimalizacja drga� zestyk�w
        {
            DELAY(100);
            if (!(PTA->PDIR & S4_MASK))
            {
                if (!S4_press)
                {
                    S4_press = 1;
                }
            }
        }
        break;
    default:
        break;
    }
    PORTA->ISFR |= S2_MASK | S3_MASK | S4_MASK; // kasowanie wszystkich bit�w ISF
    NVIC_ClearPendingIRQ(PORTA_IRQn); // usuni�cie przerwania
}

int main(void)
{   
    volatile uint8_t w;
    slider = 50; // ustawienie pocz�tkowej g�o�cno�ci na 50
    uint8_t kal_error;
    char display[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
		df = DIV_CORE / MASKA_10BIT;
		Klaw_Init(); 			// inicjalizacja klawiatury
    Klaw_S2_4_Int();  // w��czenie przerwa� dla klawiszy S2, S3 i S4
    TSI_Init(); 			// inicjalizacjia panelu dotykowego
    LCD1602_Init();           // inicjalizacja wy�wietlacza LCD
    LCD1602_Backlight(TRUE);  // w��czenie pod�wietlenia
    LCD1602_Print("---");     // ekran kontrolny - nie zniknie, je�li dalsza cz�� programu nie dzia�a
    kal_error = ADC_Init();   // inicjalizacja i kalibracja przetwornika A/C
    if (kal_error) { 
        while (1);  					// jesli b��d kalibracji, zatrzymaj program
    }

    DAC_Init(); // inicjalizacja DAC
    for (faza = 0; faza < 1024; faza++) {
			Sinus[faza] = (sin((double)faza * 0.0061359231515) * 4095.0);
}
		mod=261;
    faza = 0; // ustawienie warto�ci pocz�tkowych generatora cyfrowej fazy i modulatora fazy
    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(8); // pierwsze wyzwolenie przetwornika ADC0 w kanale 8 i odblokowanie przerwania
    NVIC_SetPriority(SysTick_IRQn, 0);								  // ustawienie priorytetu przerwania	na najwy�szy - wykorzystywane do generowania sygna��w  
    SysTick_Config(SystemCoreClock / DIV_CORE); 				// start licznika SysTick (generatora DDS)
	
    while (1)
    { 
				w = TSI_ReadSlider(); // odczytaj pozycje panelu dotykowego
				LCD1602_SetCursor(10, 0);    
				sprintf(display, "V:%3.0d", slider);   // warto�� pocz�tkowa glo�no�ci - 50
				LCD1602_Print(display);
				LCD1602_Print("% ");
			
        LCD1602_SetCursor(14,1);
				sprintf(display, "O%d", octave);
				LCD1602_Print(display);
        
				if (w != 0) {        
            slider = w;
            LCD1602_SetCursor(10, 0);
            wynik = (float)slider;        
						sprintf(display, "V:%3.0f", wynik);    // zmieniona warto�� glo�no�ci po dotkni�ciu palcem
            LCD1602_Print(display);
            LCD1602_Print("% ");
        }
        // obs�uga zmiany oktawy
       if (S3_press) {
					if (octave < 8) { // c8
							octave++; // zwi�kszenie oktawy
							DIV_CORE = (octave > 5) ? 50000 : 8192; // dla oktawy wi�kszej od 5 DIV_CORE na 50000, poniewa� dla ni�szego fclk za niska f pr�bkowania
							df = DIV_CORE / MASKA_10BIT;
							SysTick_Config(SystemCoreClock / DIV_CORE); // ponowna konfiguracja SysTick
					}
					S3_press = 0;
			}

			if (S2_press) {
					if (octave > 0) { // c0
							octave--; // zmniejszenie oktawy
							DIV_CORE = (octave > 5) ? 50000 : 8192;
							df = DIV_CORE / MASKA_10BIT;
							SysTick_Config(SystemCoreClock / DIV_CORE); // ponowna konfiguracja SysTick
					}
					S2_press = 0;
			}

				if (S4_press) {
							ksztalt++;               // zwi�ksz warto�� kszta�t, przechodz�c do kolejnego kszta�tu
							if (ksztalt > 2) {       // je�li przekroczy 2 (maksymalny indeks), resetuj do 0
									ksztalt = 0;
							}
							S4_press = 0;
						}
				
        if (wynik_ok) {
            uint8_t frequency_index = 0; 									// indeks odpowiadaj�cy wybranej nucie
						for (uint8_t i = 0; i < 13; i++) {  					// przejd� przez map� cz�stotliwo�ci
								if (wynik < (float)frequency_map[i + 1]) {// znajd� zakres, w kt�rym znajduj� si� wynik
										frequency_index = i; 									// ustaw indeks dla bie��cej nuty
										break; 
                }
            }
            float frequency = base_frequencies[frequency_index] * pow(2, octave - 4); // wyznaczenie cz�stotliwo�ci dla bie��cej oktawy
            mod = (uint16_t)((frequency)/df);  // oblicz warto�� modulatora fazy
            LCD1602_SetCursor(0, 0);
            sprintf(display, "Nuta:%s", notes[frequency_index]);  // wy�wietlenie nuty i cz�stotliwo�ci na LCD
            LCD1602_Print(display);
            LCD1602_SetCursor(0, 1);
            sprintf(display, "Freq=%4.1fHz ",frequency);
            LCD1602_Print(display);
						wynik_ok = 0; // oznacz wynik jako przetworzony
						
        }
    }
}
