#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>		// wird in aktuellen Versionen der avr-lib mit xx.h eingebunden

#define __disable_interrupt cli
#define __enable_interrupt sei

#include "USI_UART_config.h"

#include <util/delay.h>		/* in älteren avr-libc Versionen <avr/delay.h> */

#define CHARGE_OFF PORTA &= ~(1<<PA7);
#define CHARGE_ON PORTA |=(1<<PA7);
#define LED_GUE_ON PORTA |=(1<<PA6);
#define LED_GUE_OFF PORTA &= ~(1<<PA6);

#define F_CPU 8000000L

#ifndef EEMEM
#define EEMEM  __attribute__ ((section (".eeprom")))
#endif

uint8_t ee_chargetime EEMEM = 30;	// Ladezeit in sec der einzelnen Durchgaenge
uint8_t ee_deltau EEMEM = 6;	// minus delta u Abschaltkriterium
uint8_t ee_timeoutminutes EEMEM = 120;	// minus delta u Abschaltkriterium

void
long_delay (uint16_t ms)
{
  for (; ms > 0; ms--)
    _delay_ms (1);
}

uint16_t
ReadChannel (uint8_t mux)
{
  uint8_t i;
  uint16_t result;

  ADCSR = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);	// Frequenzvorteiler 
  // setzen auf 8 (1) und ADC aktivieren (1)

  ADMUX = mux;			// Kanal waehlen
  ADMUX |= (1 << REFS1) | (1 << REFS0);	// interne Referenzspannung nutzen 

  /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
  ADCSR |= (1 << ADSC);		// eine ADC-Wandlung 
  while (ADCSR & (1 << ADSC))
    {
      ;				// auf Abschluss der Konvertierung warten 
    }
  result = ADCW;		// ADCW muss einmal gelesen werden,
  // sonst wird Ergebnis der nächsten Wandlung
  // nicht übernommen.

  /* Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen */
  result = 0;
  for (i = 0; i < 4; i++)
    {
      ADCSR |= (1 << ADSC);	// eine Wandlung "single conversion"
      while (ADCSR & (1 << ADSC))
	{
	  ;			// auf Abschluss der Konvertierung warten
	}
      result += ADCW;		// Wandlungsergebnisse aufaddieren
    }
  ADCSR &= ~(1 << ADEN);	// ADC deaktivieren (2)

  result /= 4;			// Summe durch vier teilen = arithm. Mittelwert

  return result;
}

void
transmit_chars (char *c)
{
  uint8_t i = 0;
  while (c[i] != '\0')
    {
      USI_UART_Transmit_Byte (c[i++]);
    }

}

void
transmit_uint16_usi (int16_t value)
{
  unsigned char i;
  if (value < 0)
    {
      USI_UART_Transmit_Byte ('-');
      value *= -1;
    }

  i = value / 10000;
  USI_UART_Transmit_Byte (i + 48);
  while (value >= 10000)
    value -= 10000;

  i = value / 1000;
  USI_UART_Transmit_Byte (i + 48);
  while (value >= 1000)
    value -= 1000;

  i = value / 100;
  USI_UART_Transmit_Byte (i + 48);
  while (value >= 100)
    value -= 100;

  i = value / 10;
  USI_UART_Transmit_Byte (i + 48);
  while (value >= 10)
    value -= 10;
  i = value;
  USI_UART_Transmit_Byte (i + 48);
  USI_UART_Transmit_Byte ('\n');
  USI_UART_Transmit_Byte ('\r');
  return;
}

void
pio_init (void)
{
  CHARGE_OFF DDRA |= (1 << DDA7) | (1 << DDA6);
}

int
main (void)
{
  uint16_t adcval = 0;
  uint16_t adcval_max = 0;
  int16_t adcval_dif;
  uint8_t chargetime;
  uint8_t deltau;
  uint8_t i;
  uint8_t charged_minutes;
  uint8_t charged_seconds;
  uint8_t timeoutminutes;
  uint8_t charge_error;

  USI_UART_Flush_Buffers ();
  USI_UART_Initialise_Receiver ();	// Initialisation for USI_UART receiver
  __enable_interrupt ();	// Enable global interrupts

  pio_init ();

  MCUCR = (1 << SE) | (0 << SM1) | (0 << SM0);	// Enable Sleepmode: Idle
  while (1)
    {
      chargetime = eeprom_read_byte (&ee_chargetime);
      deltau = eeprom_read_byte (&ee_deltau);
      timeoutminutes = eeprom_read_byte (&ee_timeoutminutes);

      transmit_chars ("ChargeTime: ");
      transmit_uint16_usi (chargetime);
      transmit_chars ("-delta U: ");
      transmit_uint16_usi (deltau);
      transmit_chars ("timeoutminutes: ");
      transmit_uint16_usi (timeoutminutes);

      adcval_dif = 0;
      adcval_max = 0;
      charged_minutes = 0;
      charged_seconds = 0;
      CHARGE_OFF;
      transmit_uint16_usi (adcval);

      while (adcval < 400)
	{
	  adcval = ReadChannel (0);
	  transmit_uint16_usi (adcval);
	  long_delay (500);
	}
      transmit_uint16_usi (adcval);

      long_delay (1000);	// settle down und warte bis akku "richtig" eingelegt

      // Ladebeginn
      while ((adcval_dif < deltau) && (charged_minutes < timeoutminutes))
	{
	  CHARGE_ON;
	  for (i = 0; i < chargetime; i++)
	    long_delay (1000);

	  charged_seconds += chargetime;
	  CHARGE_OFF;
	  long_delay (500);
	  adcval = ReadChannel (0);
	  if (adcval < 100)
	    {
	      charge_error = 1;
	      break;
	    }
	  if (adcval > adcval_max)
	    adcval_max = adcval;

	  adcval_dif = adcval_max - adcval;
	  USI_UART_Transmit_Byte ('D');
	  transmit_uint16_usi (adcval_dif);
	  long_delay (100);
	  USI_UART_Transmit_Byte ('V');
	  transmit_uint16_usi (adcval);
	  long_delay (100);
	  USI_UART_Transmit_Byte ('M');
	  transmit_uint16_usi (adcval_max);

	  if (charged_seconds >= 60)
	    {
	      charged_seconds -= 60;
	      charged_minutes++;
	    }
	  if (charged_minutes > timeoutminutes)
	    USI_UART_Transmit_Byte ('T');

	  transmit_chars ("charged_minutes: ");
	  transmit_uint16_usi (charged_minutes);

	}
      long_delay (1000);
      LED_GUE_ON;
      if (charge_error)
	USI_UART_Transmit_Byte ('E');
      else
	USI_UART_Transmit_Byte ('F');

      while ((adcval > 400) && !charge_error)
	{
	  adcval = ReadChannel (0);
	}
      LED_GUE_OFF;
      charge_error = 0;
      // Ladeende, auf Ausgang zurueck
      long_delay (1000);
    }
}
