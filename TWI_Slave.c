//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 09.07.2018.
//  Copyright __Ruedi Heimlicher__ 2018. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "lcd.c"

//***********************************
//Reset							*
//									*
//***********************************

#define TWI_PORT		PORTD
#define TWI_PIN		PIND
#define TWI_DDR		DDRD


#define LOOPLEDPIN            0        // Blink-LED

#define RASPISUPPLYPIN        1     // Eingang vom Raspi: Betriebsspannung
#define RASPITAKTPIN			   2		// PCINT2

#define OSZIPIN               3
#define REPORTPIN             3       // Wie OSZI. Meldet Reset an Webserver, active LO

#define RELAISPIN             4        // Schaltet Relais


#define DELTA                 0x28   // 10s: Fehlercounter: Zeit bis Reset ausgeloest wird
#define RESETFAKTOR           3       // Vielfaches von DELTA

#define SHUTDOWNFAKTOR        3     //faktor fuer shutdown des Raspi
#define KILLFAKTOR            1     //faktor fuer Zeit bis zum Ausschalten


#define REBOOTFAKTOR          10

#define RESETDELAY            2   // Waitcounter: Blockiert wiedereinschalten
#define RESTARTFAKTOR         1  // faktor fuer Raspi-Restart


#define WAIT                  0
#define SHUTDOWNWAIT          1
#define KILLWAIT              2
#define RELAXWAIT             3
#define RESTARTWAIT           4 // gesetzt, wenn Raspi neu startet. 
#define REBOOTWAIT            5 // gesetzt, wenn SDA zulange LO ist
#define FIRSTRUN              6 // Warten auf Raspi
#define CHECK                 7 // in ISR gesetzt, resetcount soll erhoeht werden


#define Raspi_LO_MAX            0x8000
#define Raspi_HI_MAX            0xFFFF

/* AVR port and pins connected to '164 and/or LCD */
#define LCD_PORT                 PORTD
#define LCD_DDR                  DDRD

#define LCD_RSDS_PIN            5
#define LCD_ENABLE_PIN          6
#define LCD_CLOCK_PIN           7


void delay_ms(unsigned int ms);

volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	resetcount=0;
volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	rebootdelaycount=0; // Zaehler fuer Zeit, die der Raspi fuer Reboot braucht 

volatile uint8_t statusflag=0;

volatile uint16_t	restartcount=0; // counter fuer Restart-Zeit

volatile uint16_t   firstruncount=0; // warten auf Raspi bei plugin

void slaveinit(void)
{
   
   TCCR0 |= (1<<CS02); // attiny
   
   TWI_DDR |= (1<<LOOPLEDPIN);
   TWI_DDR |= (1<<RELAISPIN);       // Ausgang: Schaltet Reset-Ausgang fuer Zeit RESETDELAY auf LO
   TWI_PORT |= (1<<RELAISPIN);     // HI	
   
   TWI_DDR |= (1<<OSZIPIN);        // Ausgang
   TWI_PORT |= (1<<OSZIPIN);       // HI
   
   TWI_DDR &= ~(1<<RASPITAKTPIN);        // Eingang: Verbunden mit Raspi, misst LO-Zeit, um Stillstand zu erkennen
   TWI_PORT &= ~(1<<RASPITAKTPIN);        // HI
   
   TWI_DDR &= ~(1<<RASPISUPPLYPIN);        // Eingang: Verbunden mit Raspi~-Betriebspannung: blockiert resetter im FIRSTrun, wenn R noch OFF
   TWI_PORT &= ~(1<<RASPISUPPLYPIN);        // LO
   
   TWI_DDR &= ~(1<<PB2);
   
   
   //LCD
   LCD_DDR |= (1<<LCD_RSDS_PIN);      //Pin 4 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_PIN);   //Pin 5 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);   //Pin 6 von PORT B als Ausgang fuer LCD
   
   
   //   TWI_DDR &= ~(1<<WEBSERVERPIN);        // Eingang: Verbunden mit Webserver, empfŠngt Signal zum reset
   //   TWI_PORT |= (1<<WEBSERVERPIN);        // HI
   
   
   
   //TWI_DDR &= ~(1<<VCCPIN);	// Eingang, Abfragen von VCC von Master
   //TWI_PORT |= (1<<VCCPIN);	// HI
   
   //TWI_DDR &= ~(1<<SCLPIN);	// Eingang
   //TWI_PORT |= (1<<SCLPIN);	// HI
   
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
	{
		_delay_ms(0.96);
		ms--;
	}
}

/* Initializes the hardware timer  */
void timer_init(void)
{
	/* Set timer to CTC mode */
	//TCCR0A = (1 << WGM01);
	/* Set prescaler */
	TCCR0 = (1 << CS00)|(1 << CS02); // clock/1024
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK = (1 << TOIE0); // TOV0 Overflow
   
}


ISR(TIMER0_OVF_vect) // Aenderung an SDA
{
   statusflag |= (1<<CHECK);
   TWI_PORT ^=(1<<LOOPLEDPIN);
   //TWI_PORT ^= (1<<OSZIPIN);
}

/*
ISR(PCINT0_vect) // Potential-Aenderung 
{
   //TWI_PORT ^=(1<<OSZIPIN);
   statusflag &= ~(1<<FIRSTRUN); // Flag resetten, Raspi list gestartet
   if ((!((statusflag & (1<<WAIT)) )))// || (statusflag & (1<<REBOOTWAIT)) )))// WAIT verhindert, dass Relais von Raspi_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      
    }

}
*/
ISR(INT0_vect) // Potential-Aenderung von Raspi
{
   //TWI_PORT ^=(1<<OSZIPIN);
   statusflag &= ~(1<<FIRSTRUN); // Flag resetten, Raspi list gestartet
   if ((!((statusflag & (1<<WAIT))  || (statusflag & (1<<REBOOTWAIT)) )))// WAIT verhindert, dass Relais von Raspi_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      
   }
   
}

/*
ISR (SPI_STC_vect) // Neue Zahl angekommen
{
   OSZI_B_LO;
   if (inindex==0)
   {
      //OSZI_B_LO;
      //OSZI_B_HI;
      //isrcontrol = spi_txbuffer[inindex] ;
   }
   isrcontrol++;
   spi_rxbuffer[inindex] = SPDR;
   //isrcontrol = inindex;
   //isrcontrol +=inindex;
   SPDR = spi_txbuffer[inindex];
   //uint8_t input = SPDR;
   
   spi_rxdata=1;
   //inindex = inc(&inindex);
   inindex++;
   //inindex &= 0x0F;
   //SPI_Data_counter++;
   OSZI_B_HI;
}
*/


void main (void) 
{
   cli();
// *** 	wdt_disable();
//	MCUSR &= ~(1<<WDRF);
// *** 	wdt_reset();
//	WDTCR |= (1<<WDCE) | (1<<WDE);
//	WDTCR = 0x00;
//   WDTCR |= (1 << WDP3) | (1 << WDP0); // timer goes off every 8 seconds
	slaveinit();
   
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

   MCUCR |= (1<<ISC00); //Any logical change on INT0 generates an interrupt request.
   GICR |= (1<<INT0);
//   GIMSK |= (1<<INT0);
   
  //  PCICR |= 1<<PCIE0;
  //GIMSK |= 1<<PCIE;// attiny
   
 //   PCMSK0 |= 1<<PCINT2;
   //PCMSK |= 1<<PCINT2; // attiny
   timer_init();
   sei();
   
   lcd_gotoxy(0,0);
   lcd_puts("Raspi-Resetter");
   
   statusflag |= (1<<FIRSTRUN);
#pragma mark while
	while (1)
   {
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      if (loopcount0>=0x00AF)
      {
         //lcd_gotoxy(0, 0);
         //lcd_putint(loopcount1);
         loopcount0=0;
         //TWI_PORT ^=(1<<LOOPLEDPIN);
         loopcount1++;
         if (loopcount1 >0x4F)
         {
            //TWI_PORT ^=(1<<LOOPLEDPIN);
            loopcount1=0;
         }         
      }
      //continue;
      //statusflag =0;
      if (statusflag & (1<<CHECK))// Timer gibt Takt der Anfrage an
      {    
         lcd_gotoxy(18,0);
         lcd_puthex(statusflag);
         if (statusflag & (1<<FIRSTRUN))  //  Beim Start warten auf Betriebsspannung
         {
            //lcd_gotoxy(12,2);
            //lcd_puts("firstrun");
            // firstrun: wenn Raspi noch off: keine Aktionen
            if (TWI_PIN & (1<<RASPISUPPLYPIN)) // Raspi ist ON
            {               
               
               lcd_gotoxy(12,1);
               lcd_puts("R on ");
            }
            else // noch warten mit Aktionen
            {
               lcd_gotoxy(12,1);
               lcd_puts("R off");
               resetcount=0; // Kein Reset 
            }
         }
         else if (TWI_PIN & (1<<RASPISUPPLYPIN)) // Raspi ist ON
         {
            
         }
         
         //TWI_PORT ^=(1<<OSZIPIN);
         statusflag &= ~(1<<CHECK);
         // resetcount wird bei Aenderungen am RaspiPIN  in ISR von INT0 zurueckgesetzt. (Normalbetrieb)
         lcd_gotoxy(0,3);
         lcd_putint12(resetcount);
         
         lcd_gotoxy(6,3);
         lcd_putint12(rebootdelaycount);
         
         lcd_gotoxy(12,3);
         lcd_putint12(restartcount);
         
         
         if ((resetcount > RESETFAKTOR * DELTA) && (!(statusflag & (1<<WAIT)))   && (!(statusflag & (1<<REBOOTWAIT))))     // Zeit erreicht, kein wait-status, kein reboot-status: Reboot-vorgang nicht unterbrechen 
         {
            //TWI_PORT ^=(1<<OSZIPIN);
            // 3 Impuldse zum Abschalten
            uint8_t i = 0;
            lcd_gotoxy(0,1);
            lcd_puts("3 Imp");
            lcd_putint(RESETFAKTOR);
            lcd_putc(' ');
            lcd_putint(DELTA);
            for (i=0;i<3;i++)
            {
               TWI_PORT &= ~(1<<RELAISPIN);    // RELAISPIN LO, Reset fuer raspi
               _delay_ms(200);
               TWI_PORT |= (1<<RELAISPIN); //Ausgang wieder HI
               _delay_ms(200);
            }
           
            statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Ausgang wird von Raspi_HI nicht sofort wieder zurueckgesetzt
            delaycount = 0;
         
         }
           
         if (statusflag & (1<<WAIT))
         {
            lcd_gotoxy(0,1);
            lcd_puts("wait");

            delaycount++; // Counter fuer Warten bis Raspi-shutdown, anschliessend ausschalten: Relasipin low fuer 5 sec 
            //TWI_PORT ^=(1<<OSZIPIN);
            if (delaycount > RESETDELAY) //Raspi ist down
            {
 
               // TWI_PORT |=(1<<OSZIPIN);
              // statusflag &= ~0x1B ; // alle reset-Bits (3,4)
              statusflag &= ~0x3B ; 
               // TWI_PORT &= ~(1<<RELAISPIN); //Ausgang wieder LO
               statusflag &= ~(1<<WAIT);// WAIT zurueckgesetzt, Raspi_HI ist wieder wirksam
               statusflag |= (1<<REBOOTWAIT); //  Warten auf Ausschalten
               resetcount =0; 
               rebootdelaycount = 0;
               lcd_gotoxy(0,1);
               lcd_puts("resdelay");

            }            
         }
         else if (statusflag & (1<<REBOOTWAIT)) // reboot-procedure beginnen
         {
            lcd_gotoxy(0,2);
            lcd_puts("rebootwait");

            rebootdelaycount++; // fortlaufend incrementieren, bestimmt ablauf
            if (rebootdelaycount == DELTA * SHUTDOWNFAKTOR) // Raspi ist down
            {
               lcd_gotoxy(12,2);
               lcd_puts("shutoff");

               TWI_PORT &= ~(1<<RELAISPIN); // Ausschalten einleiten
            }
            
            if (rebootdelaycount == DELTA * (SHUTDOWNFAKTOR + KILLFAKTOR)) // Ausgeschaltet
            {
               lcd_gotoxy(12,2);
               lcd_puts("restart ");
             
               TWI_PORT |= (1<<RELAISPIN); //Ausgang wieder HI
               _delay_ms(1000); // kurz warten
               TWI_PORT &= ~(1<<RELAISPIN);    // RELAISPIN LO, Restart fuer raspi
               _delay_ms(200);
               TWI_PORT |= (1<<RELAISPIN); //Ausgang wieder HI
               statusflag |= (1<<RESTARTWAIT);
               restartcount=0; // counter fuer Restart-Zeit
               
               //statusflag &= ~(1<<REBOOTWAIT);
               
               TWI_PORT &= ~(1<<OSZIPIN);
            }
            
            if (statusflag & (1<<RESTARTWAIT))
            {
               restartcount++;
               if (restartcount > (DELTA*RESTARTFAKTOR))
               {
                  lcd_gotoxy(12,2);
                  lcd_puts("end     ");
                  //lcd_clr_line(0);
                  lcd_clr_line(1);
                  lcd_clr_line(2);
                  
                  
                  TWI_PORT |=(1<<OSZIPIN);
                  statusflag &= ~(1<<RESTARTWAIT);
                  statusflag &= ~(1<<REBOOTWAIT); // Vorgang beendet
                  statusflag |= (1<<FIRSTRUN);
                  
                  rebootdelaycount=0;
                  restartcount=0;
               }
               
            }
            
         }
         else
         {
            // resetcounter inkrementieren, Normalbetrieb
            resetcount++;
            
         }
         //TWI_PORT ^=(1<<OSZIPIN);
         
         
        /* 
         // Reset durch Webserver: WEBSERVERPIN abfragen: Reset wenn LO
         
         if (TWI_PIN & (1 << WEBSERVERPIN))
         {
            //HI, alles OK
            webserverresetcount =0;
            delaycount=0;
            statusflag &= ~(1<<WAIT);
            //           TWI_PORT &= ~(1<<RELAISPIN);
         }
         else // webserverreset inc, reset wenn Eingang vom Webserver lange genug LO ist: Fehlerfall auf Webserver
         {
            webserverresetcount++;
            TWI_PORT ^=(1<<OSZIPIN);
            if (webserverresetcount > RASPIRESETDELAY)
            {
               TWI_PORT |= (1<<RELAISPIN);    // RELAISPIN Hi, Relais schaltet aus
               statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von Raspi_HI nicht zurueckgesetzt
               
            }
            
            if (webserverresetcount > (RASPIRESETDELAY + RESETDELAY))
            {
               //TWI_PORT |=(1<<OSZIPIN);
               TWI_PORT &= ~(1<<RELAISPIN);
               statusflag &= ~(1<<WAIT);// WAIT zurueckgesetzt, Raspi_HI ist wieder wirksam
               webserverresetcount =0;
               resetcount =0;
            }
            
         }
         */
      } // if check
      
   }//while
   
   
    //return 0;
}
