//
//  RESET_Slave.c
//  RESET_Slave
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
//#include "lcd_4b.c"
//***********************************
//Reset							*
//									*
//***********************************

#define TEST 0
#define LOOPLED_PORT PORTB
#define LOOPLED_DDR  DDRB
#define LOOPLED_PIN  PINB
#define LOOPLEDPIN            0        // Blink-LED

#define WDRLEDPIN 1
#define TESTPIN               2

#define RESET_PORT		PORTD
#define RESET_PIN		   PIND
#define RESET_DDR		   DDRD

#define RASPISUPPLYPIN        1     // Eingang vom Raspi: Betriebsspannung ist ON
#define RASPITAKTPIN          2      // INT0: periodisches Signal vom shutoff-Process 
#define RASPISYSTEMPIN        3     // Eingang vom Raspi: HI wenn System ON (LO wenn Raspi ausgeschaltet, aber ev. Betriebspannung noch eingeschaltet ist: direkt Shutoff einleiten 
#define OSZIPIN               7
#define REPORTPIN             3       //  Meldet Reset an Webserver, active LO
#define TASTEPIN              4        // Schaltet Relais



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
#define LCD_PORT                 PORTB
#define LCD_DDR                  DDRB

//#define LCD_RSDS_PIN            5
//#define LCD_ENABLE_PIN          6
//#define LCD_CLOCK_PIN           7



volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	resetcount=0;
volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	rebootdelaycount=0; // Zaehler fuer Zeit, die der Raspi fuer Reboot braucht 

volatile uint8_t statusflag=0;

volatile uint16_t	restartcount=0; // counter fuer Restart-Zeit

volatile uint16_t   firstruncount=0; // warten auf Raspi bei plugin

volatile uint8_t   wdtcounter=0;
volatile uint8_t   wdt_rep_counter=0;
volatile uint8_t   wdt_isr_counter=0;
volatile uint8_t wdr_count=0;
void slaveinit(void)
{
   
   TCCR0B |= (1<<CS02); // attiny
   
   LOOPLED_DDR |= (1<<LOOPLEDPIN);
   LOOPLED_PORT |= (1<<LOOPLEDPIN);     // HI

   LOOPLED_DDR &= ~(1<<TESTPIN);    // Eingang
   LOOPLED_PORT |= (1<<TESTPIN);     // HI
   
   LOOPLED_DDR |= (1<<WDRLEDPIN);
   LOOPLED_PORT |= (1<<WDRLEDPIN);
   
   RESET_DDR |= (1<<TASTEPIN);       // Ausgang: Schaltet Reset-Ausgang fuer Zeit RESETDELAY auf LO
   RESET_PORT |= (1<<TASTEPIN);     // HI	
   //RESET_PORT &= ~(1<<TASTEPIN);
   
   //RESET_PORT &= ~(1<<RELAISPIN);     // LO   
   
   RESET_DDR |= (1<<OSZIPIN);        // Ausgang
   RESET_PORT |= (1<<OSZIPIN);       // HI
   
   RESET_DDR &= ~(1<<RASPITAKTPIN);        // Eingang: Verbunden mit Raspi, misst LO-Zeit, um Stillstand zu erkennen
   RESET_PORT &= ~(1<<RASPITAKTPIN);        // HI
   
   RESET_DDR &= ~(1<<RASPISUPPLYPIN);        // Eingang: Verbunden mit Raspi~-Betriebspannung: blockiert resetter im FIRSTrun, wenn R noch OFF
   RESET_PORT &= ~(1<<RASPISUPPLYPIN);        // LO
   
   RESET_DDR &= ~(1<<RASPISYSTEMPIN);        // Eingang: Verbunden mit Raspi-Systemausgang: HI wenn System run
   RESET_PORT &= ~(1<<RASPISYSTEMPIN);
   
   
   //RESET_DDR &= ~(1<<PB2);
   
   
   //LCD
   LCD_DDR |= (1<<LCD_RSDS_PIN);      //Pin 4 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_PIN);   //Pin 5 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);   //Pin 6 von PORT B als Ausgang fuer LCD
   
   EIMSK |= (1<<INT0); // INT0 enabled
   EICRA |= (1<<ISC00);//Any logical change on INT0 generates an interrupt request
}




/* Initializes the hardware timer  */
void timer_init(void)
{
	/* Set timer to CTC mode */
	//TCCR0A = (1 << WGM01);
	/* Set prescaler */
	TCCR0B = (1 << CS00)|(1 << CS02); // clock/1024
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK0 = (1 << TOIE0); // TOV0 Overflow

}


ISR(TIMER0_OVF_vect) // Aenderung an SDA
{
   statusflag |= (1<<CHECK);
   LOOPLED_PORT ^=(1<<LOOPLEDPIN);
   //RESET_PORT ^= (1<<OSZIPIN);
}

/*
ISR(PCINT0_vect) // Potential-Aenderung 
{
   //RESET_PORT ^=(1<<OSZIPIN);
   statusflag &= ~(1<<FIRSTRUN); // Flag resetten, Raspi list gestartet
   if ((!((statusflag & (1<<WAIT)) )))// || (statusflag & (1<<REBOOTWAIT)) )))// WAIT verhindert, dass Relais von Raspi_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      
    }

}
*/

ISR(INT0_vect) // Potential-Aenderung von Raspi-Takt
{
   
   //RESET_PORT ^=(1<<OSZIPIN);
   statusflag &= ~(1<<FIRSTRUN); // Flag resetten, Raspi ist gestartet
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

void WDT_Init(uint8_t status)
{
   unsigned char reset_flags;
   /* Read and clear reset flags */
   reset_flags = MCUSR;                // Save reset flags.
   
   cli();
   wdt_reset();
   MCUSR &= ~(1<<WDRF);
   MCUSR       = 0;                    // Clear all flags previously set.
   WDTCSR = (1<<WDCE) | (1<<WDE) ;
   WDTCSR = (1<<WDP0)|(1<<WDP1)|(1<<WDP2); // 2s
   WDTCSR |= (1<< WDIE); // muss nach wdt_enable stehen
   
   wdt_reset();
   sei();
}


ISR(WDT_vect)
{
   
   //RESET_PORT  ^= (1<<TASTEPIN);
   wdt_isr_counter++;
   for (uint8_t i=0;i<4;i++)
   {
      //LED ON
      LOOPLED_PORT |= (1<<WDRLEDPIN);
      //~0.1s delay
      _delay_ms(20);
      //LED OFF
      LOOPLED_PORT &= ~(1<<WDRLEDPIN);
      _delay_ms(80);
   }
   MCUSR &= ~(1<<WDRF);
   WDTCSR=0;
   WDTCSR   =   (1<<WDCE)|(1<<WDE ); // next WDR ohne WDIE, -> reset
 }

void main (void) 
{
   cli();
   wdt_disable();
	MCUSR &= ~(1<<WDRF);
   MCUSR       = 0;                    // Clear all flags previously set.
   wdt_disable();
   WDTCSR=0;
   
// *** 	wdt_reset();
	slaveinit();
   wdt_reset();
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

   timer_init();
   
   lcd_gotoxy(0,0);
   lcd_puts("Raspi-Resetter");

   if (TEST) // Start nach reset anzeigen
   {
      uint8_t i=0;
      for (i=0;i<3;i++)
      {
         LOOPLED_PORT &=~(1<<LOOPLEDPIN);
         _delay_ms(600);
         LOOPLED_PORT |=(1<<LOOPLEDPIN);
         _delay_ms(50);
         wdt_reset();
      }
   }
   MCUSR &= ~(1<<WDRF);
   
   //sei();
   
   lcd_gotoxy(15,0);
   lcd_puts("go");

   //_delay_ms(400);
   statusflag |= (1<<FIRSTRUN);
   //lcd_cls();
   WDT_Init(1);
    sei();

#pragma mark while
	while (1)
   {
      if (TEST)
      {
         uint8_t i=0;
         //wdt_reset();
         if((LOOPLED_PIN & (1<<TESTPIN))) 
         {
            for (i=0;i<24;i++)
            {
               //wdt_reset();
               _delay_ms(200);     
               wdt_rep_counter++;
               lcd_gotoxy(0,1);
               lcd_puthex(wdt_rep_counter);
            }
            //RESET_PORT |=(1<<TASTEPIN);
            _delay_ms(100); 
            //lcd_gotoxy(6,1);
            //lcd_puthex(wdt_isr_counter);
         }
         wdt_rep_counter=0;
      }
      else 
      {
         wdtcounter=0;
         wdt_reset();
      }
      //RESET_PORT |=(1<<TASTEPIN);
      //Blinkanzeige
      loopcount0++;
      if (loopcount0>=0x00AF)
      {
         //lcd_gotoxy(0, 0);
         //lcd_putint(loopcount1);
         loopcount0=0;
         //RESET_PORT ^=(1<<LOOPLEDPIN);
         loopcount1++;
         if (loopcount1 >0x4F)
         {
            //RESET_PORT ^=(1<<LOOPLEDPIN);
            loopcount1=0;
            //RESET_PORT  ^= (1<<TASTEPIN);
            //_delay_ms(20);

         }         
      }
      
       //continue;
      //statusflag =0;
      if (statusflag & (1<<CHECK))// Timer gibt Takt der Anfrage an
      {    
         lcd_gotoxy(18,0);
         //lcd_puthex(statusflag);
         lcd_gotoxy(6,1);
         lcd_puthex(wdt_isr_counter);
         if (statusflag & (1<<FIRSTRUN))  //  Beim Start warten auf Takt vom Raspi (anstatt:Betriebsspannung)
         {
            
            //lcd_gotoxy(12,2);
            //lcd_puts("firstrun");
            // firstrun: wenn Raspi noch off: keine Aktionen
            resetcount=0; // Kein Reset 
            /*
             if (RESET_PIN & (1<<RASPISUPPLYPIN)) // Raspi ist ON
             {               
             
             lcd_gotoxy(0,1);
             lcd_puts("R on ");
             }
             else // noch warten mit Aktionen
             {
             lcd_gotoxy(0,1);
             lcd_puts("R off");
             resetcount=0; // Kein Reset 
             
             }
             */
            
            //
              
            //
         }
         /*
         if (!(RESET_PIN & (1<<RASPISYSTEMPIN))) // System ist OFF
         {
            lcd_gotoxy(0,2);
            lcd_puts("syst 0");
            
            statusflag |= (1<<REBOOTWAIT); // Ausschaltprocess ueberspringen
            if (!(RESET_PIN & (1<<RASPISUPPLYPIN))) // Keine Betriebsspannung, Raspi ist OFF: sofort starten
            {
               lcd_gotoxy(10,2);
               lcd_puts("suppl 0");
               //rebootdelaycount = DELTA * (SHUTDOWNFAKTOR + KILLFAKTOR)-1;
            }
            else // zuerst Betriebsspannung ausschalten
            {
               lcd_gotoxy(10,2);
               lcd_puts("suppl 1");
               
               //rebootdelaycount = DELTA * (SHUTDOWNFAKTOR)-1;
            }
         }
         */
         
         
         //RESET_PORT ^=(1<<OSZIPIN);
         statusflag &= ~(1<<CHECK);
         // resetcount wird bei Aenderungen am RaspiPIN  in ISR von INT0 zurueckgesetzt. (Normalbetrieb)
         lcd_gotoxy(17,1);
         lcd_putint(resetcount);
         
         lcd_gotoxy(6,3);
         lcd_putint12(rebootdelaycount);
         
         lcd_gotoxy(12,3);
         lcd_putint12(restartcount);
         if ((RESET_PIN & (1<<RASPISYSTEMPIN))) // System ist ON
            
         {
            lcd_gotoxy(0,3);
            lcd_puts("Sys ON");
         }
         else
         {
            lcd_gotoxy(0,3);
            lcd_puthex(RESET_PIN);
            lcd_puts("Sys OFF");
         }

 //        wdt_reset();
         if ((resetcount > RESETFAKTOR * DELTA) && (!(statusflag & (1<<WAIT)))   && (!(statusflag & (1<<REBOOTWAIT))))     // Zeit erreicht, kein wait-status, kein reboot-status: Reboot-vorgang nicht unterbrechen 
         {
            
            //RESET_PORT ^=(1<<OSZIPIN);
             {
               // 3 Impuldse zum Abschalten
               uint8_t i = 0;
               lcd_gotoxy(0,2);
               lcd_puts("3 Imp");
               lcd_putint(RESETFAKTOR);
               lcd_putc(' ');
               lcd_putint(DELTA);
               for (i=0;i<3;i++)
               {
                  RESET_PORT &= ~(1<<TASTEPIN);    // RELAISPIN LO, Reset fuer raspi
                  _delay_ms(300);
                  RESET_PORT |= (1<<TASTEPIN); //Ausgang wieder HI
                  _delay_ms(300);
                  wdt_reset();
               }
               statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Ausgang wird von Raspi_HI nicht sofort wieder zurueckgesetzt
               delaycount = 0;
            }
         }
         
         
         if (statusflag & (1<<WAIT))
         {
            lcd_gotoxy(0,1);
            lcd_puts("wait");
            
            delaycount++; // Counter fuer Warten bis Raspi-shutdown, anschliessend ausschalten: Relasipin low fuer 5 sec 
            //RESET_PORT ^=(1<<OSZIPIN);
            if (delaycount > RESETDELAY) //Raspi ist down
            {
               
               // RESET_PORT |=(1<<OSZIPIN);
               // statusflag &= ~0x1B ; // alle reset-Bits (3,4)
               statusflag &= ~0x3B ; 
               // RESET_PORT &= ~(1<<RELAISPIN); //Ausgang wieder LO
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
            lcd_gotoxy(10,1);
            lcd_puts("rebwait");
            
            
            rebootdelaycount++; // fortlaufend incrementieren, bestimmt ablauf
            if (rebootdelaycount == DELTA * SHUTDOWNFAKTOR) // Raspi ist down
            {
               lcd_gotoxy(0,1);
               lcd_puts("shut_off");
               
               RESET_PORT &= ~(1<<TASTEPIN); // Ausschalten einleiten, Relaispin 5s down
            }
            
            if (rebootdelaycount == DELTA * (SHUTDOWNFAKTOR + KILLFAKTOR)) // Ausgeschaltet
            {
               lcd_gotoxy(0,1);
               lcd_puts("restart ");
               
               RESET_PORT |= (1<<TASTEPIN); //Ausgang wieder HI
               _delay_ms(1000); // kurz warten
               wdt_reset();
               RESET_PORT &= ~(1<<TASTEPIN);    // RELAISPIN LO, Restart fuer raspi
               _delay_ms(200);
               wdt_reset();
               RESET_PORT |= (1<<TASTEPIN); //Ausgang wieder HI
               statusflag |= (1<<RESTARTWAIT);
               restartcount=0; // counter fuer Restart-Zeit
               RESET_PORT &= ~(1<<OSZIPIN);
            }
            
            if (statusflag & (1<<RESTARTWAIT))
            {
               restartcount++;
               if (restartcount > (DELTA*RESTARTFAKTOR))
               {
                  lcd_gotoxy(0,1);
                  lcd_puts("end      ");
                  //lcd_clr_line(0);
                  lcd_clr_line(1);
                  lcd_clr_line(2);
                  
                  RESET_PORT |=(1<<OSZIPIN);
                  statusflag &= ~(1<<RESTARTWAIT);
                  statusflag &= ~(1<<REBOOTWAIT); // Vorgang beendet
                  statusflag |= (1<<FIRSTRUN);
                  
                  rebootdelaycount=0;
                  restartcount=0;
               }
            }
         }
         else // Kein WAIT, kein REBOOTWAIT: resetcounter inkrementieren, Normalbetrieb
         {
            wdt_reset();
            resetcount++;
         }
         //RESET_PORT ^=(1<<OSZIPIN);
         
         
         /* 
          // Reset durch Webserver: WEBSERVERPIN abfragen: Reset wenn LO
          
          if (RESET_PIN & (1 << WEBSERVERPIN))
          {
          //HI, alles OK
          webserverresetcount =0;
          delaycount=0;
          statusflag &= ~(1<<WAIT);
          //           RESET_PORT &= ~(1<<RELAISPIN);
          }
          else // webserverreset inc, reset wenn Eingang vom Webserver lange genug LO ist: Fehlerfall auf Webserver
          {
          webserverresetcount++;
          RESET_PORT ^=(1<<OSZIPIN);
          if (webserverresetcount > RASPIRESETDELAY)
          {
          RESET_PORT |= (1<<RELAISPIN);    // RELAISPIN Hi, Relais schaltet aus
          statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von Raspi_HI nicht zurueckgesetzt
          
          }
          
          if (webserverresetcount > (RASPIRESETDELAY + RESETDELAY))
          {
          //RESET_PORT |=(1<<OSZIPIN);
          RESET_PORT &= ~(1<<RELAISPIN);
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
