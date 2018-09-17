#include "lcd_4b.h"

#include <avr/delay.h>






static void lcd4b_nibble( uint8_t d )
{
  LCD4b_DATA_OUT=(d& 0xF0);
  LCD4b_ENABLE_HI;
  _delay_us( 5);			// 10us
  LCD4b_ENABLE_LO;
}


static void lcd4b_send_char( uint8_t d )
{
	LCD4b_RSDS_HI; 
//	_delay_us( 5 );
  lcd4b_nibble( d );
  lcd4b_nibble( d<<4 );
  _delay_us( 45 );			// 45us
}


void lcd4b_send_cmd( uint8_t d )
{
  LCD4b_RSDS_LO;
  lcd4b_nibble( d );
  lcd4b_nibble( d<<4 );
  _delay_us( 45 );			// 45us
  switch( d ){
    case 1:
    case 2:
    case 3: _delay_ms( 2 );		// wait 2ms
  }
}

void lcd4b_putc(const char c)
{
	lcd4b_send_char(c);
}

void lcd4b_puts(const char *s)
{
    register char c;

    while ( (c = *s++) ) {
        lcd4b_putc(c);
    }

}// lcd_puts 

void lcd4b_putint(uint8_t zahl)
{
char string[4];
  int8_t i;                             // schleifenzähler
 
  string[3]='\0';                       // String Terminator
  for(i=2; i>=0; i--) 
  {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
   lcd4b_puts(string);
}


void lcd4b_putint2(uint8_t zahl)	//zweistellige Zahl
{
	char string[3];
	int8_t i;								// Schleifenzähler
	zahl%=100;								// 2 hintere Stelle
	//  string[4]='\0';                     // String Terminator
	string[2]='\0';							// String Terminator
	for(i=1; i>=0; i--) {
		string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
		zahl /= 10;
	}
	lcd4b_puts(string);
}

void lcd4b_putint1(uint8_t zahl)	//einstellige Zahl
{
	//char string[5];
	char string[2];
	zahl%=10;								//  hinterste Stelle
	string[1]='\0';							// String Terminator
	string[0]=zahl +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
	lcd4b_puts(string);
}



void lcd4b_gotoxy(uint8_t x, uint8_t y)
{
    if ( y==0 ) 
		{
		
        lcd4b_send_cmd((1<<LCD_DDRAM)+LCD_START_LINE1+x);
		;
		}
    else
	{
       
		lcd4b_send_cmd((1<<LCD_DDRAM)+LCD_START_LINE2+x);
		
		}

}// lcd_gotoxy 

void lcd4b_cls(void)   
{
	lcd4b_send_cmd(0x01);
	_delay_ms(3);			// dauert eine Weile, Wert ausprobiert

	lcd4b_send_cmd(0x02);
	
	_delay_ms(3);			// dauert eine Weile, Wert ausprobiert
}

void lcd4b_clr_line(uint8_t Linie)
{
	lcd4b_gotoxy(0,Linie);
	uint8_t i=0;
	for (i=0;i<LCD_DISP_LENGTH;i++)
	{
		lcd4b_putc(' ');
	}
	lcd4b_gotoxy(0,Linie);
	
}	// Linie Loeschen


 void lcd4b_puthex(uint8_t zahl)
{
	//char string[5];
	char string[3];
	uint8_t i,l,h;                             // schleifenzähler
	
	string[2]='\0';                       // String Terminator
	l=(zahl % 16);
	if (l<10)
	string[1]=l +'0';  
	else
	{
	l%=10;
	string[1]=l + 'A'; 
	
	}
	zahl /=16;
	h= zahl % 16;
	if (h<10)
	string[0]=h +'0';  
	else
	{
	h%=10;
	string[0]=h + 'A'; 
	}

	lcd4b_puts(string);
}

void lcd4b_put_zeit(uint8_t minuten, uint8_t stunden)
{
	//							13:15
	uint8_t i; 
	if (stunden< 10)
	{
		//	lcd_putc(' ');
	}
	
	char zeitString[6];
	zeitString[5]='\0';
	
	//	Minuten einsetzen
	zeitString[4]=(minuten % 10) +'0';	//hinterste Stelle
	if (minuten>9)
	{
		minuten/=10;
		zeitString[3]=(minuten % 10) +'0';
	}
	else
	{
		zeitString[3]='0';
	}
	 
	zeitString[2]=':';
	
	//	Stunden einsetzen
	zeitString[1]=(stunden % 10) +'0'; 
	if (stunden>9)
	{		
		stunden/=10;
		zeitString[0]=(stunden % 10) +'0';
	}
	else
	{
		zeitString[0]='0';
	}
	 
	
	lcd4b_puts(zeitString);
}


void lcd4b_data( uint8_t d )
{
  LCD4b_RSDS_HI;
  lcd4b_send_char( d );
}


void lcd4b_text( uint8_t *t )
{
  while( *t ){
    lcd4b_data( *t );
    t++;
  }
}


void lcd4b_init( void )
{
  LCD4b_DATA_DDR |= (1<<D4)|(1<<D5)|(1<<D6)|(1<<D7); // PIN 4-7 Output
  LCD4b_CTRL_DDR |= (1<<LCD4b_RSDS_PIN)|(1<<LCD4b_RW_PIN)|(1<<LCD4b_ENABLE_PIN);
	

  LCD4b_CTRL_OUT &= ~(1<<LCD4b_RW_PIN);		// TRW immer LO
  LCD4b_ENABLE_LO;
  LCD4b_RSDS_LO;				// send commands
  
  _delay_ms( 30 );			// wait 15ms

  lcd4b_nibble( 0x30 );		//Befehl: 8-bit-Interface an LCD
  _delay_ms( 6 );			// wait >4.1ms

  lcd4b_nibble( 0x30 );		// Wiederholung des Befehls: 8-bit-Interface an LCD
  _delay_us( 100 );			// wait >100us

  lcd4b_nibble( 0x30 );		// Wiederholung des Befehls: 8-bit-Interface an LCD
  _delay_us( 100 );			// wait >100us

  lcd4b_nibble( 0x20 );		// 4 bit mode
  _delay_us( 100 );			// wait >100us
  
  lcd4b_send_cmd( 0x08 );		// display off
  _delay_us( 100 );			// wait >100us

  lcd4b_send_cmd( 0x28 );			// 2 lines 5*7
  _delay_us( 100 );			// wait >100us
  
  lcd4b_send_cmd( 0x08 );			// display off
  _delay_us( 100 );			// wait >100us
  
  lcd4b_send_cmd( 0x01 );			// display clear
  _delay_us( 100 );			// wait >100us
  
  lcd4b_send_cmd( 0x06 );                  // cursor increment
  _delay_us( 100 );			// wait >100us
	lcd4b_send_cmd( 0x0C );			// on, no cursor, no blink
  _delay_us( 100 );			// wait >100us
}


void lcd4b_pos( uint8_t line, uint8_t column )
{
  if( line & 1 )
    column += 0x40;

  lcd4b_send_cmd( 0x80 + column );
}
