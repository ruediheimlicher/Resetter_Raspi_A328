#ifndef _lcd_4b_h_
#define _lcd_4b_h_

#define	LCD4b_DATA_OUT		PORTD
#define	LCD4b_IN				PIND
#define	LCD4b_DATA_DDR		DDRD

#define	D4			4
#define	D5			5
#define	D6			6
#define	D7			7

#define	LCD4b_CTRL_OUT		PORTB
#define	LCD4b_CTRL_DDR		DDRB

#define	LCD4b_RSDS_PIN			5
#define	LCD4b_RW_PIN				6 // Hier immer LO (schreiben)
#define	LCD4b_ENABLE_PIN			7

#define LCD4b_ENABLE_HI	LCD4b_CTRL_OUT |= (1<<LCD4b_ENABLE_PIN)		// E HI
#define LCD4b_ENABLE_LO	LCD4b_CTRL_OUT &= ~(1<<LCD4b_ENABLE_PIN)	// E LO

#define LCD4b_RSDS_HI	LCD4b_CTRL_OUT |= (1<<LCD4b_RSDS_PIN)		// RSDS HI
#define LCD4b_RSDS_LO	LCD4b_CTRL_OUT &= ~(1<<LCD4b_RSDS_PIN)	// RSDS LO


//	Register Select Constants
#define	DATA_REGISTER		0
#define	COMMAND_REGISTER	1

#define LCD_LINES           2     /**< number of visible lines of the display */
#define LCD_DISP_LENGTH    20     /**< visibles characters per line of the display */

#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
#define LCD_DDRAM             7      /* DB7: set DD RAM address             */




void lcd4b_send_cmd( uint8_t d );
void lcd4b_data( uint8_t d );
void lcd4b_text( uint8_t *t );
void lcd4b_init( void );
void lcd4b_pos( uint8_t line, uint8_t column );

#endif
