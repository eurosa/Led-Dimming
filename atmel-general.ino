/****************************************************************************
    LCD-AVR-4d.c  - Use an HD44780U based LCD with an Atmel ATmega processor

    Copyright (C) 2013 Donald Weiman    (weimandn@alfredstate.edu)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/****************************************************************************
      File:    LCD-AVR-4d.c
      Date:    September 16, 2013

      Target:    ATmega328
      Compiler:  avr-gcc (AVR Studio 6)
      Author:    Donald Weiman

      Summary:    4-bit data interface, busy flag not implemented.
                  Any LCD pin can be connected to any available I/O port.
                  Includes a simple write string routine.
*/
/******************************* Program Notes ******************************

            This program uses a 4-bit data interface but does not use the
            busy flag to determine when the LCD controller is ready.  The
            LCD RW line (pin 5) is not connected to the uP and it must be
            connected to GND for the program to function.

            All time delays are longer than those specified in most datasheets
              in order to accommodate slower than normal LCD modules.  This
              requirement is well documented but almost always ignored.  The
              information is in a note at the bottom of the right hand
              (Execution Time) column of the instruction set.

  ***************************************************************************

            The four data lines as well as the two control lines may be
              implemented on any available I/O pin of any port.  These are
              the connections used for this program:

                 -----------                   ----------
                | ATmega328 |                 |   LCD    |
                |           |                 |          |
                |        PD7|---------------->|D7        |
                |        PD6|---------------->|D6        |
                |        PD5|---------------->|D5        |
                |        PD4|---------------->|D4        |
                |           |                 |D3        |
                |           |                 |D2        |
                |           |                 |D1        |
                |           |                 |D0        |
                |           |                 |          |
                |        PB1|---------------->|E         |
                |           |         GND --->|RW        |
                |        PB0|---------------->|RS        |
                 -----------                   ----------

  **************************************************************************/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define endos_copy_port   PORTD
#define endos_copy_bit    PD6
#define endos_copy_ddr    DDRD
#define endos_copy_pin    PIND

#define BUTTON_MASK_END_OS  (1<<PD6)
#define BUTTON_PIN_END_OS  PIND
#define BUTTON_PORT_END_OS  PORTD

#define white_upbtn_port   PORTC
#define white_upbtn_bit    PC1
#define white_upbtn_ddr    DDRC
#define white_upbtn_pin    PINC

#define BUTTON_MASK_WH_UP  (1<<PC1)
#define BUTTON_PIN_WH_UP   PINC
#define BUTTON_PORT_WH_UP  PORTC

#define BUTTON_MASK_WH_DWN  (1<<PC0)
#define BUTTON_PIN_WH_DWN   PINC
#define BUTTON_PORT_WH_DWN  PORTC

#define white_dwnbtn_port   PORTC
#define white_dwnbtn_bit    PC0
#define white_dwnbtn_ddr    DDRC
#define white_dwnbtn_pin    PINC

#define yellow_upbtn_port   PORTC
#define yellow_upbtn_bit     PC2
#define yellow_upbtn_ddr    DDRC
#define yellow_upbtn_pin    PINC


#define BUTTON_MASK_YL_UP  (1<<PC2)
#define BUTTON_PIN_YL_UP   PINC
#define BUTTON_PORT_YL_UP  PORTC

#define BUTTON_MASK_YL_DWN  (1<<PB5)
#define BUTTON_PIN_YL_DWN   PINB
#define BUTTON_PORT_YL_DWN  PORTB


#define yellow_dwnbtn_port   PORTB
#define yellow_dwnbtn_bit    PB5
#define yellow_dwnbtn_ddr    DDRB
#define yellow_dwnbtn_pin    PINB

#define power_onoffbtn_port   PORTD
#define power_onoffbtn_bit    PD7
#define power_onoffbtn_ddr    DDRD
#define power_onoffbtn_pin    PIND

#define BUTTON_MASK_ON_OFF  (1<<PD7)
#define BUTTON_PIN_ON_OFF   PIND
#define BUTTON_PORT_ON_OFF  PORTD

#define power_led_port      PORTB
#define power_led_ddr       DDRB
#define power_led_pin       PINB
#define power_led_bit       PB0

#define yellow_led_port PORTB
#define yellow_led_ddr  DDRB
#define yellow_led_pin  PINB
#define yellow_led_bit  PB2

#define white_led_port PORTB
#define white_led_ddr  DDRB
#define white_led_pin  PINB
#define white_led_bit  PB1

#define DEBOUNCE_TIME 25 // time to wait while "de-bouncing" button 
#define LOCK_INPUT_TIME 300 // time to wait after a button press


// LCD interface (should agree with the diagram above)
//   make sure that the LCD RW pin is connected to GND
#define lcd_D7_port     PORTC                   // lcd D7 connection
#define lcd_D7_bit      PC3
#define lcd_D7_ddr      DDRC

#define lcd_D6_port     PORTC                   // lcd D6 connection
#define lcd_D6_bit      PC4
#define lcd_D6_ddr      DDRC

#define lcd_D5_port     PORTC                   // lcd D5 connection
#define lcd_D5_bit      PC5
#define lcd_D5_ddr      DDRC

#define lcd_D4_port     PORTD                   // lcd D4 connection
#define lcd_D4_bit      PD2
#define lcd_D4_ddr      DDRD

#define lcd_E_port      PORTD                   // lcd Enable pin
#define lcd_E_bit       PD3
#define lcd_E_ddr       DDRD

#define lcd_RS_port     PORTD                   // lcd Register Select pin
#define lcd_RS_bit      PD4
#define lcd_RS_ddr      DDRD

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
//#define   lcd_LineThree   0x14                  // start of line 3 (20x4)
//#define   lcd_lineFour    0x54                  // start of line 4 (20x4)
//#define   lcd_LineThree   0x10                  // start of line 3 (16x4)
//#define   lcd_lineFour    0x50                  // start of line 4 (16x4)

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// Program ID
uint8_t program_author[]   = "Radha Ma";
uint8_t program_version[]  = "LCD-AVR(gcc)";
uint8_t program_date[]     = "Sep 16, 2013";
uint8_t maxBrightness = 255; //this can be any number - it's the number of steps between dimmest and brightest.

// variables will change:
uint8_t brightness  = 0;
uint8_t interval[] = "25";

unsigned int addr_white_up = 0;
unsigned int addr_white_dwn = 1;
unsigned int addr_yellow_up = 2;
unsigned int addr_yellow_dwn = 3;
uint8_t  rb_wh_up, wb_wh_up = 'a';
uint8_t  rb_wh_dwn, wb_wh_dwn = 'b';
uint8_t  rb_yellow_up, wb_yellow_up = 'c';
uint8_t  rb_yellow_dwn, wb_yellow_dwn = 'd';

// Function Prototypes
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(uint8_t *);
void lcd_init_4d(void);

// Variable to tell main that the button is pressed (and debounced).
// Main will clear it after a detected button press.
volatile uint8_t button_wh_down;
volatile uint8_t button_wh_up;
volatile uint8_t button_yl_up;
volatile uint8_t button_yl_down;
volatile uint8_t button_endos;
volatile uint8_t button_on_off;

/******************************* Main Program Code *************************/
int main(void)
{
  uint8_t brightness = 0;
  //InitPWM();
  //initTimer();
  //DDRB |= (1 << PB1) | (1 << PB2);
  /************************************************************************
                         Endoscopy Push Button

  ************************************************************************/
// Enable internal pullup resistor on the input pin
    BUTTON_PORT_WH_UP |= BUTTON_MASK_WH_UP;
    BUTTON_PORT_WH_DWN |= BUTTON_MASK_WH_DWN;
    BUTTON_PORT_YL_UP |= BUTTON_MASK_YL_UP;
    BUTTON_PORT_YL_DWN |= BUTTON_MASK_YL_DWN;
    BUTTON_PORT_ON_OFF |= BUTTON_MASK_ON_OFF;
    BUTTON_PORT_END_OS |= BUTTON_MASK_END_OS;
   
  power_led_ddr |= (1 << power_led_bit); // Power LED as output
  yellow_led_ddr |= (1 << yellow_led_bit);
  white_led_ddr |= (1 << white_led_bit);


  // configure the microprocessor pins for the data lines
  lcd_D7_ddr |= (1 << lcd_D7_bit);                // 4 data lines - output
  lcd_D6_ddr |= (1 << lcd_D6_bit);
  lcd_D5_ddr |= (1 << lcd_D5_bit);
  lcd_D4_ddr |= (1 << lcd_D4_bit);

  // configure the microprocessor pins for the control lines
  lcd_E_ddr |= (1 << lcd_E_bit);                  // E line - output
  lcd_RS_ddr |= (1 << lcd_RS_bit);                // RS line - output

  // initialize the LCD controller as determined by the defines (LCD instructions)
  lcd_init_4d();                                  // initialize the LCD display for a 4-bit interface

  // display the first line of information
  lcd_write_string_4d(program_author);

  // set cursor to start of second line
  lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
  _delay_us(80);                                  // 40 uS delay (min)

  // display the second line of information
  lcd_write_string_4d(program_version);

  /*******************************************************************************************************************

                                               Write to eeprom

  *******************************************************************************************************************/
  eeprom_write_byte( (uint8_t*)addr_white_up, wb_wh_up );
  eeprom_write_byte( (uint8_t*)addr_white_dwn, wb_wh_dwn );
  eeprom_write_byte( (uint8_t*)addr_yellow_up, wb_yellow_up );
  eeprom_write_byte( (uint8_t*)addr_yellow_dwn, wb_yellow_dwn );



  // endless loop
  while (1)
  {
    
    debounce_power_on();
    debounce_endoscopy();
    debounce_yello_up();
    debounce_white_up();
    debounce_white_down();
    debounce_yellow_down();

        // Check if the button is pressed.
        if (button_wh_up)
        {
      // Clear flag
      button_wh_up = 0;
            // Toggle the LED
            white_led_port ^= (1<<white_led_bit);

             //SetPWMOutput(brightness);
      
      Wait();
        }


         // Check if the button is pressed.
        if (button_wh_down)
        {
      // Clear flag
      button_wh_down = 0;
            // Toggle the LED
            white_led_port ^= (1<<white_led_bit);

             //SetPWMOutput(brightness);
      
      Wait();
        }


           // Check if the button is pressed.
        if (button_yl_up)
        {
      // Clear flag
      button_yl_up = 0;
            // Toggle the LED
            white_led_port ^= (1<<white_led_bit);

             //SetPWMOutput(brightness);
      
      Wait();
        }

    // Check if the button is pressed.
        if (button_yl_down)
        {
      // Clear flag
      button_yl_down = 0;
            // Toggle the LED
            white_led_port ^= (1<<white_led_bit);

             //SetPWMOutput(brightness);
      
      Wait();
        }
    
    /*if (white_upbtn_state()) {
      
      SetPWMOutput(brightness);
      brightness= brightness+25;
      Wait();

    }*/

   /* if (white_dwnbtn_state()) {
      brightness= brightness-25;
      SetPWMOutput(brightness);
      Wait();

    }*/

    
    /* for(brightness=0;brightness<255;brightness++)
      {
        SetPWMOutput(brightness);
        Wait();
      }*/


    

    /*if((white_upbtn_pin & 1<< white_upbtn_bit) ){
      brightness = brightness ;
       _delay_ms(1000);

      lcd_init_4d();                                  // initialize the LCD display for a 4-bit interface


      // set cursor to start of second line
       //lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
       _delay_ms(1000);                                  // 40 uS delay (min)

      // display the second line of information
      lcd_write_string_4d(interval);
      //lcd_write_character_4d(0b010100);
      _delay_ms(1000);

      }*/
    //rb8 = eeprom_read_byte((uint8_t*)addr)
    if (button_state_endoscopy()) {
      rb_wh_up = eeprom_read_byte((uint8_t*)addr_white_up);
      rb_wh_dwn = eeprom_read_byte((uint8_t*)addr_white_dwn);
      rb_yellow_up = eeprom_read_byte((uint8_t*)addr_yellow_up);
      rb_yellow_dwn = eeprom_read_byte((uint8_t*)addr_yellow_dwn);
      lcd_write_character_4d(rb_yellow_dwn);
      lcd_init_4d();
      //lcd_write_character_4d(rb_wh_up);
      _delay_ms(3000);
      lcd_write_character_4d(rb_yellow_dwn);
      lcd_init_4d();

    }

 

  }

  return 0;
}


void SetPWMOutput(uint8_t duty)
{
  OCR1A = duty;
}

void Wait()
{
  _delay_ms(1000);
}

unsigned char button_state_power_on_off()
{
  /* the button is pressed when BUTTON1 bit is clear */
  if (!(power_onoffbtn_pin & (1 << power_onoffbtn_bit)))
  {
    _delay_ms(DEBOUNCE_TIME);
    if (!(power_onoffbtn_pin & (1 << power_onoffbtn_bit))) return 1;
  }
  return 0;
}

//white_dwnbtn_pin

unsigned char white_upbtn_state()
{
  /* the button is pressed when BUTTON1 bit is clear */
  if (!(white_upbtn_pin & 1 << white_upbtn_bit))
  {
    _delay_ms(DEBOUNCE_TIME);
    if (!(white_upbtn_pin & 1 << white_upbtn_bit)) return 1;
  }
  return 0;
}


unsigned char white_dwnbtn_state()
{
  /* the button is pressed when BUTTON1 bit is clear */
  if (!(white_dwnbtn_pin & 1 << white_dwnbtn_bit))
  {
    _delay_ms(DEBOUNCE_TIME);
    if (!(white_dwnbtn_pin & 1 << white_dwnbtn_bit)) return 1;
  }
  return 0;
}



unsigned char button_state_endoscopy()
{
  /* the button is pressed when BUTTON1 bit is clear */
  if (!(endos_copy_pin & (1 << endos_copy_bit)))
  {
    _delay_ms(DEBOUNCE_TIME);
    if (!(endos_copy_pin & (1 << endos_copy_bit))) return 1;
  }
  return 0;
}




void initTimer() {

  ICR1 = 40000;
  //OCR1A = n * x;
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

}
/******************************* End of Main Program Code ******************/

/*============================== 4-bit LCD Functions ======================*/
/*
  Name:     lcd_init_4d
  Purpose:  initialize the LCD module for a 4-bit data interface
  Entry:    equates (LCD instructions) set up for the desired operation
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_init_4d(void)
{
  // Power-up delay
  _delay_ms(100);                                 // initial 40 mSec delay

  // IMPORTANT - At this point the LCD module is in the 8-bit mode and it is expecting to receive
  //   8 bits of data, one bit on each of its 8 data lines, each time the 'E' line is pulsed.
  //
  // Since the LCD module is wired for the 4-bit mode, only the upper four data lines are connected to
  //   the microprocessor and the lower four data lines are typically left open.  Therefore, when
  //   the 'E' line is pulsed, the LCD controller will read whatever data has been set up on the upper
  //   four data lines and the lower four data lines will be high (due to internal pull-up circuitry).
  //
  // Fortunately the 'FunctionReset' instruction does not care about what is on the lower four bits so
  //   this instruction can be sent on just the four available data lines and it will be interpreted
  //   properly by the LCD controller.  The 'lcd_write_4' subroutine will accomplish this if the
  //   control lines have previously been configured properly.

  // Set up the RS and E lines for the 'lcd_write_4' subroutine.
  lcd_RS_port &= ~(1 << lcd_RS_bit);              // select the Instruction Register (RS low)
  lcd_E_port &= ~(1 << lcd_E_bit);                // make sure E is initially low

  // Reset the LCD controller
  lcd_write_4(lcd_FunctionReset);                 // first part of reset sequence
  _delay_ms(10);                                  // 4.1 mS delay (min)

  lcd_write_4(lcd_FunctionReset);                 // second part of reset sequence
  _delay_us(200);                                 // 100uS delay (min)

  lcd_write_4(lcd_FunctionReset);                 // third part of reset sequence
  _delay_us(200);                                 // this delay is omitted in the data sheet

  // Preliminary Function Set instruction - used only to set the 4-bit mode.
  // The number of lines or the font cannot be set at this time since the controller is still in the
  //  8-bit mode, but the data transfer mode can be changed since this parameter is determined by one
  //  of the upper four bits of the instruction.

  lcd_write_4(lcd_FunctionSet4bit);               // set 4-bit mode
  _delay_us(80);                                  // 40uS delay (min)

  // Function Set instruction
  lcd_write_instruction_4d(lcd_FunctionSet4bit);   // set mode, lines, and font
  _delay_us(80);                                  // 40uS delay (min)

  // The next three instructions are specified in the data sheet as part of the initialization routine,
  //  so it is a good idea (but probably not necessary) to do them just as specified and then redo them
  //  later if the application requires a different configuration.

  // Display On/Off Control instruction
  lcd_write_instruction_4d(lcd_DisplayOff);        // turn display OFF
  _delay_us(80);                                  // 40uS delay (min)

  // Clear Display instruction
  lcd_write_instruction_4d(lcd_Clear);             // clear display RAM
  _delay_ms(4);                                   // 1.64 mS delay (min)

  // ; Entry Mode Set instruction
  lcd_write_instruction_4d(lcd_EntryMode);         // set desired shift characteristics
  _delay_us(80);                                  // 40uS delay (min)

  // This is the end of the LCD controller initialization as specified in the data sheet, but the display
  //  has been left in the OFF condition.  This is a good time to turn the display back ON.

  // Display On/Off Control instruction
  lcd_write_instruction_4d(lcd_DisplayOn);         // turn the display ON
  _delay_us(80);                                  // 40uS delay (min)
}

/*...........................................................................
  Name:     lcd_write_string_4d
  ; Purpose:  display a string of characters on the LCD
  Entry:    (theString) is the string to be displayed
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_write_string_4d(uint8_t theString[])
{
  volatile int i = 0;                             // character counter*/
  while (theString[i] != 0)
  {
    lcd_write_character_4d(theString[i]);
    i++;
    _delay_us(80);                              // 40 uS delay (min)
  }
}

/*...........................................................................
  Name:     lcd_write_character_4d
  Purpose:  send a byte of information to the LCD data register
  Entry:    (theData) is the information to be sent to the data register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/

void lcd_write_character_4d(uint8_t theData)
{
  lcd_RS_port |= (1 << lcd_RS_bit);               // select the Data Register (RS high)
  lcd_E_port &= ~(1 << lcd_E_bit);                // make sure E is initially low
  lcd_write_4(theData);                           // write the upper 4-bits of the data
  lcd_write_4(theData << 4);                      // write the lower 4-bits of the data
}

/*...........................................................................
  Name:     lcd_write_instruction_4d
  Purpose:  send a byte of information to the LCD instruction register
  Entry:    (theInstruction) is the information to be sent to the instruction register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/
void lcd_write_instruction_4d(uint8_t theInstruction)
{
  lcd_RS_port &= ~(1 << lcd_RS_bit);              // select the Instruction Register (RS low)
  lcd_E_port &= ~(1 << lcd_E_bit);                // make sure E is initially low
  lcd_write_4(theInstruction);                    // write the upper 4-bits of the data
  lcd_write_4(theInstruction << 4);               // write the lower 4-bits of the data
}


/*...........................................................................
  Name:     lcd_write_4
  Purpose:  send a byte of information to the LCD module
  Entry:    (theByte) is the information to be sent to the desired LCD register
            RS is configured for the desired LCD register
            E is low
            RW is low
  Exit:     no parameters
  Notes:    use either time delays or the busy flag
*/
void lcd_write_4(uint8_t theByte)
{
  lcd_D7_port &= ~(1 << lcd_D7_bit);                      // assume that data is '0'
  if (theByte & 1 << 7) lcd_D7_port |= (1 << lcd_D7_bit); // make data = '1' if necessary

  lcd_D6_port &= ~(1 << lcd_D6_bit);                      // repeat for each data bit
  if (theByte & 1 << 6) lcd_D6_port |= (1 << lcd_D6_bit);

  lcd_D5_port &= ~(1 << lcd_D5_bit);
  if (theByte & 1 << 5) lcd_D5_port |= (1 << lcd_D5_bit);

  lcd_D4_port &= ~(1 << lcd_D4_bit);
  if (theByte & 1 << 4) lcd_D4_port |= (1 << lcd_D4_bit);

  // write the data
  // Address set-up time' (40 nS)
  lcd_E_port |= (1 << lcd_E_bit);                 // Enable pin high
  _delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
  lcd_E_port &= ~(1 << lcd_E_bit);                // Enable pin low
  _delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}




// Check button state and set the button_down variable if a debounced
// button down press is detected.
// Call this function about 100 times per second.
static inline void debounce_white_down(void)
{
    // Counter for number of equal states
    static uint8_t count = 0;
    // Keeps track of current (debounced) state
    static uint8_t button_state = 0;

    // Check if button is high or low for the moment
    uint8_t current_state = (~BUTTON_PIN_WH_DWN & BUTTON_MASK_WH_DWN) != 0;
    
    if (current_state != button_state) {
  // Button state is about to be changed, increase counter
  count++;
  if (count >= 4) {
      // The button have not bounced for four checks, change state
      button_state = current_state;
      // If the button was pressed (not released), tell main so
      if (current_state != 0) {
    button_wh_down = 1;
      }
      count = 0;
  }
    } else {
  // Reset counter
  count = 0;
    }
}


// Check button state and set the button_down variable if a debounced
// button down press is detected.
// Call this function about 100 times per second.
static inline void debounce_white_up(void)
{
    // Counter for number of equal states
    static uint8_t count = 0;
    // Keeps track of current (debounced) state
    static uint8_t button_state = 0;

    // Check if button is high or low for the moment
    uint8_t current_state = (~BUTTON_PIN_WH_UP & BUTTON_MASK_WH_UP) != 0;
    
    if (current_state != button_state) {
  // Button state is about to be changed, increase counter
  count++;
  if (count >= 4) {
      // The button have not bounced for four checks, change state
      button_state = current_state;
      // If the button was pressed (not released), tell main so
      if (current_state != 0) {
    button_wh_up = 1;
      }
      count = 0;
  }
    } else {
  // Reset counter
  count = 0;
    }
}


// Check button state and set the button_down variable if a debounced
// button down press is detected.
// Call this function about 100 times per second.
static inline void debounce_yellow_down(void)
{
    // Counter for number of equal states
    static uint8_t count = 0;
    // Keeps track of current (debounced) state
    static uint8_t button_state = 0;

    // Check if button is high or low for the moment
    uint8_t current_state = (~BUTTON_PIN_YL_DWN & BUTTON_MASK_YL_DWN) != 0;
    
    if (current_state != button_state) {
  // Button state is about to be changed, increase counter
  count++;
  if (count >= 4) {
      // The button have not bounced for four checks, change state
      button_state = current_state;
      // If the button was pressed (not released), tell main so
      if (current_state != 0) {
    button_yl_down = 1;
      }
      count = 0;
  }
    } else {
  // Reset counter
  count = 0;
    }
}



// Check button state and set the button_down variable if a debounced
// button down press is detected.
// Call this function about 100 times per second.
static inline void debounce_yello_up(void)
{
    // Counter for number of equal states
    static uint8_t count = 0;
    // Keeps track of current (debounced) state
    static uint8_t button_state = 0;

    // Check if button is high or low for the moment
    uint8_t current_state = (~BUTTON_PIN_YL_UP & BUTTON_MASK_YL_UP) != 0;
    
    if (current_state != button_state) {
  // Button state is about to be changed, increase counter
  count++;
  if (count >= 4) {
      // The button have not bounced for four checks, change state
      button_state = current_state;
      // If the button was pressed (not released), tell main so
      if (current_state != 0) {
    button_yl_up = 1;
      }
      count = 0;
  }
    } else {
  // Reset counter
  count = 0;
    }
}



// Check button state and set the button_down variable if a debounced
// button down press is detected.
// Call this function about 100 times per second.
static inline void debounce_endoscopy(void)
{
    // Counter for number of equal states
    static uint8_t count = 0;
    // Keeps track of current (debounced) state
    static uint8_t button_state = 0;

    // Check if button is high or low for the moment
    uint8_t current_state = (~BUTTON_PIN_END_OS & BUTTON_MASK_END_OS) != 0;
    
    if (current_state != button_state) {
  // Button state is about to be changed, increase counter
  count++;
  if (count >= 4) {
      // The button have not bounced for four checks, change state
      button_state = current_state;
      // If the button was pressed (not released), tell main so
      if (current_state != 0) {
    button_endos = 1;
      }
      count = 0;
  }
    } else {
  // Reset counter
  count = 0;
    }
}



// Check button state and set the button_down variable if a debounced
// button down press is detected.
// Call this function about 100 times per second.
static inline void debounce_power_on(void)
{
    // Counter for number of equal states
    static uint8_t count = 0;
    // Keeps track of current (debounced) state
    static uint8_t button_state = 0;

    // Check if button is high or low for the moment
    uint8_t current_state = (~BUTTON_PIN_ON_OFF & BUTTON_MASK_ON_OFF) != 0;
    
    if (current_state != button_state) {
  // Button state is about to be changed, increase counter
  count++;
  if (count >= 4) {
      // The button have not bounced for four checks, change state
      button_state = current_state;
      // If the button was pressed (not released), tell main so
      if (current_state != 0) {
    button_on_off = 1;
      }
      count = 0;
  }
    } else {
  // Reset counter
  count = 0;
    }
}
