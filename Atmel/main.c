/*
 * 1-WireRoutines.c
 *
 * Created: 01/10/2016 14:30:24 PM
 * Author : A. Uomoto








=====
FUSES
Reading the fuses on a brand-new ATtiny841:
avrdude -c usbtiny -p attiny841 -U lfuse:r:-:h

gives:
lfuse = 0x42
hfuse = 0xdf
efuse = 0xff

I noticed that lfuse in the datasheet (p 219) is 0x62. Bit 5, which is not used,
is programmed (value = 0) on my device but the datasheet says the default is 1.

=====
CLOCK SPEED 8 MHz
The factory default divides the internal clock by 8 resulting in a 1 MHz clock.
Change this to 8 MHz (no clock division) by making lfuse 0xC2:
avrdude -c usbtiny -p attiny841 -U lfuse:w:0xC2:m

so:

lfuse = 0xc2
hfuse = 0xdf
efuse = 0xff

CLOCK SPEED 14.7 MHz
Using a 14.7 MHz crystal, change the CKSEL[3:0] bits (p 26 & p 220) to 111X or
lfuse = 0xce

=====
SPI
Pins are all on PORTA:
SCK		PORTA4
MISO	PORTA5
MOSI	PORTA6
SS		PORTA7

SPCR clock setups using a 14.7456 MHz crystal:
______________________________________________________________
SPI2X | SPR1  | SPR0  | Divisor |SCK (MHz) | SCK period (ns) |
------|-------|-------|---------|----------|-----------------|
  0   |   0   |   0   |    /4   |  3.6864  |       271       |
  0   |   0   |   1   |    /16  |  0.9216  |      1085       |
  0   |   1   |   0   |    /64  |  0.2304  |      4340       |
  0   |   1   |   1   |    /128 |  0.1152  |      8681       |
  1   |   0   |   0   |    /2   |  7.3728  |       136       |
  1   |   0   |   1   |    /8   |  1.8432  |       543       |
  1   |   1   |   0   |    /32  |  0.4608  |      2170       |
  1   |   1   |   1   |    /64  |  0.2304  |      4340       |
_____________________________________________________________|

=====
USART
There are two:
USART0: RXD0 is PA2 on pin 11, TXD0 is PA1 on pin 12
USART1: RXD1 is PA4 on pin 9, TXD1 is PA5 on pin 8

=====
TM1638

The minimum clock pulse width for the TM1638 (PWCLK) is 400 ns (datasheet p17).
We can use the /8 divisor with the 14.7456 MHz crystal for a 543 ns width.

=====
LED
This is wired to PB2

=====
PIN Summary

PA0
PA1, PA2	TX, RX
PA3			DQ (DS18B20)
PA4			SCK
PA5			MISO
PA6			MOSI
PA7			SS
PB0, PB1	XTAL
PB2			LED
PB3			RESET


 */

#define	F_CPU	14745600

#include <avr/io.h>
#include <util/delay.h>

// Serial I/O
#define BAUDRATE	9600
#define MYUBRR		((F_CPU / 16 / BAUDRATE) - 1)
#define TX0READY	(UCSR0A & (1 << UDRE0))	// UDRE0 bit in UCSR0A is set when the TX buffer is clear
#define CHARSENT	(UCSR0A & (1<<RXC0))	// RXC0 bit in UCSR0A is set when a char is available

// SPI pin states
#define SS_LOW		(PORTA &= ~(1 << PORTA7))				// Pull SS low
#define SS_HIGH		(PORTA |= (1 << PORTA7)); _delay_us(1)	// Strobe puls width PWSTB must be >= 1us
#define SPI_WAIT	while (!(SPSR & (1 << SPIF)))			// Wait for transfer to finish

// 1-Wire Commands
#define	SKIPROM				0xCC
#define	CONVERT				0x44
#define	READSCRATCH			0xBE
#define SEARCH				0xF0

// Function Prototypes
uint8_t bitReverse(uint8_t);
uint16_t byteToBCD(uint8_t);
void flashLED(uint8_t);
void initialize(void);
void intToBCD(uint8_t *, uint8_t, uint8_t, uint16_t);
uint8_t owReadBit(void);
uint8_t owReadByte(void);
uint8_t owReset(void);
void owWriteBit(uint8_t);
void owWriteByte(uint8_t);
uint8_t serial0RecvByte(void);
void serial0SendByte(uint8_t);
void tm1638_brightness(uint8_t);
void tm1638_clear(void);
void tm1638_off(void);
void tm1638_putc(uint8_t, uint8_t);
void tm1638_putLED(uint8_t, uint8_t);
void tm1638_putn(uint8_t, uint8_t);
void tm1638_writeTo(uint8_t, uint8_t);

// Globals
const uint8_t number[] =		// Numeral patterns for the 7-segment displays
{	0x3F,	// 0
	0x06,	// 1
	0x5B,	// 2
	0x4F,	// 3
	0x66,	// 4
	0x6D,	// 5
	0x7D,	// 6
	0x07,	// 7
	0x7F,	// 8
	0x6F,	// 9
	0xBF,	// 0. (decimal points) number[10]
	0x86,	// 1.
	0xDB,	// 2.
	0xCF,	// 3.
	0xE6,	// 4.
	0xED,	// 5.
	0xFD,	// 6.
	0x87,	// 7.
	0xFF,	// 8.
	0xEF,	// 9.
	0x40,	// - (minus sign)	number[20]
	0x00	// space			number[21]
};

const uint8_t address_7seg[] =		// TM1638 addresses for the 7-segment displays
{
	0xCE,	// rightmost segment is at [0]
	0xCC,
	0xCA,
	0xC8,
	0xC6,
	0xC4,
	0xC2,
	0xC0	// leftmost segment
};

const uint8_t address_LED[] =		// TM1638 addresses for the LEDs
{
	0xCF,	// rightmost LED is at [0]
	0xCD,
	0xCB,
	0xC9,
	0xC7,
	0xC5,
	0xC3,
	0xC1	// leftmost LED
};

int main(void)
{

	uint8_t charArray[8];
	uint8_t i = 0, b0, b1;
	uint16_t negativeC, negativeF, precision;
	int16_t T;
	float C, F;

	initialize();

	negativeC = negativeF = 0;
	precision = 1;

	for (;;) {
		if (owReset()) {
			flashLED(1);
		} else {
			flashLED(3);
		}
		owWriteByte(SKIPROM);
		owWriteByte(CONVERT);
		_delay_ms(750);
		owReset();
		owWriteByte(SKIPROM);
		owWriteByte(READSCRATCH);
		b0 = owReadByte();
		b1 = owReadByte();
		T = b1;
		T = T << 8;
		T += b0;
		C = (float) T * 0.0625;
		F = ((C * 9.0)/5.0) + 32;
		C = 10.0 * C;
		F = 10.0 * F;
		if (C < 0.0) {
			negativeC = 1;
			C = -C;
		}
		if (F < 0.0) {
			negativeF = 1;
			F = -F;
		}
		intToBCD(charArray, negativeC, precision, (uint16_t) (C + 0.5));
		_delay_ms(50);
		for (i = 0; i < 4; i++) {
			tm1638_putc(i, charArray[i]);
		}
		intToBCD(charArray, negativeF, precision, (uint16_t) (F + 0.5));
		for (i = 0; i < 4; i++) {
			tm1638_putc(i+4, charArray[i]);
		}
	}

}

void flashLED(uint8_t ntimes)
{

	uint8_t i;
	
	for (i = 0; i < ntimes; i++) {
		PORTB |= (1 << PORTB2);
		_delay_ms(5);
		PORTB &= ~(1 << PORTB2);
		_delay_ms(5);
	}

}

void initialize(void)
{

	// Set up serial port 0
	UBRR0H = (uint8_t) (MYUBRR >> 8);			// Set the baud rate
	UBRR0L = (uint8_t) MYUBRR;					// Set the baud rate (second half)
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);		// Enable USART receive and transmit
	UCSR0C = (3 << UCSZ00);						// 8 bits, no parity, one stop bit

	// Set up SPI (MISO is an input port so DDRA is not changed)
	DDRA |= (1 << DDA4);						// SCK output
	DDRA |= (1 << DDA6);						// MOSI output
	DDRA |= (1 << DDA7);						// SS output

	// Set SS high before enabling SPI to de-select the TM1638
	PORTA |= (1 << PORTA7);

	// Enable SPI as master, set clock rate to fclk/8 for a 543ns clock period (1.8432 MHz)
	SPCR |= (1 << SPE);							// Enable SPI
	SPCR |= (1 << MSTR);						// SPI Master mode
	SPCR |= (1 << SPR0);						// SPR0 and SPI2X bits tp make Fclk/8
	SPCR |= (1 << SPI2X);						// Fclk/8
	SPCR |= (1 << DORD);						// Make LSB sent first (for TM1638)
	SPCR |= (1 << CPOL);						// Clock polarity, SCK is high when idle (for TM1638)
	SPCR |= (1 << CPHA);						// Clock phase, sample on trailing edge (for TM1683)

	// Initial display setup
	tm1638_clear();								// Clear all tm1638 registers
	tm1638_brightness(0);						// Set initial brightness to minimum

	// 1-Wire pin is PA3. Leave it as an input on intialization.

	// Set up LED on the ATtiny841 Basic board
	DDRB |= (1 << PORTB2);						// LED is on PORTB2
	flashLED(3);

	serial0SendByte('>');

}

uint8_t serial0RecvByte(void)
{

	while (!CHARSENT) {
		asm("nop");
	}
	return(UDR0);

}

void serial0SendByte(uint8_t c)
{

	while (!TX0READY) {
		asm("nop");
	}
	UDR0 = c;

}

/*-----------------------------------------------------------------------------

tm1638_brightness(brightness)

	Sets the brightness level on a scale of brightness = 0 to 7.
	0 is not turned off, it's just the lowest brightness.
	On the board I'm using now, brightness levels above 4 are all the same.

-----------------------------------------------------------------------------*/
void tm1638_brightness(uint8_t brightness)
{

	SS_LOW;
	SPDR = 0x88 + (0x07 & brightness);
	SPI_WAIT;
	SS_HIGH;

}

/*-----------------------------------------------------------------------------

tm1638_clear()

	Writes a blank to all 16 grid locations (all numerals and LEDs).

-----------------------------------------------------------------------------*/
void tm1638_clear(void)
{

	uint8_t i;

	// Command to write to incrementing addresses
	SS_LOW;
	SPDR = 0x40;					// Write to incrementing addresses
	SPI_WAIT;
	SS_HIGH;

	// Send the starting address 0xC0 (first grid position), don't set SS_HIGH after this
	SS_LOW;
	SPDR = 0xC0;					// Address
	SPI_WAIT;

	// Send zeros for data
	for (i = 0; i < 16; i++) {
		SPDR = 0x00;
		SPI_WAIT;
	}

	SS_HIGH;

}

/*-----------------------------------------------------------------------------

tm1638_off()

	Turns off the display but doesn't erase the registers.

-----------------------------------------------------------------------------*/
void tm1638_off(void)
{

		SS_LOW;
		SPDR = 0x80;		// Display off command
		SPI_WAIT;
		SS_HIGH;

}

void tm1638_putc(uint8_t location, uint8_t displayCode)
{

	uint8_t address;

	address = address_7seg[location];
	tm1638_writeTo(address, displayCode);


}

/* tm1638_putLED(location, color)----------------------------------------------------

	Light the LED in position "location."
	Color is 0 for off, 1 for red, 2 for green. 3 lights both but it just looks red.

	location selects the LED.
	location = 7 is the leftmost LED.
-----------------------------------------------------------------------------*/
void tm1638_putLED(uint8_t location, uint8_t color)
{

	uint8_t data, address;

	data = color;
	address = address_LED[location];
	tm1638_writeTo(address, data);

}

/* tm1638_putn(location, n)----------------------------------------------------

	Put the numeral representing "n" onto the 7-segment digit at "location."
	n is an 8-bit unsigned integer indicating the digit (0-9).
	For 0 <= n <= 9 the numeral displayed is the corresponding number.
	For 10 <= n <= 19 the numeral (from 0-9) contains the decimal point.
	n = 20 is the minus sign.
	n = 21 is a space.

	location selects the 7-segment display.
	location = 7 is the leftmost 7-segment display.
-----------------------------------------------------------------------------*/
void tm1638_putn(uint8_t location, uint8_t n)
{

	uint8_t data, address;

	data = number[n];
	address = address_7seg[location];
	tm1638_writeTo(address, data);

}

/*-----------------------------------------------------------------------------

tm1638_writeTo(address, data)

	Write data (light pattern) to a specific address.

-----------------------------------------------------------------------------*/
void tm1638_writeTo(uint8_t address, uint8_t data)
{

	SS_LOW;				// Start command.
	SPDR = 0x44;		// Command to write to a fixed address.
	SPI_WAIT;
	SS_HIGH;			// End command

	SS_LOW;				// Start address & data transfer.
	SPDR = address;		// Send address.
	SPI_WAIT;
	SPDR = data;		// Send data.
	SPI_WAIT;
	SS_HIGH;			// End data transfer

}

/*-----------------------------------------------------------------------------

uint32_t decToBCD(*charArray, negative, precision, n)

	Fills charArray with the 7-segment display codes representing the 16-bit
	number n. If you want a minus sign, make "negative" non-zero. The value n
	is always a positive integer so negative values need to be abs()'d first.
	Precision is the decimal point location. If precision is zero, no decimal
	point will be displayed.
	
	charArray is assumed to be 8 bytes long although the maximum number of
	digits expected from the uint16_t n is five.

-----------------------------------------------------------------------------*/
void intToBCD(uint8_t * charArray, uint8_t negative, uint8_t precision, uint16_t n)
{

	uint8_t i, firstNonZeroDigit, shift, index;
	uint32_t bcd = 0;

	shift = 0;
	while (n) {										// Convert n to BCD
		bcd += ((uint32_t) (n % 10)) << shift;		// Note the cast required for AVR
		n /= 10;
		shift += 4;
	}

	for (i = 0; i < 8; i++) {						// Fill the char array with the 7-segment codes
		index = (uint8_t) (bcd & 0x0F);
		if ((precision) && (precision == i)) {
			index += 10;
		}
		charArray[i] = number[index];
		bcd = bcd >> 4;
	}

	firstNonZeroDigit = 0;
	for (i = 7; i > 0; i--) {						// Replace leading zeros with spaces
		if (charArray[i] == number[0]) {
			charArray[i] = number[21];
		} else {
			firstNonZeroDigit = i;
			break;
		}
	}

	if (negative) {
		charArray[firstNonZeroDigit+1] = number[20];
	}

}

uint8_t owReset()
{

	uint8_t i;

	PORTA |= (1 << PORTA3);
	DDRA |= (1 << PORTA3);			// Set DQ port as output, value high
	PORTA &= ~(1 << PORTA3);		// DQ low
	_delay_us(480);					// Wait 480us (minimum wait time)
	PORTA |= (1 << PORTA3);			// DQ High
	DDRA &= ~(1 << PORTA3);			// Set as input
	_delay_us(85);					// This is a good number for the DS18B20s I have
	i = !(PINA & (1 << PINA3));
	_delay_us(395);					// Readback time must be 480us minimum
	return (i);

}

/*-----------------------------------------------------------------------------

void owWriteBit(bit)
	Sends the bit value.

-----------------------------------------------------------------------------*/
void owWriteBit(uint8_t bit)
{

	PORTA |= (1 << PORTA3);
	DDRA |= (1 << PORTA3);			// Set DQ port as output, value high
	PORTA &= ~(1 << PORTA3);		// DQ low
	if (bit) {
		_delay_us(14);				// Write 1 pulse
		PORTA |= (1 << PORTA3);		// DQ must go high before 15us passes
		DDRA &= ~(1 << PORTA3);		// Set DQ as input
		_delay_us(45);				// Time slots must be >= 60us
	} else {
		_delay_us(60);				// Write 0, must stay low for 60us
		PORTA |= (1 << PORTA3);		// DQ high
		DDRA &= ~(1 << PORTA3);		// Set DQ as input
	}
	_delay_us(1);					// Time between bit writes must be 1us or more

}

/*-----------------------------------------------------------------------------

void owReadBit()
	Reads the bit value.

-----------------------------------------------------------------------------*/
uint8_t owReadBit()
{

	uint8_t bit;

	PORTA |= (1 << PORTA3);					// Set DQ port high
	DDRA |= (1 << PORTA3);					// Set DQ port as output, value high
	PORTA &= ~(1 << PORTA3);				// Pull DQ low for 1us
	_delay_us(1);							// Short 1us pulse to start a read
	PORTA |= (1 << PORTA3);					// DQ high
	DDRA &= ~(1 << PORTA3);					// Set DQ port as input
	_delay_us(13);							// Wait towards end of the window before reading the wire
	bit = (PINA & (1 << PINA3)) ? 1 : 0;	// Read the bit
	_delay_us(47);							// Time slot must be 60us plus 1us before next time slot start
	return(bit);

}

void owWriteByte(uint8_t byte)
{

	uint8_t i;

	for (i = 0; i < 8; i++) {
		owWriteBit(byte & 0x01);
		byte = byte >> 1;
	}
}

uint8_t owReadByte()
{

	uint8_t i, n = 0;

	n = owReadBit();
	for (i = 1; i < 8; i++) {
		n = n << 1;
		n += owReadBit();
	}
	return(bitReverse(n));

}

uint8_t bitReverse(uint8_t x)
{
	
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return(x);

}