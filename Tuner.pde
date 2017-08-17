/*

Arduino chromatic guitar tuner.
2008 Frederic Hamel

As a modified version of the Arduino core library, the LiquidCrystal class is licensed under the LGPL.
See attached license.

The rest is released under the Attribution-NonCommercial 3.0 Unported Creative Commons license:
http://creativecommons.org/licenses/by-nc/3.0/

See the project writeup:
http://deambulatorymatrix.blogspot.com/2010/11/digital-chromatic-guitar-tuner-2008.html.

Note that this code was more or less hacked together in a marathon along with the building of the actual
device; it should *definitely* not be considered an example of production-quality code.

For all pitch detection-related code, see the YIN paper:
de Cheveigne, Alain and Kawahara, Hideki.  "YIN, a fundamental frequency estimator for speech and music", Journal
of the Acoustical Society of America, Vol 111(4), pp. 1917-30, April 2002.
http://recherche.ircam.fr/equipes/pcm/cheveign/pss/2002_JASA_YIN.pdf

 */

#include <avr/pgmspace.h>
#include <EEPROM.h>

#define ASSERT(x) \
	if (!x) \
	{ \
		while (1) \
		{ \
			Blink(); \
		} \
	}

// from wiring_private.h
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

// from pins_arduino.h
#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER2  5
#define TIMER2A 6
#define TIMER2B 7
//extern const uint8_t PROGMEM port_to_input_PGM[];
extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
//#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_byte( port_to_input_PGM + (P))) )

#define ENABLE_PRINT 1
#define ENABLE_LCD 1

static int const kStatusPin = 13;
static int const kStatusPin2 = 12;
static int const kPitchDownButtonPin = 9;
static int const kPitchUpButtonPin = 10;
static int const kModeButtonPin = 11;

#include <inttypes.h>
#include "Print.h"

///////////////////////////////////////////////////////////////////////////////
// LCD code.  This began as a straight copy of the Arduino LiquidCrystal library,
// and it was tweaked afterwards for the MTC-S16205DFYHSAY.
// Timings *should* still work for the HD44780, but this particular model
// requires a bit of extra "convincing" on initialisation.
///////////////////////////////////////////////////////////////////////////////

static int const CHARACTER_WIDTH = 5;
static int const CHARACTER_HEIGHT = 7;
static int const DISPLAY_WIDTH = 16;
static int const MAX_TICK = CHARACTER_WIDTH * (DISPLAY_WIDTH - 1);
typedef char Glyph[CHARACTER_HEIGHT];
typedef int WideGlyph[CHARACTER_HEIGHT];

Glyph g_tickGlyph =
{
	0b00011111,
	0b00011111,
	0b00001110,
	0b00001110,
	0b00000100,
	0b00000100,
	0b00000100
};
static int const TICK_GLYPH = CHARACTER_WIDTH;
static int const NOTE_GLYPH = TICK_GLYPH + 1; // wide

class LiquidCrystal : public Print {
public:
  LiquidCrystal(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
  LiquidCrystal(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
    uint8_t, uint8_t, uint8_t, uint8_t);
  void clear();
  void home();
  void setCursor(int, int); 
  virtual size_t write(uint8_t);

  void setCharacterGlyph(uint8_t character, Glyph g);
  void setWideCharacterGlyph(uint8_t character, WideGlyph g);
private:
  void send(uint8_t, uint8_t);
  void command(uint8_t);
  
  uint8_t _four_bit_mode;
  uint8_t _rs_pin; // LOW: command.  HIGH: character.
  uint8_t _rw_pin; // LOW: write to LCD.  HIGH: read from LCD.
  uint8_t _enable_pin; // activated by a HIGH pulse.
  uint8_t _data_pins[8];
};

LiquidCrystal::LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
  uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) :
  _four_bit_mode(1), _rs_pin(rs), _rw_pin(rw), _enable_pin(enable)
{
  _data_pins[0] = d0;
  _data_pins[1] = d1;
  _data_pins[2] = d2;
  _data_pins[3] = d3; 
  
  pinMode(_rs_pin, OUTPUT);
  pinMode(_rw_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);
  
  for (int i = 0; i < 4; i++)
    pinMode(_data_pins[i], OUTPUT);
 
  delay(250);
  command(0x28);  // function set: 4 bits, 1 line, 5x8 dots @fhamel: 2 lines for MTC-S16205DFYHSAY?
  delay(15);
  command(0x28);  // (again)
  delay(15);
  command(0x28);  // (again)
  delay(15);
  command(0x28);  // (again)
  delay(15);
  command(0x0C);  // display control: turn display on, cursor off, no blinking
  delay(15);
  command(0x0C);  // (again)
  delay(15);
  command(0x0C);  // (again)
  delay(15);
  command(0x0C);  // (again)
  delay(15);
  command(0x06);  // entry mode set: increment automatically, display shift, right shift
  delay(15);
  command(0x06);  // (again)
  delay(15);
  command(0x06);  // (again)
  delay(15);
  command(0x06);  // (again)
  delay(15);
  clear();
  delay(15);
}

void LiquidCrystal::clear()
{
  command(0x01);  // clear display, set cursor position to zero
  delayMicroseconds(2000);
}

void LiquidCrystal::home()
{
  command(0x02);  // set cursor position to zero
  delayMicroseconds(2000);
}

// Moves the cursor around on the LCD.
void LiquidCrystal::setCursor(int col, int row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  command(0x80 | (col + row_offsets[row]));
}

// Customizes a single character glyph in the LCD's CGRAM.
void LiquidCrystal::setCharacterGlyph(uint8_t characterIndex, Glyph g)
{
	// Index into the CGRAM, which is indexed with 3 MSB indicating glyph number and 3 LSB indicating row number.
	// However, there is no need to reset the address every time, since the entry mode's advance bit applies here
	// as well (i.e. the address autoincrements after each access).
	command(0x40 | (characterIndex << 3));
	for (int row = 0; row < CHARACTER_HEIGHT; ++row)
	{
		//command(0x40 | (characterIndex << 3) | row);
		write(g[row]);
	}
	write(0);
}

// Customizes a set pair of custom character glyps in the LCD's CGRAM; used to render large note names which are
// easier to read from afar.
void LiquidCrystal::setWideCharacterGlyph(uint8_t characterIndex, WideGlyph g)
{
	command(0x40 | (characterIndex << 3));
	for (int row = 0; row < CHARACTER_HEIGHT; ++row)
	{
		write(char((g[row] >> 5) & 0x1F));
	}
	write(0);
	command(0x40 | ((characterIndex + 1) << 3));
	for (int row = 0; row < CHARACTER_HEIGHT; ++row)
	{
		write(char(g[row] & 0x1F));
	}
	write(0);
}

void LiquidCrystal::command(uint8_t value) {
  send(value, LOW);
}

size_t LiquidCrystal::write(uint8_t value) {
  send(value, HIGH);
  return 1;
}

void LiquidCrystal::send(uint8_t value, uint8_t mode) {
  digitalWrite(_rs_pin, mode);
  digitalWrite(_rw_pin, LOW);

  if (_four_bit_mode) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(_data_pins[i], (value >> (i + 4)) & 0x01);
    }
    
    digitalWrite(_enable_pin, HIGH);
    digitalWrite(_enable_pin, LOW);
    
    for (int i = 0; i < 4; i++) {
      digitalWrite(_data_pins[i], (value >> i) & 0x01);
    }

    digitalWrite(_enable_pin, HIGH);
    digitalWrite(_enable_pin, LOW);
  } else {
    for (int i = 0; i < 8; i++) {
      digitalWrite(_data_pins[i], (value >> i) & 0x01);
    }

    digitalWrite(_enable_pin, HIGH);
    digitalWrite(_enable_pin, LOW);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Utility stuff
///////////////////////////////////////////////////////////////////////////////

// From http://www.arduino.cc/playground/Code/AvailableMemory
// This function will return the number of bytes currently free in RAM.
// written by David A. Mellis
// based on code by Rob Faludi http://www.faludi.com
// (reformatted for clarity)
// Note: I believe this does not work on recent versions of the Arduino environment.
// See the webpage above for alternatives.
int AvailableMemory()
{
	int size = 1024;
	byte* buf;

	while ((buf = (byte *) malloc(--size)) == NULL)
		;

	free(buf);

	return size;
}

#if ENABLE_PRINT
LiquidCrystal* g_lcd;
#define DEFAULT_PRINT g_lcd
void Ln(Print* p = DEFAULT_PRINT)
{
	p->println("");
}

void Space(Print* p = DEFAULT_PRINT)
{
	p->print(" ");
}

void Cls(Print* p = DEFAULT_PRINT)
{
	p->print(char(27));
	p->print("[2J");
}

void PrintFloat(float f, int decimals = 5, Print* p = DEFAULT_PRINT)
{
	if (f < 0)
	{
		p->print("-");
		f = -f;
	}
	else
	{
		p->print(" ");
	}
	
	int b = int(f);
	p->print(b);
	p->print(".");
	f -= b;
	for (int i = 0; i < decimals; ++i)
	{
		f *= 10.0f;
		int a = int(f);
		p->print(a);
		f -= a;
	}
}

void PrintHex(int h, Print* p = DEFAULT_PRINT)
{
	static char const* hex = "0123456789ABCDEF";
	p->print(hex[(h & 0xF000) >> 12]);
	p->print(hex[(h & 0x0F00) >> 8]);
	p->print(hex[(h & 0x00F0) >> 4]);
	p->print(hex[(h & 0x000F) >> 0]);
}

void PrintStringInt(char const* s, int v, Print* p = DEFAULT_PRINT)
{
	p->print(s);
	p->print(": ");
	p->print(v);
}

void PrintStringLong(char const* s, long v, Print* p = DEFAULT_PRINT)
{
	p->print(s);
	p->print(": ");
	p->print(v);
}

void PrintStringFloat(char const* s, float f, int decimals = 5, Print* p = DEFAULT_PRINT)
{
	p->print(s);
	p->print(": ");
	PrintFloat(f, decimals);
}

#endif //ENABLE_PRINT

const int kDebounceMs = 50;

void Blink(int times = 1)
{
	for (int i = 0; i < times; ++i)
	{
		digitalWrite(kStatusPin, HIGH);
		delay(kDebounceMs);
		digitalWrite(kStatusPin, LOW);
		delay(kDebounceMs);
	}
}

class PushButton
{
public:
	PushButton(int pin, bool activeLow = true)
		: m_pin(pin)
		, m_debounceStart(0)
		, m_activeLow(activeLow)
	{
		pinMode(m_pin, INPUT);
	}
	
	void Update()
	{
		m_justPressed = false;
		m_justReleased = false;

		if (millis() < m_debounceStart + kDebounceMs)
		{
			return;
		}
		
		int buttonValue = digitalRead(m_pin);
		int const ACTIVE = m_activeLow ? 0 : 1;
		int const INACTIVE = 1 - ACTIVE;
		if ((m_lastValue == INACTIVE) && (buttonValue == ACTIVE))
		{
			m_debounceStart = millis();
			m_justPressed = true;
		}
		else if ((m_lastValue == ACTIVE) && (buttonValue == INACTIVE))
		{
			m_debounceStart = millis();
			m_justReleased = true;
		}
		m_lastValue = buttonValue;
	}
	
	bool IsPressed() { return m_lastValue == (m_activeLow ? 0 : 1); }
	bool JustPressed() { return m_justPressed; }
	bool JustReleased() { return m_justReleased; }

	void WaitForPress()
	{
		for (; !JustPressed(); Update());
		Update();
	}
	
private:
	int m_pin;
	int m_lastValue;
	long m_debounceStart;
	bool m_activeLow;
	bool m_justPressed;
	bool m_justReleased;
};

PushButton g_pitchDownButton(kPitchDownButtonPin);
PushButton g_pitchUpButton(kPitchUpButtonPin);
PushButton g_modeButton(kModeButtonPin);

///////////////////////////////////////////////////////////////////////////////
// The Tuner
///////////////////////////////////////////////////////////////////////////////
// Note Freq    Cycles
// A	110		145454.55
// A#	116.54	137290.81
// B	123.47	129585.27
// C	130.81	122312.21
// C#	138.59	115447.35
// D	146.83	108967.79
// D#	155.56	102851.9
// E	164.81	97079.26
// F	174.61	91630.62
// F#	185		86487.79
// G	196		81633.6
// G#	207.65	77051.86
// A	220		72727.27

static int const NUM_NOTES = 13;
static char* g_noteNames[NUM_NOTES] =
{
	"A ",
	"A#",
	"B ",
	"C ",
	"C#",
	"D ",
	"D#",
	"E ",
	"F ",
	"F#",
	"G ",
	"G#",
	"A "
};

int g_noteGlyphIndex[NUM_NOTES] =
{
	0,
	0,
	1,
	2,
	2,
	3,
	3,
	4,
	5,
	5,
	6,
	6,
	0
};

bool g_noteSharpSign[NUM_NOTES] =
{
	false,
	true,
	false,
	false,
	true,
	false,
	true,
	false,
	false,
	true,
	false,
	true,
	false
};

WideGlyph g_noteGlyphs[7] =
{
	{//   9876543210
		0b0000110000,
		0b0011111100,
		0b0110000110,
		0b0111111110,
		0b0111111110,
		0b0110000110,
		0b0110000110
	},
	{
		0b0111111000,
		0b0111111100,
		0b0110001100,
		0b0111111110,
		0b0110000110,
		0b0111111110,
		0b0111111110
	},
	{
		0b0011111100,
		0b0111111110,
		0b0110000110,
		0b0110000000,
		0b0110000110,
		0b0111111110,
		0b0011111100
	},
	{
		0b0111111000,
		0b0111111100,
		0b0110000110,
		0b0110000110,
		0b0110000110,
		0b0111111100,
		0b0111111000
	},
	{
		0b0111111110,
		0b0111111110,
		0b0110000000,
		0b0111111000,
		0b0110000000,
		0b0111111110,
		0b0111111110
	},
	{
		0b0111111110,
		0b0111111110,
		0b0110000000,
		0b0111111000,
		0b0110000000,
		0b0110000000,
		0b0110000000
	},
	{
		0b0011111100,
		0b0111111110,
		0b0110000000,
		0b0110001110,
		0b0110000110,
		0b0111111110,
		0b0011111110
	}
};

static int const A440_NOTE = 69;
static int const A880_NOTE = A440_NOTE + 12;
float const QUARTERTONE_UP = 1.0293022366f;
float const QUARTERTONE_DOWN = 0.9715319412f;
float const SEMITONE_UP = 1.0594630944f;
float const SEMITONE_DOWN = 0.9438743127f;

typedef int Fixed;
static int const FIXED_SHIFT = 5;
static int const FIXED_ONE = (1 << FIXED_SHIFT);
static int const FRAC_MASK = (1 << FIXED_SHIFT) - 1;
static int const FIXED_SHIFT_HI = 8 - FIXED_SHIFT;
static int const FIXED_MASK_HI = ~((1 << FIXED_SHIFT_HI) - 1);
#define FIXED_INT(x) int(x >> FIXED_SHIFT)
#define FIXED_FRAC(x) int(x & FRAC_MASK)
#define		I2FIXED(x) ((Fixed) ((x) << FIXED_SHIFT))
#define		F2FIXED(x) ((Fixed) ((x) * (1 << FIXED_SHIFT)))
#define		FIXED2I(x) ((x) >> FIXED_SHIFT)
#define		FIXED2F(x) ((x) / float(1 << FIXED_SHIFT))

// Useful to print compile-time integer values, since instanciating a template with this incomplete type will
// cause the compiler to include the value of N in the error message.  (On avr-gcc, this seems to only work
// with free-standing variables, not members.)
template<int N> struct PrintInt;

// Linearly interpolates between two 8-bit signed values.
char InterpolateChar(char a, char b, char tFrac)
{
	int d = b - a;
	return a + ((d * tFrac) >> FIXED_SHIFT);
}

namespace TunerMode
{
	enum Type
	{
		Tuner = 0,
		Midi,
		Max
	};
}

#define ALWAYSPRINT(s, v) { m_lcd.clear(); m_lcd.print(s); m_lcd.setCursor(0, 1); m_lcd.print(v); delay(1800); }
#define DEBUGPRINT(s, v) if (debug) { m_lcd.clear(); m_lcd.print(s); m_lcd.setCursor(0, 1); m_lcd.print(v); delay(1800); }
class Tuner
{
public:
	Tuner(int audioPin)
		: m_audioPin(audioPin)
#if ENABLE_LCD
		, m_lcd(2, 3, 4, 5, 6, 7, 8)
#endif
		, m_midiNote(-1)
		, m_a440(440.0f)
		, m_mode(TunerMode::Tuner)
	{
#if ENABLE_LCD
		g_lcd = &m_lcd;
#endif
	}

	~Tuner()
	{
#if ENABLE_LCD
		g_lcd = NULL;
#endif
	}

	static int const PRESCALER = 0b00000111;
	static int const PRESCALER_DIVIDE = (1 << PRESCALER);
	static int const ADC_CLOCKS_PER_ADC_CONVERSION = 13;
	static unsigned long const CPU_CYCLES_PER_SAMPLE = ADC_CLOCKS_PER_ADC_CONVERSION * PRESCALER_DIVIDE;
	static unsigned long const SAMPLES_PER_SECOND = F_CPU / CPU_CYCLES_PER_SAMPLE;

	static int const MIN_FREQUENCY = 60;
	static int const MAX_FREQUENCY = 880;
	static int const MIN_SAMPLES = SAMPLES_PER_SECOND / MAX_FREQUENCY;
	static int const MAX_SAMPLES = SAMPLES_PER_SECOND / MIN_FREQUENCY;
	static int const WINDOW_SIZE = MAX_SAMPLES; //96; // samples
	static int const BUFFER_SIZE = WINDOW_SIZE + MAX_SAMPLES + 1; // for interpolation
	static int const AMPLITUDE_THRESHOLD = 30;
	static int const CORRELLATION_STEP = 2;
	// For pure sine, 8 is better than 10 or 12, which causes octave errors at higher frequencies
	// (>400 for 10, >350 for 12).
	// For guitar, sweet spot is between 1/2 and 1/3
	static int const PRIME_THRESHOLD = (FIXED_ONE * 5) / 12;
	static int const OFFSET_STEP = 1;
	
	void Start()
	{
#if ENABLE_LCD
		m_lcd.clear();
		m_lcd.print("Hello world");
		delay(2000);
		m_lcd.clear();
		for (int i = 0; i < CHARACTER_WIDTH; ++i)
		{
			Glyph g;
			for (int j = 0; j < CHARACTER_HEIGHT; ++j)
			{
				g[j] = (1 << (CHARACTER_WIDTH - 1 - i));
			}
			m_lcd.setCharacterGlyph(i, g);
		}
		m_lcd.setCharacterGlyph(TICK_GLYPH, g_tickGlyph);
#endif

		// Enable auto-trigger enable
		sbi(ADCSRA, ADATE);
		// Set auto-trigger to free-running mode
		cbi(ADCSRB, ADTS0);
		cbi(ADCSRB, ADTS1);
		cbi(ADCSRB, ADTS2);

		// Select input channel + set reference to Vcc
		ADMUX = /*(0 << 6) |*/ (m_audioPin & 0x0f);
		ADMUX = (1 << 6) | (m_audioPin & 0x0f);

		// Disable other ADC channels (try to reduce noise?)
		DIDR0 = (0x3F ^ (1 << m_audioPin));

		// Left-adjust result so we only have to read 8 bits
		sbi(ADMUX, ADLAR); // right-adjust for 8 bits

		// Setup the prescaler; divide by 32
		unsigned char adcsra = ADCSRA;
		adcsra = ((adcsra & 0xF8) | PRESCALER); // mask off / re-set prescaler bits
		ADCSRA = adcsra;

		// Disable the conversion complete interrupt so we can read the flag
		cbi(ADCSRA, ADIE); // Interrupt Enable

		// Start the shebang
		sbi(ADCSRA, ADSC);

#if ENABLE_LCD
		static int const MIN_MEMORY = 89;
		if (AvailableMemory() < MIN_MEMORY)
		{
			for(;;)
			{
				m_lcd.clear();
				m_lcd.print("OOM");
				m_lcd.setCursor(0, 1);
				m_lcd.print(AvailableMemory() - MIN_MEMORY);
				delay(2000);
			}
		}
#endif

		LoadTuning();
		TunePitch();
	}

	void Stop()
	{
		// Disable auto-trigger mode
		cbi(ADCSRA, ADATE);

		// Right-adjust result (default)
		cbi(ADMUX, ADLAR);
	}

	unsigned long LoadEepromLong(int address)
	{
		unsigned long result = 0;
		unsigned long t = 0; // to avoid spurious warning :/ 
		result |= EEPROM.read(address);
		t = EEPROM.read(address + 1);
		result |= (t << 8);
		t = EEPROM.read(address + 2);
		result |= (t << 16);
		t = EEPROM.read(address + 3);
		result |= (t << 24);
		return result;
	}

	void SaveEepromLong(int address, unsigned long i)
	{
		EEPROM.write(address, i & 0xFF);
		EEPROM.write(address + 1, (i >> 8) & 0xFF);
		EEPROM.write(address + 2, (i >> 16) & 0xFF);
		EEPROM.write(address + 3, (i >> 24) & 0xFF);
	}

	void SaveTuning()
	{
		SaveEepromLong(0, *((unsigned long*) &m_a440));
	}

	void LoadTuning()
	{
		*((unsigned long*) &m_a440) = LoadEepromLong(0);
	}

	// The following two functions require proper setup in Start()
	unsigned int ReadInput8BitsUnsigned()
	{
		while ((ADCSRA & _BV(ADIF)) == 0)
		{
		}
		unsigned int result = ADCH;
		sbi(ADCSRA, ADIF);
		return result;
	}
	
	int ReadInput8BitsSigned()
	{
		return ReadInput8BitsUnsigned() - 128;
	}
	
	// Given a MIDI note index, returns the corresponding index into the global string array of note names.
	int GetNoteNameIndex(int note)
	{
		return (note + 3) % 12;
	}

	// Given a MIDI note index, get the name of the note as ASCII.
	char* GetNoteName(int note)
	{
		if (note >= 0)
		{
			return g_noteNames[GetNoteNameIndex(note)];
		}
		else
		{
			return "  ";
		}
	}

	// Converts a fundamental frequency in Hz to a MIDI note index.  Slow.
	int GetMidiNoteIndexForFrequency(float frequency)
	{
		if (frequency < 0.0f)
		{
			return -1;
		}

		frequency *= QUARTERTONE_DOWN; // kind of like rounding the note, half a semitone (it's a multiplication because of logarithmic space)

		int note = A440_NOTE;
		float a440 = m_a440;
		float a440_2 = 2.0f * a440;
		while (frequency < a440)
		{
			frequency *= 2.0f;
			note -= 12;
		}
		while (frequency > a440_2)
		{
			frequency *= 0.5f;
			note += 12;
		}
		while (frequency > a440)
		{
			frequency *= SEMITONE_DOWN;
			note += 1;
		}
		return note;
	}

	// Compute the fundamental frequency of a given MIDI note index.  Slow.
	float GetFrequencyForMidiNoteIndex(int note)
	{
		if (note < 0.0f)
		{
			return -1.0f;
		}

		float result = m_a440;
		
		while (note < A440_NOTE)
		{
			note += 12;
			result *= 0.5f;
		}
		
		while (note > A880_NOTE)
		{
			note -= 12;
			result *= 2.0f;
		}

		while (note > A440_NOTE)
		{
			--note;
			result *= SEMITONE_UP;
		}

		return result;
	}

	unsigned long GetCorrellationFactorFixed(char* buffer, Fixed fixedOffset)
	{
		unsigned long result = 0;
		int integer = FIXED_INT(fixedOffset);
		int frac = FIXED_FRAC(fixedOffset);
		int correllationStep = CORRELLATION_STEP;

		// If we're in MIDI mode, lower the precision to gain speed.
		if (m_mode == TunerMode::Midi)
		{
			correllationStep <<= 1;
		}

		for (int i = 0; i < WINDOW_SIZE; i += correllationStep)
		{
			// Note this is done with 16-bit math; this is slower, but gives more precision.  In tests, using 8-bit
			// math did not yield sufficient precision.
			int a = buffer[i];
			int b = InterpolateChar(buffer[i + integer], buffer[i + integer + 1], frac);
			result += abs(b - a);			
		}
		return result;
	}

	unsigned long GetCorrellationFactorPrime(unsigned long currentCorrellation, int numToDate, unsigned long sumToDate)
	{
		if (numToDate == 0)
		{
			return FIXED_ONE;
		}
		else
		{
			return ((currentCorrellation << FIXED_SHIFT) * numToDate) / sumToDate;
		}
	}

	// Compute the frequency corresponding to a given a fixed-point offset into our sampling buffer (usually where
	// the best/minimal autocorrellation was achieved).
	float GetFrequencyForOffsetFixed(Fixed offset)
	{
		float floatOffset = FIXED2F(offset);
		return F_CPU / (floatOffset * CPU_CYCLES_PER_SAMPLE);
	}

	float DetermineSignalPitch()
	{
		// Sample the signal into our buffer, and track its amplitude.
		m_maxAmplitude = 0;
		for (int i = 0; i < BUFFER_SIZE; ++i)
		{
			m_buffer[i] = ReadInput8BitsSigned();
			if (abs(m_buffer[i]) > m_maxAmplitude)
			{
				m_maxAmplitude = abs(m_buffer[i]);
			}
		}

		float result = 0.0f;

		// If we haven't reached the amplitude threshold, don't try to determine pitch.
		if (m_maxAmplitude < AMPLITUDE_THRESHOLD)
		{
			result = -1.0f;
		}

		// Alright, now try to figure what the ballpark note this is by calculating autocorrellation
		Fixed bestOffset = 0;
		Fixed incrementAtBestOffset = 0;

		if (result >= 0.0f)
		{
			int numCorrellations = 0;
			unsigned long sumToDate = 0;
			bool inThreshold = false;		
			unsigned long bestPrime = ~0;
			Fixed offsetStep = OFFSET_STEP;
			
			// If we're in MIDI mode, double the step; this will reduce precision, but increase speed (and thus reduce
			// latency).
			if (m_mode == TunerMode::Midi)
			{
				offsetStep <<= 1;
			}
			
			Fixed offsetIncrement = offsetStep;
			for (Fixed offset = I2FIXED(MIN_SAMPLES); FIXED_INT(offset) < MAX_SAMPLES; offset += offsetIncrement)
			{
				unsigned long curCorrellation = GetCorrellationFactorFixed(m_buffer, offset);
				++numCorrellations;
				sumToDate += curCorrellation;

				// Increment the increment right away, so we save the range appropriately for the refined search
				offsetIncrement += offsetStep;

				unsigned long prime = GetCorrellationFactorPrime(curCorrellation, numCorrellations, sumToDate);
				if (prime < bestPrime)
				{
					bestPrime = prime;
					bestOffset = offset;
					incrementAtBestOffset = offsetIncrement;
				}

				if (prime < PRIME_THRESHOLD)
				{
					inThreshold = true;
				}
				else if (inThreshold) // was in threshold, now exited, have best minimum in threshold
				{
					//found = true;
					break;
				}
			}
		}

		if (FIXED_INT(bestOffset) == MAX_SAMPLES)
		{
			result = -1.0f;
		}

#define SUBDIVIDE 1
#if SUBDIVIDE
		// If we're in tuner mode, try to refine the pitch estimate by interpolating samples, for subsample accuracy.
		if ((result >= 0.0f) && (m_mode == TunerMode::Tuner))
		{
			// Upsample the signal to get a better bearing on the real frequency
			Fixed minSamples = bestOffset - incrementAtBestOffset;
			Fixed maxSamples = bestOffset + incrementAtBestOffset;
			unsigned long bestCorrellation = ~0;
			bestOffset = 0;
			for (Fixed offset = minSamples; offset <= maxSamples; ++offset) // step by one
			{
				unsigned long curCorrellation = GetCorrellationFactorFixed(m_buffer, offset);
				if (curCorrellation < bestCorrellation)
				{
					bestCorrellation = curCorrellation;
					bestOffset = offset;
				}
			}
		}
#endif

		int const T1 = AMPLITUDE_THRESHOLD;
		int const T2 = 120;

		if (m_maxAmplitude > T1)
		{
			if (result >= 0.0f)
			{
				// If we hit the end, assume no periodicity
				result = GetFrequencyForOffsetFixed(bestOffset);
			}
			digitalWrite(kStatusPin, HIGH); 
		}
		else
		{
			result = -1.0f;
			digitalWrite(kStatusPin, LOW); 
		}
		
		if (m_maxAmplitude > T2)
		{
			digitalWrite(kStatusPin2, HIGH); 
		}
		else
		{
			digitalWrite(kStatusPin2, LOW); 
		}

		return result;
	}

	void RenderWideGlyphForNote(int note)
	{
		if (note >= 0)
		{
			int nameIndex = GetNoteNameIndex(note);
			int glyphIndex = g_noteGlyphIndex[nameIndex];
			m_lcd.setWideCharacterGlyph(NOTE_GLYPH, g_noteGlyphs[glyphIndex]);
			m_lcd.setCursor(1, 0); // DO NOT FACTOR THIS OUT - setting a character glyph sets the next write address; this sets it to DD RAM, whereas setWideCharacterGlyph will leave it in CGRAM
			m_lcd.write(NOTE_GLYPH);
			m_lcd.write(NOTE_GLYPH + 1);
			if (g_noteSharpSign[nameIndex])
			{
				m_lcd.write('#');
			}
			else
			{
				m_lcd.write(' ');
			}
		}
		else
		{
			m_lcd.setCursor(1, 0);
			m_lcd.print("   ");
		}
	}

	void PrintCenterTick()
	{
#if ENABLE_LCD
		m_lcd.setCursor(DISPLAY_WIDTH >> 1, 0);
		m_lcd.write(TICK_GLYPH);
#endif
	}

	void LcdTick(float f)
	{
		uint8_t tick = uint8_t(f * MAX_TICK);
		uint8_t x = (tick / 5) + 1;
		uint8_t glyph = tick % 5;
#if ENABLE_LCD
		if (x < DISPLAY_WIDTH)
		{
			m_lcd.setCursor(x, 1);
			m_lcd.write(glyph);
		}
		if ((x != m_lastLcdTickX) && (m_lastLcdTickX < DISPLAY_WIDTH))
		{
			m_lcd.setCursor(m_lastLcdTickX, 1);
			m_lcd.write(' ');
		}
		m_lastLcdTickX = x;
#endif
	}

	void NoteOn()
	{
		Serial.write(0x90); // note on, channel 0
		Serial.write(char(m_midiNote));
		Serial.write(0x40); // velocity 64
	}

	void NoteOff()
	{
		if (m_midiNote >= 0)
		{
			Serial.write(0x80); // note off, channel 0;
			Serial.write(char(m_midiNote));
			Serial.write(0x7F); // velocity 127
			m_midiNote = -1;
		}
	}

	void TunePitch()
	{
		unsigned long lastPress = millis();
		bool firstTime = true;
		while (millis() - lastPress < 2000)
		{
			g_pitchDownButton.Update();
			g_pitchUpButton.Update();

			bool buttonPressed = firstTime;
			if (g_pitchDownButton.JustPressed())
			{
				m_a440 -= 1.0f;
				buttonPressed = true;
			}

			if (g_pitchUpButton.JustPressed())
			{
				m_a440 += 1.0f;
				buttonPressed = true;
			}

			if (m_a440 < MIN_FREQUENCY)
			{
				m_a440 = MIN_FREQUENCY;
			}

			if (m_a440 > MAX_FREQUENCY)
			{
				m_a440 = MAX_FREQUENCY;
			}

			if (buttonPressed)
			{
				lastPress = millis();
				m_lcd.clear();
				m_lcd.print("A = ");
				PrintFloat(m_a440, 2);
				m_lcd.print(" Hz   ");
			}

			firstTime = false;
		}
		m_lcd.clear();
		SaveTuning();
	}

	void Go()
	{
		Start();

		int i = 0;
		float filteredFrequency = -1.0f;
		int noteRepeatCount = 0;

		while(1)
		{
			g_pitchDownButton.Update();
			g_pitchUpButton.Update();

			if (g_pitchDownButton.IsPressed() || g_pitchUpButton.IsPressed())
			{
				TunePitch();
			}

			g_modeButton.Update();

			if (g_modeButton.JustPressed())
			{
				// Exit current mode
				switch (m_mode)
				{
				case TunerMode::Midi: NoteOff(); break;
				}

				// Cycle modes
				m_mode = static_cast<TunerMode::Type>((m_mode + 1) % TunerMode::Max);
				
				// Enter new mode
				m_lcd.clear();
				switch (m_mode)
				{
				case TunerMode::Midi:
					{
						m_newMidiNote = false;
						m_lastMaxAmplitude = 0;
					}
					break;
				}
			}

#if ENABLE_LCD
			m_lcd.home();
#endif

			float instantFrequency = DetermineSignalPitch();

			m_tunerNote = GetMidiNoteIndexForFrequency(instantFrequency);
			float centerFrequency = GetFrequencyForMidiNoteIndex(m_tunerNote);
			float minFrequency = centerFrequency * QUARTERTONE_DOWN;
			float maxFrequency = centerFrequency * QUARTERTONE_UP;

			if (instantFrequency >= 0.0f)
			{
				if ((filteredFrequency >= 0.0f) && (filteredFrequency >= minFrequency) && (filteredFrequency <= maxFrequency))
				{
					static float const RATE = 0.1f;
					filteredFrequency = (1.0f - RATE) * filteredFrequency + (RATE * instantFrequency);
				}
				else
				{
					filteredFrequency = instantFrequency;
				}
			}
			else
			{
				filteredFrequency = -1.0f;
			}

			// Mini state machine: (disallows voice though - dammit)
			// Detect note strikes
			// only trigger "new note" messages on strikes, or on 3-4 repeatcount
			// only allow semitone changes outside a strike; otherwise display nothing (i.e. false-detect an 330 as a 110, etc.)
			if (m_mode == TunerMode::Midi)
			{
				if (m_maxAmplitude - m_lastMaxAmplitude > 25)
				{
					m_newMidiNote = true;
				}
				if (m_tunerNote >= 0)
				{
					if (m_tunerNote != m_midiNote)
					{
						NoteOff();
						m_midiNote = m_tunerNote;
						noteRepeatCount = 0;
					}
					else
					{
						++noteRepeatCount;
					}

					if ((m_newMidiNote && (noteRepeatCount == 1)) || (!m_newMidiNote & (noteRepeatCount == 3)))
					{
						NoteOn();
						if (m_newMidiNote)
						{
							// Skip the repeat
							noteRepeatCount = 5;
							m_newMidiNote = false;
						}
					}
				}
				else
				{
					NoteOff();
				}
				m_lastMaxAmplitude = m_maxAmplitude;
			}

			float percent = 0.0f;
			if (filteredFrequency < centerFrequency)
			{
				percent = 0.5f * (filteredFrequency - minFrequency) / (centerFrequency - minFrequency);
			}
			else
			{
				percent = 0.5f + 0.5f * (filteredFrequency - centerFrequency) / (maxFrequency - centerFrequency);
			}
#if ENABLE_LCD
			RenderWideGlyphForNote(m_tunerNote);

			switch(m_mode)
			{
			case TunerMode::Midi:
				{
					m_lcd.setCursor(1, 1);
					m_lcd.print("(MIDI mode)");
				}
				break;
			case TunerMode::Tuner:
				{
					PrintCenterTick();
					if (filteredFrequency > 0.0f)
					{
						LcdTick(percent);
					}
					else
					{
						LcdTick(-1);
					}
				}
				break;
			}
#endif
		}

		Stop();
	}
private:
	int m_audioPin;
	char m_buffer[BUFFER_SIZE];
#if ENABLE_LCD
	LiquidCrystal m_lcd;
#endif
	int m_lastLcdTickX;
	float m_a440;
	int m_midiNote;
	int m_tunerNote;
	int m_maxAmplitude;
	int m_lastMaxAmplitude;
	TunerMode::Type m_mode;
	bool m_newMidiNote;
};
#undef ALWAYSPRINT
#undef DEBUGPRINT

///////////////////////////////////////////////////////////////////////////////
// Main stuff
///////////////////////////////////////////////////////////////////////////////

void setup()                                        // run once, when the sketch starts
{
	//Serial.begin(9600); // for debugging
	Serial.begin(31250); // for MIDI
	pinMode(kStatusPin, OUTPUT);            // sets the digital pin as output
	pinMode(kStatusPin2, OUTPUT);            // sets the digital pin as output
	digitalWrite(kStatusPin, LOW); 
	digitalWrite(kStatusPin2, LOW);
}

void loop()
{
	Tuner tuner(5); // tried ADC pins 0, 3, 5, none is better
	tuner.Go();
} 
