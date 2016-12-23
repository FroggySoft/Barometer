/*
 Copyright (c) 2015 FroggySoft.com.  All right reserved.

The Nokia 5110 is a basic graphic LCD screen for lots of applications. It was 
originally intended for as a cell phone screen. This one is mounted on an easy
to solder PCB.
It uses the PCD8544 controller, which is the same used in the Nokia 3310 LCD. 
The PCD8544 is a low power CMOS LCD controller/driver, designed to drive a 
graphic display of 48 rows and 84 columns. All necessary functions for the 
display are provided in a single chip, including on-chip generation of LCD 
supply and bias voltages, resulting in a minimum of external components and 
low power consumption. The PCD8544 interfaces to microcontrollers through a 
serial bus interface (SPI).

*/

//#define Use_Serial 
//#define Use_SPI

#include <EEPROM.h>
#include <Wire.h>
#ifdef Use_SPI
#include <SPI.h>
#endif
#include <math.h>
#include <LowPower.h>
#include <EEPROM.h>

#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define LCD_COMMAND 0
#define LCD_DATA    1
#define LCD_BUF_SIZE  504

#define MIN_PRESSURE  960
#define MAX_PRESSURE  1040

#define LCD_MIN_Y    0
#define LCD_MAX_Y    47
#define LCD_MIN_X    0
#define LCD_MAX_X    83

#define FONT_HEIGHT  12

// To reduce the need of character font data, we ony define those letters which we need:
// 0..9 C m B a r
// and to make things easy, we just redefine some characters:
// a -> D
// r -> E

const byte SmallFont[] =
{
//0x06, 0x08, 0x20, 0x5f,
0x06, 0x08, 0x2E, 0x5f,
/*
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // sp
0x00, 0x00, 0x00, 0x2f, 0x00, 0x00,   // !
0x00, 0x00, 0x07, 0x00, 0x07, 0x00,   // "
0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14,   // #
0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12,   // $
0x00, 0x62, 0x64, 0x08, 0x13, 0x23,   // %
0x00, 0x36, 0x49, 0x55, 0x22, 0x50,   // &
0x00, 0x00, 0x05, 0x03, 0x00, 0x00,   // '
0x00, 0x00, 0x1c, 0x22, 0x41, 0x00,   // (
0x00, 0x00, 0x41, 0x22, 0x1c, 0x00,   // )
0x00, 0x14, 0x08, 0x3E, 0x08, 0x14,   // *
0x00, 0x08, 0x08, 0x3E, 0x08, 0x08,   // +
0x00, 0x00, 0x00, 0xA0, 0x60, 0x00,   // ,
0x00, 0x08, 0x08, 0x08, 0x08, 0x08,   // -
*/
0x00, 0x00, 0x60, 0x60, 0x00, 0x00,   // .
0x00, 0x20, 0x10, 0x08, 0x04, 0x02,   // /
0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E,   // 0
0x00, 0x00, 0x42, 0x7F, 0x40, 0x00,   // 1
0x00, 0x42, 0x61, 0x51, 0x49, 0x46,   // 2
0x00, 0x21, 0x41, 0x45, 0x4B, 0x31,   // 3
0x00, 0x18, 0x14, 0x12, 0x7F, 0x10,   // 4
0x00, 0x27, 0x45, 0x45, 0x45, 0x39,   // 5
0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30,   // 6
0x00, 0x01, 0x71, 0x09, 0x05, 0x03,   // 7
0x00, 0x36, 0x49, 0x49, 0x49, 0x36,   // 8
0x00, 0x06, 0x49, 0x49, 0x29, 0x1E,   // 9
0x00, 0x00, 0x36, 0x36, 0x00, 0x00,   // :
0x00, 0x00, 0x56, 0x36, 0x00, 0x00,   // ;
0x00, 0x08, 0x14, 0x22, 0x41, 0x00,   // <
0x00, 0x14, 0x14, 0x14, 0x14, 0x14,   // =
0x00, 0x00, 0x41, 0x22, 0x14, 0x08,   // >
0x00, 0x02, 0x01, 0x51, 0x09, 0x06,   // ?

0x00, 0x32, 0x49, 0x59, 0x51, 0x3E,   // @
0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C,   // A
0x00, 0x7F, 0x49, 0x49, 0x49, 0x36,   // B
0x00, 0x3E, 0x41, 0x41, 0x41, 0x22,   // C
/*
0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C,   // D
0x00, 0x7F, 0x49, 0x49, 0x49, 0x41,   // E
0x00, 0x7F, 0x09, 0x09, 0x09, 0x01,   // F
0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A,   // G
0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F,   // H
0x00, 0x00, 0x41, 0x7F, 0x41, 0x00,   // I
0x00, 0x20, 0x40, 0x41, 0x3F, 0x01,   // J
0x00, 0x7F, 0x08, 0x14, 0x22, 0x41,   // K
0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,   // L
0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F,   // M
0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F,   // N
0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E,   // O

0x00, 0x7F, 0x09, 0x09, 0x09, 0x06,   // P
0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E,   // Q
0x00, 0x7F, 0x09, 0x19, 0x29, 0x46,   // R
0x00, 0x46, 0x49, 0x49, 0x49, 0x31,   // S
0x00, 0x01, 0x01, 0x7F, 0x01, 0x01,   // T
0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F,   // U
0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F,   // V
0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F,   // W
0x00, 0x63, 0x14, 0x08, 0x14, 0x63,   // X
0x00, 0x07, 0x08, 0x70, 0x08, 0x07,   // Y
0x00, 0x61, 0x51, 0x49, 0x45, 0x43,   // Z

0x00, 0x00, 0x7F, 0x41, 0x41, 0x00,   // [
0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,   // Backslash (Checker pattern)
0x00, 0x00, 0x41, 0x41, 0x7F, 0x00,   // ]
0x00, 0x04, 0x02, 0x01, 0x02, 0x04,   // ^
0x00, 0x40, 0x40, 0x40, 0x40, 0x40,   // _

0x00, 0x00, 0x03, 0x05, 0x00, 0x00,   // `
*/
0x00, 0x20, 0x54, 0x54, 0x54, 0x78,   // a
/*
0x00, 0x7F, 0x48, 0x44, 0x44, 0x38,   // b
0x00, 0x38, 0x44, 0x44, 0x44, 0x20,   // c
0x00, 0x38, 0x44, 0x44, 0x48, 0x7F,   // d
0x00, 0x38, 0x54, 0x54, 0x54, 0x18,   // e
0x00, 0x08, 0x7E, 0x09, 0x01, 0x02,   // f
0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,   // g
0x00, 0x7F, 0x08, 0x04, 0x04, 0x78,   // h
0x00, 0x00, 0x44, 0x7D, 0x40, 0x00,   // i
0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,   // j
0x00, 0x7F, 0x10, 0x28, 0x44, 0x00,   // k
0x00, 0x00, 0x41, 0x7F, 0x40, 0x00,   // l
*/
0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,   // m
/*
0x00, 0x7C, 0x08, 0x04, 0x04, 0x78,   // n
0x00, 0x38, 0x44, 0x44, 0x44, 0x38,   // o

0x00, 0xFC, 0x24, 0x24, 0x24, 0x18,   // p
0x00, 0x18, 0x24, 0x24, 0x18, 0xFC,   // q
*/
0x00, 0x7C, 0x08, 0x04, 0x04, 0x08,   // r
/*
0x00, 0x48, 0x54, 0x54, 0x54, 0x20,   // s
0x00, 0x04, 0x3F, 0x44, 0x40, 0x20,   // t
0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C,   // u
0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C,   // v
0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C,   // w
0x00, 0x44, 0x28, 0x10, 0x28, 0x44,   // x
0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,   // y
0x00, 0x44, 0x64, 0x54, 0x4C, 0x44,   // z
0x00, 0x00, 0x10, 0x7C, 0x82, 0x00,   // {
0x00, 0x00, 0x00, 0xFF, 0x00, 0x00,   // |
0x00, 0x00, 0x82, 0x7C, 0x10, 0x00,   // }
0x00, 0x00, 0x06, 0x09, 0x09, 0x06    // ~ (Degrees)
*/
};

byte scrbuf[LCD_BUF_SIZE];

struct _current_font
{
	const byte* font;
	byte x_size;
	byte y_size;
	byte offset;
	byte numchars;
};
_current_font cfont;

// IO for the I2C bus to the sensor
const int PinSda = 8;
const int PinScl = 9;
// For a pro mini the SDA is on A4 and the SCL on A5

// IO for the SPI bus to the display
#ifndef Use_SPI
const int PinSck = 2;
const int PinMosi = 3;
#endif
const int PinDs = 4;
const int PinRst = 5;
const int PinCs = 6;

double mTemp=0;
double mPress=0;
byte mHistory[LCD_MAX_X+1];

int AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
unsigned int AC4,AC5,AC6; 
double c5,c6,mc,md,x0,x1,x2,my0,my1,my2,p0,p1,p2;
char _error;
int sampleCounter=0;

void setup() {
#ifdef Use_Serial
  Serial.begin(9600);
  Serial.println("Reset");
#endif

  // init SPI to the LCD

#ifndef Use_SPI
  pinMode(PinSck,OUTPUT);
  pinMode(PinMosi,OUTPUT);
#else
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
#endif
  pinMode(PinCs,OUTPUT);
  pinMode(PinDs,OUTPUT);
  pinMode(PinRst,OUTPUT);

  InitLCD();
  setFont(SmallFont);

  // init I2C to the sensor
  Wire.begin();  
  InitBMP180();
  
  for( int i=0; i<=LCD_MAX_X; i++)
  {
    mHistory[i]=-1;
  }
  
  // when magic numbers do not exists: create then
  if(EEPROM.read(0)!=0x5A || EEPROM.read(1)!=0xBC)
  {
    EEPROM.write(0, 0x5A);
    EEPROM.write(1, 0xBC);
    for(int i=0; i<=LCD_MAX_X; i++)
    {
      mHistory[i] = -1;
    }
  }
  else  // read old values form NV mem
  {
    for(int i=0; i<=LCD_MAX_X; i++)
    {
      mHistory[i] = EEPROM.read(i+2);
    }
  }
  
#ifdef Use_Serial
  for(int i=0; i<=LCD_MAX_X; i++)
  {
    Serial.print(mHistory[i]+MIN_PRESSURE);
    Serial.print(",");
  }
  Serial.println();
#endif
}


void loop()
{
  int x=0;
  
  clrScr();

#ifdef Use_Serial
  Serial.print("Temp=");
#endif
  if( startTemperature()!=0 )
  {
    delay(5);
    if( getTemperature(mTemp)==1 )
    {
      int d = startPressure(1);    // start for next sample
      x = printNumF(mTemp,1,x, 0);
      x = printLcd("C",x, 0);
#ifdef Use_Serial
      Serial.println(mTemp);

      Serial.print("Pressure=");
#endif
      if( d>0 )
      {
        x = 38;
        delay(d);
        if( getPressure(mPress,mTemp)==1 )
        {
          x = printNumI(mPress,x, 0);
// the a, m and r and redefined as D, E and F
          printLcd("EBDF",x, 0);    // actually writes "mBar" 
#ifdef Use_Serial
          Serial.println(mPress);
#endif
          sampleCounter--;
          if(sampleCounter<=0)
          {
            addPressureToHistory(mPress);
            sampleCounter=360;  // one sample every hour
          }

        }
      }
      else
      {
        printLcd("??",x, 0);
#ifdef Use_Serial        
        Serial.println("??");
#endif
      }
    }
  }
  else
  {  
    printLcd("??",x, 0);
#ifdef Use_Serial
    Serial.println("??");
#endif
  }

  setHorRuler( LCD_MAX_Y - 40 );
  setHorRuler( LCD_MAX_Y - 30 );
  setHorRuler( LCD_MAX_Y - 20 );
  setHorRuler( LCD_MAX_Y - 10 );
  setHorRuler( LCD_MAX_Y - 0 );
  // 1 sample/hour with 84 pixles gives you 84/24=3.5 days
  setVerRuler(LCD_MAX_X-24-24-24);
  setVerRuler(LCD_MAX_X-24-24);
  setVerRuler(LCD_MAX_X-24);

  int min = MAX_PRESSURE;
  for( int i=0; i<=LCD_MAX_X; i++)
  {
    if(mHistory[i]!=-1)
    {
      int v = mHistory[i] + MIN_PRESSURE;
      if(v<min)
      {
        min = v;
      }
    }
  }
  if(min>(MAX_PRESSURE-40))
  {
    min = MAX_PRESSURE-40;
  }
  
  // round min value to units of 10mBar
  min /= 10;
  min *= 10;

  for( int i=0; i<=LCD_MAX_X; i++)
  {
    if(mHistory[i]!=-1)
    {
      int v = mHistory[i] + MIN_PRESSURE;
      int y = LCD_MAX_Y - (v - min);
      setPixel(i,y);
    }
  }
  
  updateLcd();
  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); 
}

void addPressureToHistory(double aPress)
{
  // only store the offset to the min pressure (so it fits in 1 byte) 
  double p1 = aPress - MIN_PRESSURE; 
  p1 += 0.5;  // to get a correct rounded number
  byte    p2 = (byte)p1;
  if(p2>MAX_PRESSURE)
  {
    p2 = MAX_PRESSURE;
  }
  
  for( int i=1; i<=LCD_MAX_X; i++)
  {
    mHistory[i-1]=mHistory[i];
  }
  mHistory[LCD_MAX_X] = p2;
  for(int i=0; i<=LCD_MAX_X; i++)
  {
    EEPROM.write(i+2,mHistory[i]);
  }
}

void _LCD_Write(unsigned char data, unsigned char mode)
{   

	if (mode==LCD_COMMAND)
		digitalWrite(PinDs,LOW);  // Command
	else
		digitalWrite(PinDs,HIGH); // Data

	digitalWrite(PinCs,LOW);

#ifdef Use_SPI
        SPI.transfer(data);
#else
	for (unsigned char c=0; c<8; c++)
	{
		if (data & 0x80)
			digitalWrite(PinMosi,HIGH);
		else
			digitalWrite(PinMosi,LOW);
		data = data<<1;

		digitalWrite(PinSck,LOW);
		asm ("nop");
		digitalWrite(PinSck,HIGH);
	}
#endif
	digitalWrite(PinCs,HIGH);
}

void InitLCD()
{
#ifndef Use_SPI
	digitalWrite(PinMosi,HIGH);
	digitalWrite(PinSck,HIGH);
#endif
	digitalWrite(PinDs,HIGH);
	digitalWrite(PinCs,HIGH);

	digitalWrite(PinRst,LOW);
	delay(10);
	digitalWrite(PinRst,HIGH);

	_LCD_Write(0x21, LCD_COMMAND);    //switch to extended commands
	_LCD_Write(0x80|0x38, LCD_COMMAND); //set value of Vop (controls contrast)
	_LCD_Write(0x04, LCD_COMMAND); //set temperature coefficient
	_LCD_Write(0x14, LCD_COMMAND); //set bias mode to 1:48.
	_LCD_Write(0x20, LCD_COMMAND); //switch back to regular commands
	_LCD_Write(0x0C, LCD_COMMAND); //enable normal display (dark on light), horizontal addressing
        
	clrScr();
	updateLcd();
	cfont.font=0;
}

void updateLcd()
{
	_LCD_Write(0x40, LCD_COMMAND);
	_LCD_Write(0x80, LCD_COMMAND);

	for (int b=0; b<LCD_BUF_SIZE; b++)
		_LCD_Write(scrbuf[b], LCD_DATA);
}

void clrScr()
{
	for (int c=0; c<LCD_BUF_SIZE; c++)
		scrbuf[c]=0;
}

void invert(bool mode)
{
	if (mode==true)
		_LCD_Write(0x0D, LCD_COMMAND);
	else
		_LCD_Write(0x0C, LCD_COMMAND);
}

void setPixel(int x, int y)
{
	int by, bi;

	if ((x>=0) and (x<=LCD_MAX_X) and (y>=0) and (y<=LCD_MAX_Y))
	{
		by=((y/8)*(LCD_MAX_X+1))+x;
		bi=y % 8;

		scrbuf[by]=scrbuf[by] | (1<<bi);
	}
}

void setHorRuler(int y)
{
  if( (y>=0) and (y<=LCD_MAX_Y) )
  {
    int bi = 1<<(y%8);
    for( int x=0; x<(LCD_MAX_X+1); x+=2)
    {
      int by=((y/8)*(LCD_MAX_X+1))+x;
      scrbuf[by]=scrbuf[by] | bi;
    }
  }
}

void setVerRuler(int x)
{
  int by, bi;
  if ((x>=0) and (x<84))
  {
    for( int y=10; y<=LCD_MAX_Y; y+=2)
    {
      by=((y/8)*(LCD_MAX_X+1))+x;
      bi=y % 8;
      scrbuf[by]=scrbuf[by] | (1<<bi);
    }
  }
}

void clrPixel(int x, int y)
{
  int by, bi;
  
  if ((x>=0) and (x<=LCD_MAX_X) and (y>=0) and (y<=LCD_MAX_Y))
  {
    by=((y/8)*(LCD_MAX_X+1))+x;
    bi=y % 8;
    scrbuf[by]=scrbuf[by] & ~(1<<bi);
  }
}

void invPixel(int x, int y)
{
	int by, bi;

  if ((x>=0) and (x<=LCD_MAX_X) and (y>=0) and (y<=LCD_MAX_Y))
  {
    by=((y/8)*(LCD_MAX_X+1))+x;
    bi=y % 8;

    if ((scrbuf[by] & (1<<bi))==0)
      scrbuf[by]=scrbuf[by] | (1<<bi);
    else
      scrbuf[by]=scrbuf[by] & ~(1<<bi);
  }
}

int printLcd(char *st, int x, int y)
{
  unsigned char ch;
  int stl;

  stl = strlen(st);
  /*
  if (x == RIGHT)
    x = (LCD_MAX_X+1)-(stl*cfont.x_size);
  if (x == CENTER)
    x = ((LCD_MAX_X+1)-(stl*cfont.x_size))/2;
  */
  for (int cnt=0; cnt<stl; cnt++)
  {
    _print_char(*st++, x, y);
    x += cfont.x_size;
  }
  return x;
}

int printNumI(long num, int x, int y)
{
  char buf[13];
  char st[15];
  boolean neg=false;
  int c=0;
  
  if (num==0)
  {
	  st[0]=48;
	  st[1]=0;
  }
  else
  {
	  if (num<0)
	  {
		neg=true;
		num=-num;
	  }
	  
	  while (num>0)
	  {
		buf[c]=48+(num % 10);
		c++;
		num=(num-(num % 10))/10;
	  }
	  buf[c]=0;
	  
	  if (neg)
	  {
		st[0]=45;
	  }
	  
	  for (int i=0; i<c; i++)
	  {
		st[i+neg]=buf[c-i-1];
	  }
	  st[c+neg]=0;
  }

  return printLcd(st,x,y);
}

int printNumF(double num, byte dec, int x, int y)
{
  char buf[25];
  char st[27];
  boolean neg=false;
  int c=0;
  int c2;
  unsigned long inum;
  
  if (num==0)
  {
	  st[0]=48;
	  st[1]=46;
	  for (int i=0; i<dec; i++)
		  st[2+i]=48;
	  st[2+dec]=0;
  }
  else
  {
	  if (num<0)
	  {
		neg=true;
		num=-num;
	  }
	  
	  if (dec<1)
		dec=1;
	  if (dec>5)
		dec=5;
	  
	  inum=long(num*pow(10,dec));
	  
	  while (inum>0)
	  {
		buf[c]=48+(inum % 10);
		c++;
		inum=(inum-(inum % 10))/10;
	  }
	  if ((num<1) and (num>0))
	  {
		  buf[c]=48;
		  c++;
	  }
	  buf[c]=0;
	  
	  if (neg)
	  {
		st[0]=45;
	  }
	  
	  c2=neg;
	  for (int i=0; i<c; i++)
	  {
		st[c2]=buf[c-i-1];
		c2++;
		if ((c-(c2-neg))==dec)
		{
		  st[c2]=46;
		  c2++;
		}
	  }
	  st[c2]=0;
  }

  return printLcd(st,x,y);
}

void _print_char(unsigned char c, int x, int y)
{
	if ((cfont.y_size % 8) == 0)
	{
		int font_idx = ((c - cfont.offset)*(cfont.x_size*(cfont.y_size/8)))+4;
		for (int rowcnt=0; rowcnt<(cfont.y_size/8); rowcnt++)
		{
			for(int cnt=0; cnt<cfont.x_size; cnt++)
			{
				for (int b=0; b<8; b++)
					if ((cfont.font[font_idx+cnt+(rowcnt*cfont.x_size)] & (1<<b))!=0)
						setPixel(x+cnt, y+(rowcnt*8)+b);
					else
						clrPixel(x+cnt, y+(rowcnt*8)+b);
			}
		}
	}
	else
	{
		int font_idx = ((c - cfont.offset)*((cfont.x_size*cfont.y_size/8)))+4;
		int cbyte=cfont.font[font_idx];
		int cbit=7;
		for (int cx=0; cx<cfont.x_size; cx++)
		{
			for (int cy=0; cy<cfont.y_size; cy++)
			{
				if ((cbyte & (1<<cbit)) != 0)
					setPixel(x+cx, y+cy);
				else
					clrPixel(x+cx, y+cy);
				cbit--;
				if (cbit<0)
				{
					cbit=7;
					font_idx++;
					cbyte=cfont.font[font_idx];
				}
			}
		}
	}
}

void setFont(const uint8_t* font)
{
	cfont.font=font;
/*
	cfont.x_size=pgm_read_byte(&cfont.font[0]);
	cfont.y_size=pgm_read_byte(&cfont.font[1]);
	cfont.offset=pgm_read_byte(&cfont.font[2]);
	cfont.numchars=pgm_read_byte(&cfont.font[3]);
*/
	cfont.x_size=cfont.font[0];
	cfont.y_size=cfont.font[1];
	cfont.offset=cfont.font[2];
	cfont.numchars=cfont.font[3];
}

//byte getFontHeight()
//{
//  return cfont.y_size;
//}

#define BMP180_ADDR 0x77 // 7-bit address

#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

/*
	SFE_BMP180.cpp
	Bosch BMP180 pressure sensor library for the Arduino microcontroller
	Mike Grusin, SparkFun Electronics

	Uses floating-point equations from the Weather Station Data Logger project
	http://wmrx00.sourceforge.net/
	http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

	Forked from BMP085 library by M.Grusin

	version 1.0 2013/09/20 initial version

	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a (root) beer someday.
*/


char InitBMP180()
// Initialize library for subsequent pressure measurements
{
	double c3,c4,b1;
	
	// Start up the Arduino's "wire" (I2C) library:
	
//	Wire.begin();

	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.

	// Retrieve calibration data from device:
	
	if (readInt(0xAA,AC1) &&
		readInt(0xAC,AC2) &&
		readInt(0xAE,AC3) &&
		readUInt(0xB0,AC4) &&
		readUInt(0xB2,AC5) &&
		readUInt(0xB4,AC6) &&
		readInt(0xB6,VB1) &&
		readInt(0xB8,VB2) &&
		readInt(0xBA,MB) &&
		readInt(0xBC,MC) &&
		readInt(0xBE,MD))
	{

		// All reads completed successfully!

		// If you need to check your math using known numbers,
		// you can uncomment one of these examples.
		// (The correct results are commented in the below functions.)

		// Example from Bosch datasheet
		// AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
		// B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

		// Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
		// AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
		// VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;

		/*
		Serial.print("AC1: "); Serial.println(AC1);
		Serial.print("AC2: "); Serial.println(AC2);
		Serial.print("AC3: "); Serial.println(AC3);
		Serial.print("AC4: "); Serial.println(AC4);
		Serial.print("AC5: "); Serial.println(AC5);
		Serial.print("AC6: "); Serial.println(AC6);
		Serial.print("VB1: "); Serial.println(VB1);
		Serial.print("VB2: "); Serial.println(VB2);
		Serial.print("MB: "); Serial.println(MB);
		Serial.print("MC: "); Serial.println(MC);
		Serial.print("MD: "); Serial.println(MD);
		*/
		
		// Compute floating-point polynominals:

		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		my0 = c4 * pow(2,15);
		my1 = c4 * c3;
		my2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);

		/*
		Serial.println();
		Serial.print("c3: "); Serial.println(c3);
		Serial.print("c4: "); Serial.println(c4);
		Serial.print("c5: "); Serial.println(c5);
		Serial.print("c6: "); Serial.println(c6);
		Serial.print("b1: "); Serial.println(b1);
		Serial.print("mc: "); Serial.println(mc);
		Serial.print("md: "); Serial.println(md);
		Serial.print("x0: "); Serial.println(x0);
		Serial.print("x1: "); Serial.println(x1);
		Serial.print("x2: "); Serial.println(x2);
		Serial.print("y0: "); Serial.println(y0);
		Serial.print("y1: "); Serial.println(y1);
		Serial.print("y2: "); Serial.println(y2);
		Serial.print("p0: "); Serial.println(p0);
		Serial.print("p1: "); Serial.println(p1);
		Serial.print("p2: "); Serial.println(p2);
		*/
		
		// Success!
		return(1);
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return(0);
	}
}

char readBytesBmp180(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
	char x;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values[0]);
	_error = Wire.endTransmission();
	if (_error == 0)
	{
		Wire.requestFrom(BMP180_ADDR,length);
		while(Wire.available() != length) ; // wait until bytes are ready
		for(x=0;x<length;x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);
}


char writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
	char x;
	
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values,length);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(1);
	else
		return(0);
}


char readInt(char address, int &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (readBytesBmp180(data,2))
	{
		value = (((int)data[0]<<8)|(int)data[1]);
		if (value & 0x8000) value |= 0xFFFF0000; // sign extend if negative
		return(1);
	}
	value = 0;
	return(0);
}


char readUInt(char address, unsigned int &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (readBytesBmp180(data,2))
	{
		value = (((unsigned int)data[0]<<8)|(unsigned int)data[1]);
		return(1);
	}
	value = 0;
	return(0);
}



char startTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
	unsigned char data[2], result;
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = writeBytes(data, 2);
	if (result) // good write?
		return(5); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


char getTemperature(double &T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	double tu, a;
	
	data[0] = BMP180_REG_RESULT;

	result = readBytesBmp180(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];

		//example from Bosch datasheet
		//tu = 27898;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		//tu = 0x69EC;
		
		a = c5 * (tu - c6);
		T = a + (mc / (a + md));

		/*		
		Serial.println();
		Serial.print("tu: "); Serial.println(tu);
		Serial.print("a: "); Serial.println(a);
		Serial.print("T: "); Serial.println(*T);
		*/
	}
	return(result);
}


char startPressure(char oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	unsigned char data[2], result, delay;
	
	data[0] = BMP180_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
			delay = 8;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
			delay = 14;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
			delay = 26;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
			delay = 5;
		break;
	}
	result = writeBytes(data, 2);
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}


char getPressure(double &P, double &T)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	char result;
	double pu,s,x,y,z;
	
	data[0] = BMP180_REG_RESULT;

	result = readBytesBmp180(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

		//example from Bosch datasheet
		//pu = 23843;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;	
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
		
		s = T - 25.0;
		x = (x2 * pow(s,2)) + (x1 * s) + x0;
		y = (my2 * pow(s,2)) + (my1 * s) + my0;
		z = (pu - x) / y;
		P = (p2 * pow(z,2)) + (p1 * z) + p0;

		/*
		Serial.println();
		Serial.print("pu: "); Serial.println(pu);
		Serial.print("T: "); Serial.println(*T);
		Serial.print("s: "); Serial.println(s);
		Serial.print("x: "); Serial.println(x);
		Serial.print("y: "); Serial.println(y);
		Serial.print("z: "); Serial.println(z);
		Serial.print("P: "); Serial.println(*P);
		*/
	}
	return(result);
}


double sealevel(double P, double A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


char getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(_error);
}



