// OLED SSD1306 using the I2C interface
// Written by Larry Bank (bitbank@pobox.com)
// Project started 1/15/2017
//
// The I2C writes (through a file handle) can be single or multiple bytes.
// The write mode stays in effect throughout each call to write()
// To write commands to the OLED controller, start a byte sequence with 0x00,
// to write data, start a byte sequence with 0x40,
// The OLED controller is set to "page mode". This divides the display
// into 8 128x8 "pages" or strips. Each data write advances the output
// automatically to the next address. The bytes are arranged such that the LSB
// is the topmost pixel and the MSB is the bottom.
// The font data comes from another source and must be rotated 90 degrees
// (at init time) to match the orientation of the bits on the display memory.
// A copy of the display memory is maintained by this code so that single pixel
// writes can occur without having to read from the display controller.

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "oled96.h"

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

// extern char ucFont[], ucSmallFont[];
extern char verysmallfont[], mtbbblogo[];
char space[]= {0x00, 0x00, 0x00, 0x00, 0x00};
char lt[]= {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x03, 0x01, 0x7F, 0x01, 0x03, 0x00};
static int iScreenOffset; // current write offset of screen data
static char ucScreen[1024]; // local copy of the image buffer
static int file_i2c = 0;
static int oled_type, oled_flip;

static void oledWriteCommand(char);
//
// Opens a file system handle to the I2C device
// Initializes the OLED controller into "page mode"
// Prepares the font data for the orientation of the display
// Returns 0 for success, 1 for failure
//
int oledInit(int iChannel, int iAddr, int iType, int bFlip, int bInvert)
{
const char oled64_initbuf[]={0x00,0xae,0xa8,0x3f,0xd3,0x00,0x40,0xa1,0xc8,
      0xda,0x12,0x81,0xff,0xa4,0xa6,0xd5,0x80,0x8d,0x14,
      0xaf,0x20,0x02};
const char oled32_initbuf[] = {
0x00,0xae,0xd5,0x80,0xa8,0x1f,0xd3,0x00,0x40,0x8d,0x14,0xa1,0xc8,0xda,0x02,
0x81,0x7f,0xd9,0xf1,0xdb,0x40,0xa4,0xa6,0xaf};

char filename[32];
char uc[4];

	oled_type = iType;
	oled_flip = bFlip;
	sprintf(filename, "/dev/i2c-%d", iChannel);
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		fprintf(stderr, "Failed to open i2c bus %d\n", iChannel);
		file_i2c = 0;
		return 1;
	}

	if (ioctl(file_i2c, I2C_SLAVE, iAddr) < 0)
	{
		fprintf(stderr, "Failed to acquire bus access or talk to slave\n");
		file_i2c = 0;
		return 1;
	}

	if (iType == OLED_128x32)
	{
		write(file_i2c, oled32_initbuf, sizeof(oled32_initbuf));
	}
	else
	{
		write(file_i2c, oled64_initbuf, sizeof(oled64_initbuf));
	}
	if (bInvert)
	{
		uc[0] = 0; // command
		uc[1] = 0xa7; // invert command
		write(file_i2c, uc, 2);
	}
	if (bFlip) // rotate display 180
	{
		uc[0] = 0; // command
		uc[1] = 0xa0;
		write(file_i2c, uc, 2);
		uc[1] = 0xc0;
		write(file_i2c, uc, 2);
	}
	return 0;
} /* oledInit() */

// Sends a command to turn off the OLED display
// Closes the I2C file handle
void oledShutdown()
{
	if (file_i2c != 0)
	{
		oledWriteCommand(0xaE); // turn off OLED
		close(file_i2c);
		file_i2c = 0;
	}
}

// Send a single byte command to the OLED controller
static void oledWriteCommand(char c)
{
char buf[2];
int rc;

	buf[0] = 0x00; // command introducer
	buf[1] = c;
	rc = write(file_i2c, buf, 2);
	if (rc) {} // suppress warning
} /* oledWriteCommand() */

static void oledWriteCommand2(char c, char d)
{
char buf[3];
int rc;

	buf[0] = 0x00;
	buf[1] = c;
	buf[2] = d;
	rc = write(file_i2c, buf, 3);
	if (rc) {} // suppress warning
} /* oledWriteCommand2() */

int oledSetContrast(char ucContrast)
{
        if (file_i2c == 0)
                return -1;

	oledWriteCommand2(0x81, ucContrast);
	return 0;
} /* oledSetContrast() */

//static void oledWriteData(char c)
//{
//char buf[2];
//        buf[0] = 0x40; // data introducer
//        buf[1] = c;
//        write(file_i2c, buf, 2);
//} /* oledWriteData() */

// Send commands to position the "cursor" to the given
// row and column
static void oledSetPosition(int x, int y)
{
	iScreenOffset = (y*128)+x;
	if (oled_type == OLED_64x32) // visible display starts at column 32, row 4
	{
		x += 32; // display is centered in VRAM, so this is always true
		if (oled_flip == 0) // non-flipped display starts from line 4
		y += 4;
	}
	else if (oled_type == OLED_132x64) // SH1106 has 128 pixels centered in 132
	{
		x += 2;
	}

	oledWriteCommand(0xb0 | y); // go to page Y
	oledWriteCommand(0x00 | (x & 0xf)); // // lower col addr
	oledWriteCommand(0x10 | ((x >> 4) & 0xf)); // upper col addr
}

//static void oledWrite(char c)
//{
//	write(file_i2c, &c, 1);
//} /* oledWrite() */

// Write a block of pixel data to the OLED
// Length can be anything from 1 to 1024 (whole display)
static void oledWriteDataBlock(const std::string& ucBuf, int iLen)
{

  // debugging
  // if (std::memcmp(&space, ucBuf.data(), 5) == 0){
  //   std::cout << "Attempting to print space" << std::endl;
  // else {
  //   // std::cout << "Blank and ucBuf are NOT the same" << std::endl;
  // }

  char ucTemp[129];
  int rc;

  ucTemp[0] = 0x40; // data command
  std::memcpy(&ucTemp[1], ucBuf.data(), iLen);

  rc = write(file_i2c, ucTemp, iLen+1);
  if (rc) {} // suppress warning
  // Keep a copy in local buffer
  std::memcpy(&ucScreen[iScreenOffset], ucBuf.data(), iLen);

  iScreenOffset += iLen;

}

// Write a block of pixel data to the OLED
// Length can be anything from 1 to 1024 (whole display)
void oledWriteLogo()
{
  int y;
  int iLines, iCols;

  	iLines = 8;
  	iCols = 8;

  	// std::memset(temp.c_str(), ucData, 128);
  	for (y=0; y<iLines; y++)
  	{
  		oledSetPosition(0,y); // set to (0,Y)
  		oledWriteDataBlock(&mtbbblogo[(y*128)], iCols*16); // fill with data byte
  	} // for y
}

// Set (or clear) an individual pixel
// The local copy of the frame buffer is used to avoid
// reading data from the display controller
int oledSetPixel(int x, int y, char ucColor)
{
int i;
char uc, ucOld;

	if (file_i2c == 0)
		return -1;

	i = ((y >> 3) * 128) + x;
	if (i < 0 || i > 1023) // off the screen
		return -1;
	uc = ucOld = ucScreen[i];
	uc &= ~(0x1 << (y & 7));
	if (ucColor)
	{
		uc |= (0x1 << (y & 7));
	}
	if (uc != ucOld) // pixel changed
	{
		oledSetPosition(x, y>>3);
		oledWriteDataBlock(&uc, 1);
	}
	return 0;
} /* oledSetPixel() */
//
// Draw a string of small (8x8), large (16x24), or very small (6x8)  characters
// At the given col+row
// The X position is in character widths (8 or 16)
// The Y position is in memory pages (8 lines each)
//
int oledWriteString(int x, int y, std::string szMsg)
{
int i, iLen;
std::string s;

//debugging
// std::cout << szMsg << std::endl;

	iLen = szMsg.length();

    oledSetPosition(x*6, y);

		if (iLen + x > 25) iLen = 25 - x;
		if (iLen < 0) return -1;

		for (i=0; i<iLen; i++)
		{
      int cInt = szMsg[i];

      for(int j = 0; j < 6; j++){
        s[j] = verysmallfont[(cInt * 5) + j];
      }

      // Blank for the last line, this is for letter spacing
      s[5] = 0x0;

      // Checks to see if the character is different. Do not write if they are the same
      if (std::memcmp(&ucScreen[iScreenOffset], s.data(), iLen) == 0){
        // std::cout << "ucScreen and ucBuf are the same" << std::endl;
        x += 1;
        oledSetPosition(x*6,y);
      } else {
        oledWriteDataBlock(s, 6);
        x += 1;
      }
		}

	return 0;
} /* oledWriteString() */

// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
int oledFill(char ucData)
{
int y;
std::string temp;
int iLines, iCols;

	if (file_i2c == 0) return -1; // not initialized

	iLines = (oled_type == OLED_128x32 || oled_type == OLED_64x32) ? 4:8;
	iCols = (oled_type == OLED_64x32) ? 4:8;

  temp.resize(128,ucData);
	// std::memset(temp.c_str(), ucData, 128);
	for (y=0; y<iLines; y++)
	{
		oledSetPosition(0,y); // set to (0,Y)
		oledWriteDataBlock(temp, iCols*16); // fill with data byte
	} // for y
	return 0;
} /* oledFill() */
