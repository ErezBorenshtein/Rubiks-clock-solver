// Created by https://oleddisplay.squix.ch/ Consider a donation
// In case of problems make sure that you are using the font file with the correct version!
const uint8_t DSEG7_Modern_Bold_16Bitmaps[] PROGMEM = {

	// Bitmap Data:
	0x00, // ' '
	0x00, // '!'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '"'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '#'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '$'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '%'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '&'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '''
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '('
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // ')'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '*'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '+'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // ','
	0x7E,0xFC, // '-'
	0xD8, // '.'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '/'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // '0'
	0x09,0x6D,0xB0,0x5B,0x6D,0x90, // '1'
	0xFF,0x1F,0xE0,0x08,0x06,0x01,0x80,0x60,0x18,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x0F,0xF1,0xFE, // '2'
	0xFF,0x1F,0xE0,0x08,0x06,0x01,0x80,0x60,0x18,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x67,0xF9,0xFE, // '3'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x60,0x08, // '4'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x67,0xF9,0xFE, // '5'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // '6'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0x00,0x00,0x80,0x60,0x18,0x06,0x01,0x80,0x60,0x08, // '7'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // '8'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x67,0xF9,0xFE, // '9'
	0xD0,0x00,0x06,0x00, // ':'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // ';'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '<'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '='
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '>'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '?'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '@'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'A'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xFA,0xC1,0xB0,0x6C,0x1B,0x06,0x81,0xBF,0xE7,0xF8, // 'B'
	0x3F,0x1F,0x8C,0x03,0x00,0xC0,0x30,0x08,0x03,0xFC,0x7F,0x80, // 'C'
	0x00,0x00,0x20,0x08,0x06,0x01,0x80,0x60,0x18,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'D'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x0F,0xF1,0xFE, // 'E'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x08,0x00,0x00, // 'F'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'G'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xFA,0xC1,0xB0,0x6C,0x1B,0x06,0x81,0xA0,0x20,0x00, // 'H'
	0x0B,0x6D,0xB2, // 'I'
	0x00,0x00,0x20,0x08,0x06,0x01,0x80,0x60,0x18,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'J'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'K'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x08,0x00,0x00,0xC0,0x30,0x0C,0x03,0x00,0x80,0x3F,0xC7,0xF8, // 'L'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'M'
	0x3F,0x1F,0xAC,0x1B,0x06,0xC1,0xB0,0x68,0x1A,0x02,0x00,0x00, // 'N'
	0x3F,0x1F,0xAC,0x1B,0x06,0xC1,0xB0,0x68,0x1B,0xFE,0x7F,0x80, // 'O'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x08,0x00,0x00, // 'P'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x60,0x08, // 'Q'
	0x3F,0x3F,0x30,0x18,0x0C,0x06,0x02,0x01,0x00,0x00,0x00, // 'R'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xFA,0x01,0x80,0x60,0x18,0x06,0x01,0x9F,0xE7,0xF8, // 'S'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xF8,0xC0,0x30,0x0C,0x03,0x00,0x80,0x3F,0xC7,0xF8, // 'T'
	0x00,0x00,0x2C,0x1B,0x06,0xC1,0xB0,0x68,0x1B,0xFE,0x7F,0x80, // 'U'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'V'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'W'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'X'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x67,0xF9,0xFE, // 'Y'
	0xFF,0x1F,0xE0,0x08,0x06,0x01,0x80,0x60,0x18,0x00,0x00,0x30,0x0C,0x03,0x00,0xC0,0x20,0x0F,0xF1,0xFE, // 'Z'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '['
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '\'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // ']'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '^'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '_'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '`'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'a'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xFA,0xC1,0xB0,0x6C,0x1B,0x06,0x81,0xBF,0xE7,0xF8, // 'b'
	0x3F,0x1F,0x8C,0x03,0x00,0xC0,0x30,0x08,0x03,0xFC,0x7F,0x80, // 'c'
	0x00,0x00,0x20,0x08,0x06,0x01,0x80,0x60,0x18,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'd'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x0F,0xF1,0xFE, // 'e'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x08,0x00,0x00, // 'f'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'g'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xFA,0xC1,0xB0,0x6C,0x1B,0x06,0x81,0xA0,0x20,0x00, // 'h'
	0x0B,0x6D,0xB2, // 'i'
	0x00,0x00,0x20,0x08,0x06,0x01,0x80,0x60,0x18,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'j'
	0xFF,0x3F,0xCC,0x03,0x00,0xC0,0x30,0x0C,0x02,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'k'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x08,0x00,0x00,0xC0,0x30,0x0C,0x03,0x00,0x80,0x3F,0xC7,0xF8, // 'l'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'm'
	0x3F,0x1F,0xAC,0x1B,0x06,0xC1,0xB0,0x68,0x1A,0x02,0x00,0x00, // 'n'
	0x3F,0x1F,0xAC,0x1B,0x06,0xC1,0xB0,0x68,0x1B,0xFE,0x7F,0x80, // 'o'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x30,0x0C,0x03,0x00,0xC0,0x20,0x08,0x00,0x00, // 'p'
	0xFF,0x3F,0xEC,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x60,0x08, // 'q'
	0x3F,0x3F,0x30,0x18,0x0C,0x06,0x02,0x01,0x00,0x00,0x00, // 'r'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xFA,0x01,0x80,0x60,0x18,0x06,0x01,0x9F,0xE7,0xF8, // 's'
	0x80,0x30,0x0C,0x03,0x00,0xC0,0x30,0x0B,0xF1,0xF8,0xC0,0x30,0x0C,0x03,0x00,0x80,0x3F,0xC7,0xF8, // 't'
	0x00,0x00,0x2C,0x1B,0x06,0xC1,0xB0,0x68,0x1B,0xFE,0x7F,0x80, // 'u'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0x00,0x00,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'v'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x6F,0xF9,0xFE, // 'w'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0xB0,0x6C,0x1B,0x06,0xC1,0xA0,0x68,0x08,0x00, // 'x'
	0x00,0x20,0x2C,0x0B,0x06,0xC1,0xB0,0x6C,0x1A,0xFC,0x7E,0x80,0x60,0x18,0x06,0x01,0x80,0x67,0xF9,0xFE, // 'y'
	0xFF,0x1F,0xE0,0x08,0x06,0x01,0x80,0x60,0x18,0x00,0x00,0x30,0x0C,0x03,0x00,0xC0,0x20,0x0F,0xF1,0xFE, // 'z'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '{'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80, // '|'
	0xFA,0x28,0xA2,0x8A,0x28,0xA2,0x8A,0x2F,0x80 // '}'
};
const GFXglyph DSEG7_Modern_Bold_16Glyphs[] PROGMEM = {
// bitmapOffset, width, height, xAdvance, xOffset, yOffset
	  {     0,   2,   1,   4,    0,   -1 }, // ' '
	  {     1,   2,   1,  14,    0,   -1 }, // '!'
	  {     2,   6,  11,   7,    1,  -11 }, // '"'
	  {    11,   6,  11,   7,    1,  -11 }, // '#'
	  {    20,   6,  11,   7,    1,  -11 }, // '$'
	  {    29,   6,  11,   7,    1,  -11 }, // '%'
	  {    38,   6,  11,   7,    1,  -11 }, // '&'
	  {    47,   6,  11,   7,    1,  -11 }, // '''
	  {    56,   6,  11,   7,    1,  -11 }, // '('
	  {    65,   6,  11,   7,    1,  -11 }, // ')'
	  {    74,   6,  11,   7,    1,  -11 }, // '*'
	  {    83,   6,  11,   7,    1,  -11 }, // '+'
	  {    92,   6,  11,   7,    1,  -11 }, // ','
	  {   101,   8,   2,  14,    3,   -9 }, // '-'
	  {   103,   3,   2,   1,   -1,   -2 }, // '.'
	  {   104,   6,  11,   7,    1,  -11 }, // '/'
	  {   113,  10,  16,  14,    2,  -16 }, // '0'
	  {   133,   3,  15,  14,    9,  -16 }, // '1'
	  {   139,  10,  16,  14,    2,  -16 }, // '2'
	  {   159,  10,  16,  14,    2,  -16 }, // '3'
	  {   179,  10,  15,  14,    2,  -16 }, // '4'
	  {   198,  10,  16,  14,    2,  -16 }, // '5'
	  {   218,  10,  16,  14,    2,  -16 }, // '6'
	  {   238,  10,  15,  14,    2,  -16 }, // '7'
	  {   257,  10,  16,  14,    2,  -16 }, // '8'
	  {   277,  10,  16,  14,    2,  -16 }, // '9'
	  {   297,   3,   9,   4,    1,  -12 }, // ':'
	  {   301,   6,  11,   7,    1,  -11 }, // ';'
	  {   310,   6,  11,   7,    1,  -11 }, // '<'
	  {   319,   6,  11,   7,    1,  -11 }, // '='
	  {   328,   6,  11,   7,    1,  -11 }, // '>'
	  {   337,   6,  11,   7,    1,  -11 }, // '?'
	  {   346,   6,  11,   7,    1,  -11 }, // '@'
	  {   355,  10,  16,  14,    2,  -16 }, // 'A'
	  {   375,  10,  15,  14,    2,  -15 }, // 'B'
	  {   394,  10,   9,  14,    2,   -9 }, // 'C'
	  {   406,  10,  16,  14,    2,  -16 }, // 'D'
	  {   426,  10,  16,  14,    2,  -16 }, // 'E'
	  {   446,  10,  16,  14,    2,  -16 }, // 'F'
	  {   466,  10,  16,  14,    2,  -16 }, // 'G'
	  {   486,  10,  15,  14,    2,  -15 }, // 'H'
	  {   505,   3,   8,  14,    9,   -9 }, // 'I'
	  {   508,  10,  16,  14,    2,  -16 }, // 'J'
	  {   528,  10,  16,  14,    2,  -16 }, // 'K'
	  {   548,  10,  15,  14,    2,  -15 }, // 'L'
	  {   567,  10,  16,  14,    2,  -16 }, // 'M'
	  {   587,  10,   9,  14,    2,   -9 }, // 'N'
	  {   599,  10,   9,  14,    2,   -9 }, // 'O'
	  {   611,  10,  16,  14,    2,  -16 }, // 'P'
	  {   631,  10,  15,  14,    2,  -16 }, // 'Q'
	  {   650,   9,   9,  14,    2,   -9 }, // 'R'
	  {   661,  10,  15,  14,    2,  -15 }, // 'S'
	  {   680,  10,  15,  14,    2,  -15 }, // 'T'
	  {   699,  10,   9,  14,    2,   -9 }, // 'U'
	  {   711,  10,  16,  14,    2,  -16 }, // 'V'
	  {   731,  10,  16,  14,    2,  -16 }, // 'W'
	  {   751,  10,  16,  14,    2,  -16 }, // 'X'
	  {   771,  10,  16,  14,    2,  -16 }, // 'Y'
	  {   791,  10,  16,  14,    2,  -16 }, // 'Z'
	  {   811,   6,  11,   7,    1,  -11 }, // '['
	  {   820,   6,  11,   7,    1,  -11 }, // '\'
	  {   829,   6,  11,   7,    1,  -11 }, // ']'
	  {   838,   6,  11,   7,    1,  -11 }, // '^'
	  {   847,   6,  11,   7,    1,  -11 }, // '_'
	  {   856,   6,  11,   7,    1,  -11 }, // '`'
	  {   865,  10,  16,  14,    2,  -16 }, // 'a'
	  {   885,  10,  15,  14,    2,  -15 }, // 'b'
	  {   904,  10,   9,  14,    2,   -9 }, // 'c'
	  {   916,  10,  16,  14,    2,  -16 }, // 'd'
	  {   936,  10,  16,  14,    2,  -16 }, // 'e'
	  {   956,  10,  16,  14,    2,  -16 }, // 'f'
	  {   976,  10,  16,  14,    2,  -16 }, // 'g'
	  {   996,  10,  15,  14,    2,  -15 }, // 'h'
	  {  1015,   3,   8,  14,    9,   -9 }, // 'i'
	  {  1018,  10,  16,  14,    2,  -16 }, // 'j'
	  {  1038,  10,  16,  14,    2,  -16 }, // 'k'
	  {  1058,  10,  15,  14,    2,  -15 }, // 'l'
	  {  1077,  10,  16,  14,    2,  -16 }, // 'm'
	  {  1097,  10,   9,  14,    2,   -9 }, // 'n'
	  {  1109,  10,   9,  14,    2,   -9 }, // 'o'
	  {  1121,  10,  16,  14,    2,  -16 }, // 'p'
	  {  1141,  10,  15,  14,    2,  -16 }, // 'q'
	  {  1160,   9,   9,  14,    2,   -9 }, // 'r'
	  {  1171,  10,  15,  14,    2,  -15 }, // 's'
	  {  1190,  10,  15,  14,    2,  -15 }, // 't'
	  {  1209,  10,   9,  14,    2,   -9 }, // 'u'
	  {  1221,  10,  16,  14,    2,  -16 }, // 'v'
	  {  1241,  10,  16,  14,    2,  -16 }, // 'w'
	  {  1261,  10,  16,  14,    2,  -16 }, // 'x'
	  {  1281,  10,  16,  14,    2,  -16 }, // 'y'
	  {  1301,  10,  16,  14,    2,  -16 }, // 'z'
	  {  1321,   6,  11,   7,    1,  -11 }, // '{'
	  {  1330,   6,  11,   7,    1,  -11 }, // '|'
	  {  1339,   6,  11,   7,    1,  -11 } // '}'
};
const GFXfont DSEG7_Modern_Bold_16 PROGMEM = {
(uint8_t  *)DSEG7_Modern_Bold_16Bitmaps,(GFXglyph *)DSEG7_Modern_Bold_16Glyphs,0x20, 0x7E, 18};