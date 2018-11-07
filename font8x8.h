/*
File	: font5x7.h
Version	: 1.0
Date	: 14.01.2015
Project	: myMatrix Arduino Library

The MIT License (MIT)

Copyright (c) 2015 Silviu - www.openhardware.ro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef FONT8X8_H
#define FONT8X8_H

//static const uint8_t font8x8[96][8] =
static const uint8_t font8x8[96][8] PROGMEM =
{                                      
{ B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000 }, // Space
{ B00110000, B01111000, B01111000, B00110000, B00110000, B00000000, B00110000, B00000000 }, // !
{ B01101100, B01101100, B01101100, B00000000, B00000000, B00000000, B00000000, B00000000 }, // "
{ B01101100, B01101100, B11111110, B01101100, B11111110, B01101100, B01101100, B00000000 }, // #
{ B00110000, B01111100, B11000000, B01111000, B00001100, B11111000, B00110000, B00000000 }, // $
{ B00000000, B11000110, B11001100, B00011000, B00110000, B01100110, B11000110, B00000000 }, // %
{ B00111000, B01101100, B00111000, B01110110, B11011100, B11001100, B01110110, B00000000 }, // &
{ B01100000, B01100000, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000 }, // '
{ B00011000, B00110000, B01100000, B01100000, B01100000, B00110000, B00011000, B00000000 }, // (
{ B01100000, B00110000, B00011000, B00011000, B00011000, B00110000, B01100000, B00000000 }, // )
{ B00000000, B01100110, B00111100, B11111111, B00111100, B01100110, B00000000, B00000000 }, // *
{ B00000000, B00110000, B00110000, B11111100, B00110000, B00110000, B00000000, B00000000 }, // +
{ B00000000, B00000000, B00000000, B00000000, B00000000, B01110000, B00110000, B01100000 }, // ,
{ B00000000, B00000000, B00000000, B11111100, B00000000, B00000000, B00000000, B00000000 }, // -
{ B00000000, B00000000, B00000000, B00000000, B00000000, B00110000, B00110000, B00000000 }, // .
{ B00000110, B00001100, B00011000, B00110000, B01100000, B11000000, B10000000, B00000000 }, // /
{ B01111000, B11001100, B11011100, B11111100, B11101100, B11001100, B01111000, B00000000 }, // 0
{ B00110000, B11110000, B00110000, B00110000, B00110000, B00110000, B11111100, B00000000 }, // 1
{ B01111000, B11001100, B00001100, B00111000, B01100000, B11001100, B11111100, B00000000 }, // 2
{ B01111000, B11001100, B00001100, B00111000, B00001100, B11001100, B01111000, B00000000 }, // 3
{ B00011100, B00111100, B01101100, B11001100, B11111110, B00001100, B00001100, B00000000 }, // 4
{ B11111100, B11000000, B11111000, B00001100, B00001100, B11001100, B01111000, B00000000 }, // 5
{ B00111000, B01100000, B11000000, B11111000, B11001100, B11001100, B01111000, B00000000 }, // 6
{ B11111100, B11001100, B00001100, B00011000, B00110000, B01100000, B01100000, B00000000 }, // 7
{ B01111000, B11001100, B11001100, B01111000, B11001100, B11001100, B01111000, B00000000 }, // 8
{ B01111000, B11001100, B11001100, B01111100, B00001100, B00011000, B01110000, B00000000 }, // 9
{ B00000000, B00000000, B00110000, B00110000, B00000000, B00110000, B00110000, B00000000 }, // :
{ B00000000, B00000000, B00110000, B00110000, B00000000, B01110000, B00110000, B01100000 }, // ;
{ B00011000, B00110000, B01100000, B11000000, B01100000, B00110000, B00011000, B00000000 }, // <
{ B00000000, B00000000, B11111100, B00000000, B11111100, B00000000, B00000000, B00000000 }, // =
{ B01100000, B00110000, B00011000, B00001100, B00011000, B00110000, B01100000, B00000000 }, // >
{ B01111000, B11001100, B00001100, B00011000, B00110000, B00000000, B00110000, B00000000 }, // ?
{ B01111100, B11000110, B11011110, B11011110, B11011110, B11000000, B01111000, B00000000 }, // @
{ B00110000, B01111000, B11001100, B11001100, B11111100, B11001100, B11001100, B00000000 }, // A
{ B11111100, B01100110, B01100110, B01111100, B01100110, B01100110, B11111100, B00000000 }, // B
{ B00111100, B01100110, B11000000, B11000000, B11000000, B01100110, B00111100, B00000000 }, // C
{ B11111100, B01101100, B01100110, B01100110, B01100110, B01101100, B11111100, B00000000 }, // D
{ B11111110, B01100010, B01101000, B01111000, B01101000, B01100010, B11111110, B00000000 }, // E
{ B11111110, B01100010, B01101000, B01111000, B01101000, B01100000, B11110000, B00000000 }, // F
{ B00111100, B01100110, B11000000, B11000000, B11001110, B01100110, B00111110, B00000000 }, // G
{ B11001100, B11001100, B11001100, B11111100, B11001100, B11001100, B11001100, B00000000 }, // H
{ B01111000, B00110000, B00110000, B00110000, B00110000, B00110000, B01111000, B00000000 }, // I
{ B00011110, B00001100, B00001100, B00001100, B11001100, B11001100, B01111000, B00000000 }, // J
{ B11100110, B01100110, B01101100, B01111000, B01101100, B01100110, B11100110, B00000000 }, // K
{ B11110000, B01100000, B01100000, B01100000, B01100010, B01100110, B11111110, B00000000 }, // L
{ B11000110, B11101110, B11111110, B11010110, B11000110, B11000110, B11000110, B00000000 }, // M
{ B11000110, B11100110, B11110110, B11011110, B11001110, B11000110, B11000110, B00000000 }, // N
{ B00111000, B01101100, B11000110, B11000110, B11000110, B01101100, B00111000, B00000000 }, // O
{ B11111100, B01100110, B01100110, B01111100, B01100000, B01100000, B11110000, B00000000 }, // P
{ B01111000, B11001100, B11001100, B11001100, B11011100, B01111000, B00011100, B00000000 }, // Q
{ B11111100, B01100110, B01100110, B01111100, B01111000, B01101100, B11100110, B00000000 }, // R
{ B01111000, B11001100, B11100000, B00111000, B00011100, B11001100, B01111000, B00000000 }, // S
{ B11111100, B10110100, B00110000, B00110000, B00110000, B00110000, B01111000, B00000000 }, // T
{ B11001100, B11001100, B11001100, B11001100, B11001100, B11001100, B11111100, B00000000 }, // U
{ B11001100, B11001100, B11001100, B11001100, B11001100, B01111000, B00110000, B00000000 }, // V
{ B11000110, B11000110, B11000110, B11010110, B11111110, B11101110, B11000110, B00000000 }, // W
{ B11000110, B11000110, B01101100, B00111000, B01101100, B11000110, B11000110, B00000000 }, // X
{ B11001100, B11001100, B11001100, B01111000, B00110000, B00110000, B01111000, B00000000 }, // Y
{ B11111110, B11001100, B10011000, B00110000, B01100010, B11000110, B11111110, B00000000 }, // Z
{ B01111000, B01100000, B01100000, B01100000, B01100000, B01100000, B01111000, B00000000 }, // [
{ B11000000, B01100000, B00110000, B00011000, B00001100, B00000110, B00000010, B00000000 }, // "\"
{ B01111000, B00011000, B00011000, B00011000, B00011000, B00011000, B01111000, B00000000 }, // ]
{ B00010000, B00111000, B01101100, B11000110, B00000000, B00000000, B00000000, B00000000 }, // ^
{ B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B11111111 }, // _
{ B00110000, B00110000, B00011000, B00000000, B00000000, B00000000, B00000000, B00000000 }, // `
{ B00000000, B00000000, B01111000, B00001100, B01111100, B11001100, B01110110, B00000000 }, // a 
{ B11100000, B01100000, B01111100, B01100110, B01100110, B01100110, B10111100, B00000000 }, // b
{ B00000000, B00000000, B01111000, B11001100, B11000000, B11001100, B01111000, B00000000 }, // c
{ B00011100, B00001100, B00001100, B01111100, B11001100, B11001100, B01110110, B00000000 }, // d
{ B00000000, B00000000, B01111000, B11001100, B11111100, B11000000, B01111000, B00000000 }, // e
{ B00111000, B01101100, B01100000, B11110000, B01100000, B01100000, B11110000, B00000000 }, // f
{ B00000000, B00000000, B01110110, B11001100, B11001100, B01111100, B00001100, B11111000 }, // g
{ B11100000, B01100000, B01101100, B01110110, B01100110, B01100110, B11100110, B00000000 }, // h
{ B00110000, B00000000, B01110000, B00110000, B00110000, B00110000, B01111000, B00000000 }, // i
{ B00011000, B00000000, B01111000, B00011000, B00011000, B00011000, B11011000, B01110000 }, // j
{ B11100000, B01100000, B01100110, B01101100, B01111000, B01101100, B11100110, B00000000 }, // k
{ B01110000, B00110000, B00110000, B00110000, B00110000, B00110000, B01111000, B00000000 }, // l
{ B00000000, B00000000, B11101100, B11111110, B11010110, B11000110, B11000110, B00000000 }, // m
{ B00000000, B00000000, B11111000, B11001100, B11001100, B11001100, B11001100, B00000000 }, // n
{ B00000000, B00000000, B01111000, B11001100, B11001100, B11001100, B01111000, B00000000 }, // o
{ B00000000, B00000000, B11011100, B01100110, B01100110, B01111100, B01100000, B11110000 }, // p
{ B00000000, B00000000, B01110110, B11001100, B11001100, B01111100, B00001100, B00011110 }, // q
{ B00000000, B00000000, B11011000, B01101100, B01101100, B01100000, B11110000, B00000000 }, // r
{ B00000000, B00000000, B01111100, B11000000, B01111000, B00001100, B11111000, B00000000 }, // s
{ B00010000, B00110000, B01111100, B00110000, B00110000, B00110100, B00011000, B00000000 }, // t
{ B00000000, B00000000, B11001100, B11001100, B11001100, B11001100, B01110110, B00000000 }, // u
{ B00000000, B00000000, B11001100, B11001100, B11001100, B01111000, B00110000, B00000000 }, // v
{ B00000000, B00000000, B11000110, B11000110, B11010110, B11111110, B01101100, B00000000 }, // w
{ B00000000, B00000000, B11000110, B01101100, B00111000, B01101100, B11000110, B00000000 }, // x
{ B00000000, B00000000, B11001100, B11001100, B11001100, B01111100, B00001100, B11111000 }, // y
{ B00000000, B00000000, B11111100, B10011000, B00110000, B01100100, B11111100, B00000000 }, // z
{ B00011100, B00110000, B00110000, B11100000, B00110000, B00110000, B00011100, B00000000 }, // {
{ B00011000, B00011000, B00011000, B00000000, B00011000, B00011000, B00011000, B00000000 }, // |
{ B11100000, B00110000, B00110000, B00011100, B00110000, B00110000, B11100000, B00000000 }, // }
{ B01110110, B11011100, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000 }, // ~
{ B00010000, B00111000, B01101100, B11000110, B11000110, B11000110, B11111110, B00000000 }, // unfinished
};

#endif
