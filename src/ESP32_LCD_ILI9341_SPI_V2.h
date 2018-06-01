/*
  ESP32_LCD_ILI9341_SPI_V2.h - for Arduino core for the ESP32 ( Use SPI library ).
  Beta version 1.00
  ESP32_LCD_ILI9341_SPI library class has been redesigned.
  
The MIT License (MIT)

Copyright (c) 2018 Mgo-tec. All rights reserved.
Blog URL ---> https://www.mgo-tec.com

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

Modify Display.cpp of M5stack library.
M5stack library - MIT License
Copyright (c) 2017 M5Stack
*/

#ifndef _ESP32_LCD_ILI9341_SPI_V2_H_INCLUDED
#define _ESP32_LCD_ILI9341_SPI_V2_H_INCLUDED

#include <Arduino.h>
#include <SPI.h>
#include "soc/spi_reg.h"

using namespace std;

//********************************************************************
class FONT_PARAM
{
public:
  FONT_PARAM(){
    Xsize = 1;
    Ysize = 1;
    PrevXsize = 1;
    PrevYsize = 1;
    x0 = 0;
    y0 = 0;
    red = 31;
    green = 63;
    blue = 31;
    bg_red = 0;
    bg_green = 0;
    bg_blue = 0;
    //Serial.println(F("---------- FONT_PARAM Constructor called! -----------"));
  };

  ~FONT_PARAM(){
    //Serial.println(F("---------- FONT_PARAM Destructor called! -----------"));
  };

  uint8_t Xsize, Ysize;
  uint8_t PrevXsize, PrevYsize;
  int16_t x0, y0;
  uint8_t red, green, blue;
  uint8_t bg_red, bg_green, bg_blue;

  uint8_t Dot_MSB = 0, Dot_LSB = 0;
  uint8_t BG_Dot_MSB = 0, BG_Dot_LSB = 0;

  FONT_PARAM& Set_Xsize( uint8_t value ) { Xsize = value; return *this; };
  FONT_PARAM& Set_Ysize( uint8_t value ) { Ysize = value; return *this; };
  FONT_PARAM& Set_x0( uint8_t value ) { x0 = value; return *this; };
  FONT_PARAM& Set_y0( uint8_t value ) { y0 = value; return *this; };
  FONT_PARAM& Set_red( uint8_t value ) { red = value; return *this; };
  FONT_PARAM& Set_green( uint8_t value ) { green = value; return *this; };
  FONT_PARAM& Set_blue( uint8_t value ) { blue = value; return *this; };
  FONT_PARAM& Set_bg_red( uint8_t value ) { bg_red = value; return *this; };
  FONT_PARAM& Set_bg_green( uint8_t value ) { bg_green = value; return *this; };
  FONT_PARAM& Set_bg_blue( uint8_t value ) { bg_blue = value; return *this; };
};

//*******************************************************************************
class ScrolleSET_PARAM
{
private:

public:
  ScrolleSET_PARAM(){
    //Serial.println(F("---------- ScrolleSET_PARAM Constructor called! -----------"));
  };

  ~ScrolleSET_PARAM(){
    //Serial.println(F("---------- ScrolleSET_PARAM Destructor called! -----------"));
  };

  uint16_t Max_Disp_x0_X, Max_Disp_y0_Y;

  int16_t SclSpeed = 0;
  uint16_t FontCount = 0;
  uint8_t Zen_or_Han = 0;
  uint16_t ZenOrHan_cnt = 0;
  //uint16_t m_scl_X_cnt = 0;
  uint16_t scl_Y_cnt = 0;
  uint16_t SclPixSizeX,  SclPixSizeY;
  uint16_t FontSJ_Length = 0;
  uint8_t txt_width = 0, txt_height = 0;

  uint32_t SclLastTime = 0;
  uint8_t SingleFontSclCount = 0;
  uint16_t XsclSendBytesNum = {};
  uint16_t YsclSendBytesNum = {};
  //------------------------------------
  uint8_t **Scl_Array;
  uint16_t Scl_ArraySize1, Scl_ArraySize2;

  bool isHeap_Create = false;
};

//*******************************************************************
class ESP32_LCD_ILI9341_SPI_V2
{
private:
  const uint8_t _SPI_NUM = 0x3; //VSPI=0x3, HSPI=0x2

  #define SWAP(type, x, y) do { type tmp = x; x = y; y = tmp; } while (0)

  int8_t m_sck;
  int8_t m_miso;
  int8_t m_mosi;
  int8_t m_rst;
  int8_t m_ledpin;
  int8_t m_dc;
  int8_t m_cs;
  bool m_useHw_Cs;
  uint32_t m_Freq;

  uint16_t m_MAX_DISP_WIDTH_X = 320;
  uint16_t m_MAX_DISP_HEIGHT_Y = 240;
  uint8_t m_FNT8x16_X_MAX = m_MAX_DISP_WIDTH_X / 8;
  uint8_t m_FNT8x16_Y_MAX = m_MAX_DISP_HEIGHT_Y / 16;
  uint16_t m_Max_Pix_X = m_MAX_DISP_WIDTH_X - 1;
  uint16_t m_Max_Pix_Y = m_MAX_DISP_HEIGHT_Y - 1;

  void CommandWrite(uint8_t b);
  void DataWrite(uint8_t b);
  void DataWrite16(uint16_t b);
  void DataWrite32(uint32_t b);
  void DataWriteBytes(uint8_t *b, uint32_t b_size);
  void SPI_set_change();
  void spiWriteBlock(uint16_t color, uint32_t repeat);
  void XY_Range(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void Block_SPI_Fast_Write(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue, uint32_t repeat);
  uint8_t Array2_New_Create( ScrolleSET_PARAM &scl_set, uint16_t ary_size1, uint16_t ary_size2 );

  void Scrolle_Form_Set(FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint8_t txt_width, uint8_t txt_height, uint8_t Xsize, uint8_t Ysize);
  void Font_Param_MaxClip( FONT_PARAM &font, ScrolleSET_PARAM &scl_set );

public:
  void ILI9341_Init(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t led, uint32_t clk, bool use_hwcs);
  void Disp_Rotation(uint8_t rot);
  void Display_Clear();
  void Display_Clear(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void Draw_Pixel_65k_DotColor_sd(uint16_t x0, uint16_t y0, uint16_t DotColor);
  void Draw_Pixel_65k_DotColor(uint16_t x0, uint16_t y0, uint16_t DotColor);
  void Draw_Pixel_65k_3Color_sd(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Pixel_65k_3Color(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Horizontal_Line(int16_t x0, int16_t x1, int16_t y0, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Vertical_Line(int16_t x0, int16_t y0, int16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Rectangle_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Rectangle_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Circle_Line(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Circle_Fill(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue);
  //void Idle_mode_OFF();
  //void Idle_mode_ON();
  void Brightness(uint8_t brightness);

  //-------------以下フォント系関数----------------------------
  uint8_t X_Scrolle_Font_2Array_Init( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_width, uint8_t Xsize, uint8_t Ysize );
  uint8_t Y_Scrolle_Font_2Array_Init( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_height, uint8_t Xsize, uint8_t Ysize );
  uint8_t X_Scrolle_Font_2Array_Init_MAX( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_width, uint8_t Xsize, uint8_t Ysize );
  uint8_t Y_Scrolle_Font_2Array_Init_MAX( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_height, uint8_t Xsize, uint8_t Ysize );
  void Array2_Delete( ScrolleSET_PARAM &scl_set );

  void Scrolle_Font_SetUp( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, bool xy_range_set = false ); 
  void Scrolle_Font_Color_Set( FONT_PARAM &font );
  void Scrolle_XYrange_Com_Change( FONT_PARAM &font, ScrolleSET_PARAM &scl_set );

  void Scrolle_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t *fnt_cnt, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set = true );
  boolean Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set = true );
  void Rev_Scrolle_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t *fnt_cnt, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set = true );
  boolean Rev_Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set = true );
  boolean Y_Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set = true );
  boolean Ydown_Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set = true );

  void XYsizeUp_8x16_Font_DisplayOut( FONT_PARAM &font, uint16_t fontSJ_Length, uint8_t Fnt[][16] );
};

#endif
