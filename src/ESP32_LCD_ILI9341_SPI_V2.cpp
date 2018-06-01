/*
  ESP32_LCD_ILI9341_SPI_V2.cpp - for Arduino core for the ESP32 ( Use SPI library ).
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

#include "ESP32_LCD_ILI9341_SPI_V2.h"

//****** LCD ILI9341 ディスプレイ初期化 ***********
void ESP32_LCD_ILI9341_SPI_V2::ILI9341_Init(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t led, uint32_t clk, bool use_hwcs){
  ESP32_LCD_ILI9341_SPI_V2::Brightness(0);

  m_sck = sck;
  m_miso = miso;
  m_mosi = mosi;
  m_cs = cs;
  m_dc = dc;
  m_rst = rst;
  m_ledpin = led;

  m_useHw_Cs = use_hwcs; //VSPI Hardware CS 
  m_Freq = clk;

  pinMode(m_rst, OUTPUT); //Set RESET pin
  pinMode(m_dc, OUTPUT); //Set Data/Command pin

  SPI.begin(m_sck, m_miso, m_mosi, m_cs); //VSPI setting

  SPI.setBitOrder(MSBFIRST);
  //ILI9341 のSPI Clock Cycle Time (Write) Minimun 100ns=10MHz
  SPI.setFrequency(clk);
  SPI.setDataMode(SPI_MODE0);
  SPI.setHwCs(m_useHw_Cs); //Set Hardware CS pin

  //Hardware Reset------------
  digitalWrite(m_rst, HIGH);
  delay(5);
  digitalWrite(m_rst, LOW);
  delay(10);
  digitalWrite(m_rst, HIGH);
  delay(121);

  if(!m_useHw_Cs){
    pinMode(m_cs, OUTPUT);
    digitalWrite(m_cs, HIGH);
    digitalWrite(m_cs, LOW);
  }
  digitalWrite(m_dc, HIGH);

  ESP32_LCD_ILI9341_SPI_V2::CommandWrite(0x38); //Idle mode OFF
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite(0x3A); //COLMOD: Pixel Format Set
    ESP32_LCD_ILI9341_SPI_V2::DataWrite(0b01010101); //RGB 16 bits / pixel, MCU 16 bits / pixel
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite(0x36); //MADCTL: Memory Access Control
    ESP32_LCD_ILI9341_SPI_V2::DataWrite(0b0001000); //M5stack only. D3: BGR(RGB-BGR Order control bit )="1"
    //ESP32_LCD_ILI9341_SPI_V2::DataWrite(0b00101000); //サインスマート 2.2インチ用。D3: BGR(RGB-BGR Order control bit )="1"
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite(0x11); //Sleep OUT
  delay(10);

  ESP32_LCD_ILI9341_SPI_V2::CommandWrite(0x29); //Display ON

  ESP32_LCD_ILI9341_SPI_V2::Display_Clear(0, 0, 319, 239);

  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
  ESP32_LCD_ILI9341_SPI_V2::Brightness(100);
}
//********* Display Rotation ***************************
void ESP32_LCD_ILI9341_SPI_V2::Disp_Rotation(uint8_t rot){
  uint8_t b = 0b00001000;
  switch( rot ){
    case 0: //M5stack 横表示、デフォルト
      b = 0b00001000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 1: //M5stack 縦表示、デフォルト
      b = 0b10101000;
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    case 2: //M5stack 横表示、上下逆
      b = 0b11001000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 3: //M5stack 縦表示、上下逆
      b = 0b01101000;
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    //------------------------
    case 4: //M5stack 横表示、左右反転
      b = 0b01001000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 5: //M5stack 縦表示、左右反転
      b = 0b00101000;
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    case 6: //M5stack 横表示、上下逆、左右反転
      b = 0b10001000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 7: //M5stack 縦表示、上下逆、左右反転
      b = 0b11101000;
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    //-------------------------------
    case 8: //M5stack 横表示、デフォルト、上下反転
      b = 0b10001000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 9: //M5stack 縦表示、デフォルト、上下反転
      b = 0b11101000;
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    case 10: //M5stack 横表示、上下逆、上下反転
      b = 0b01001000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 11: //M5stack 縦表示、上下逆、上下反転
      b = 0b00101000;
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    //-------------------------------
    case 12: //M5stack で表示変わらず
      b = 0b00001100;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 13: //M5stack で表示変わらず
      b = 0b00011000;
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;

    //------------------------
    case 250:
      b = 0b00101000; //サインスマート販売のILI9341横正常表示(M5stack 縦pixelサイズと横pixelサイズが異なるのでノイズ)
      m_MAX_DISP_WIDTH_X = 320;
      m_MAX_DISP_HEIGHT_Y = 240;
      break;
    case 251:
      b = 0b10001000; //サインスマート販売のILI9341の縦方向表示
      m_MAX_DISP_WIDTH_X = 240;
      m_MAX_DISP_HEIGHT_Y = 320;
      break;
    default:
      break;
  }
  m_Max_Pix_X = m_MAX_DISP_WIDTH_X - 1;
  m_Max_Pix_Y = m_MAX_DISP_HEIGHT_Y - 1;
  m_FNT8x16_X_MAX = m_MAX_DISP_WIDTH_X / 8;
  m_FNT8x16_Y_MAX = m_MAX_DISP_HEIGHT_Y / 16;
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite(0x36); //MADCTL: Memory Access Control
    ESP32_LCD_ILI9341_SPI_V2::DataWrite(b); //M5stack only. D3: BGR(RGB-BGR Order control bit )="1"
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//********* 4wire SPI Data / Command write************
void ESP32_LCD_ILI9341_SPI_V2::CommandWrite(uint8_t b){
  digitalWrite(m_dc, LOW);
  SPI.write(b);
  digitalWrite(m_dc, HIGH);
}

void ESP32_LCD_ILI9341_SPI_V2::DataWrite(uint8_t b){
  SPI.write(b);
}

void ESP32_LCD_ILI9341_SPI_V2::DataWrite16(uint16_t b){
  SPI.write16(b);
}

void ESP32_LCD_ILI9341_SPI_V2::DataWrite32(uint32_t b){
  SPI.write32(b);
}

void ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes(uint8_t *b, uint32_t b_size){
  SPI.writeBytes(b, b_size);
}
//**************************************************
void ESP32_LCD_ILI9341_SPI_V2::SPI_set_change(){
  SPI.setFrequency(m_Freq);
  SPI.setDataMode(SPI_MODE0);
  if(!m_useHw_Cs) digitalWrite(m_cs, LOW);
}
//******** Set Column and Page Address ( X Y range setting )***********
void ESP32_LCD_ILI9341_SPI_V2::XY_Range(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  uint32_t X = (uint32_t)x0<<16 | x1;
  uint32_t Y = (uint32_t)y0<<16 | y1;

  ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2A ); //Set Column Address
    ESP32_LCD_ILI9341_SPI_V2::DataWrite32(X);
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2B ); //Set Page Address
    ESP32_LCD_ILI9341_SPI_V2::DataWrite32(Y);
}
//********* Display All Black Clear ******************************
void ESP32_LCD_ILI9341_SPI_V2::Display_Clear(){
  ESP32_LCD_ILI9341_SPI_V2::Display_Clear(0, 0, m_Max_Pix_X, m_Max_Pix_Y);
}
//********* Display All Black Clear ******************************
void ESP32_LCD_ILI9341_SPI_V2::Display_Clear(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  if( x0 > m_Max_Pix_X ) x0 = m_Max_Pix_X;
  if( y0 > m_Max_Pix_Y ) y0 = m_Max_Pix_Y;
  if( x1 > m_Max_Pix_X ) x1 = m_Max_Pix_X;
  if( y1 > m_Max_Pix_Y ) y1 = m_Max_Pix_Y;
  uint32_t Total_Pixels = (x1 - x0 + 1) * (y1 - y0 + 1) ;
  ESP32_LCD_ILI9341_SPI_V2::Block_SPI_Fast_Write(x0, y0, x1, y1, 0, 0, 0, Total_Pixels);
}
//*********** LCD ILE9341 Block Pixel SPI Fast Write *****************
void ESP32_LCD_ILI9341_SPI_V2::Block_SPI_Fast_Write(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue, uint32_t repeat){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  uint16_t ColorDot = (red << 11) | (green << 5) | blue;
  ESP32_LCD_ILI9341_SPI_V2::XY_Range(x0, y0, x1, y1);
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //LCD RAM write
  ESP32_LCD_ILI9341_SPI_V2::spiWriteBlock(ColorDot, repeat);
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//********* Display Color Pixel Block Fast Write *****************
void ESP32_LCD_ILI9341_SPI_V2::spiWriteBlock(uint16_t color, uint32_t repeat) {
  uint16_t color16 = (color >> 8) | (color << 8);
  uint32_t color32 = color16 | color16 << 16;

  if (repeat > 15) {
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(_SPI_NUM), SPI_USR_MOSI_DBITLEN, 255,
                      SPI_USR_MOSI_DBITLEN_S);

    while (repeat > 15) {
      while (READ_PERI_REG(SPI_CMD_REG(_SPI_NUM)) & SPI_USR)
        ;
      for (uint32_t i = 0; i < 16; i++)
        WRITE_PERI_REG((SPI_W0_REG(_SPI_NUM) + (i << 2)), color32);
      SET_PERI_REG_MASK(SPI_CMD_REG(_SPI_NUM), SPI_USR);
      repeat -= 16;
    }
    while (READ_PERI_REG(SPI_CMD_REG(_SPI_NUM)) & SPI_USR)
      ;
  }

  if (repeat) {
    repeat = (repeat << 4) - 1;
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(_SPI_NUM), SPI_USR_MOSI_DBITLEN, repeat,
                      SPI_USR_MOSI_DBITLEN_S);
    for (uint32_t i = 0; i < 16; i++)
      WRITE_PERI_REG((SPI_W0_REG(_SPI_NUM) + (i << 2)), color32);
    SET_PERI_REG_MASK(SPI_CMD_REG(_SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(_SPI_NUM)) & SPI_USR)
      ;
  }
}
//*********** 65k Color Pixel (Dot) Write ( SD card use )*************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor_sd(uint16_t x0, uint16_t y0, uint16_t DotColor){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(x0, y0, DotColor);
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//*********** 65k Color Pixel (Dot) Write ****************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(uint16_t x0, uint16_t y0, uint16_t DotColor){
  ESP32_LCD_ILI9341_SPI_V2::XY_Range(x0, y0, x0, y0);
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
  ESP32_LCD_ILI9341_SPI_V2::DataWrite16( DotColor );
}
//*********** 65k Pixel RGB color Write ( SD card use )****************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_3Color_sd(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_3Color(x0, y0, red, green, blue);
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//*********** 65k Pixel RGB color Write ****************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_3Color(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  //red (0-31), green (0-63), blue (0-31)
  ESP32_LCD_ILI9341_SPI_V2::XY_Range(x0, y0, x0, y0);

  uint16_t Dot = ((uint16_t)red << 11) | ((uint16_t)green << 5) | (uint16_t)blue;

  ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
  ESP32_LCD_ILI9341_SPI_V2::DataWrite16( Dot );
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Rectangle_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  ESP32_LCD_ILI9341_SPI_V2::Draw_Horizontal_Line(x0, x1, y0, red, green, blue);
  ESP32_LCD_ILI9341_SPI_V2::Draw_Horizontal_Line(x0, x1, y1, red, green, blue);
  ESP32_LCD_ILI9341_SPI_V2::Draw_Vertical_Line(x0, y0, y1, red, green, blue);
  ESP32_LCD_ILI9341_SPI_V2::Draw_Vertical_Line(x1, y0, y1, red, green, blue);
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Horizontal_Line(int16_t x0, int16_t x1, int16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  if(x1 < x0) SWAP(int16_t, x0, x1);
  uint32_t Width_x = x1 - x0 + 1;
  ESP32_LCD_ILI9341_SPI_V2::Block_SPI_Fast_Write(x0, y0, x1, y0, red, green, blue, Width_x);
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Vertical_Line(int16_t x0, int16_t y0, int16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  if(y1 < y0) SWAP(int16_t, y0, y1);
  uint16_t Width_y = y1 - y0 + 1;
  ESP32_LCD_ILI9341_SPI_V2::Block_SPI_Fast_Write(x0, y0, x0, y1, red, green, blue, Width_y);
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  int i;
  int16_t Y = 0, X = 0;
  int16_t length_x = x1 - x0;
  int16_t length_y = y1 - y0;

  uint16_t Dot = (red << 11) | (green << 5) | blue;

  if(abs(length_x) > abs(length_y)){
    float degY = ((float)length_y) / ((float)length_x);
    if(x0 < x1){
      for(i=x0; i<(x1+1); i++){
        Y = y0 + round((i-x0) * degY);
        ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(i, Y, Dot);
      }
    }else{
      for(i=x0; i>=x1; i--){
        Y = y0 + round((i-x0) * degY);
        ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(i, Y, Dot);
      }
    }
  }else{
    float degX = ((float)length_x) / ((float)length_y);

    if(y0 < y1){
      for(i=y0; i<(y1+1); i++){
        X = x0 + round((i-y0) * degX);
        ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(X, i, Dot);
      }
    }else{
      for(i=y0; i>=y1; i--){
        X = x0 + round((i-y0) * degX);
        ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(X, i, Dot);
      }
    }
  }
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Rectangle_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  uint16_t Width_x = x1 - x0 + 1;
  uint16_t Width_y = y1 - y0 + 1;
  uint32_t Total = Width_x * Width_y ;
  ESP32_LCD_ILI9341_SPI_V2::Block_SPI_Fast_Write(x0, y0, x1, y1, red, green, blue, Total);
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Circle_Line(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  uint16_t x1, y1;
  float i;
  float deg = 1.0;
  if( r > 50 ) deg = 0.5;
  if( r > 110) deg = 0.25;

  uint16_t Dot = ((uint16_t)red << 11) | ((uint16_t)green << 5) | (uint16_t)blue;
  int16_t dummy_x = -1, dummy_y = -1;

  for(i=0; i<360; i=i+deg){
    x1 = round( (float)(x0 + (r * cos(radians(i)))) );
    y1 = round( (float)(y0 + (r * sin(radians(i)))) );

    if((dummy_x != x1) || (dummy_y != y1)){
      ESP32_LCD_ILI9341_SPI_V2::Draw_Pixel_65k_DotColor(x1, y1, Dot);
      dummy_x = x1;
      dummy_y = y1;
    }
  }
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//***************************************
void ESP32_LCD_ILI9341_SPI_V2::Draw_Circle_Fill(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue){
  //red (0-31), green (0-63), blue (0-31)
  uint16_t x1, y1;
  float i;
  float deg = 1.0;
  //半径が大きくなると、角度の刻み方を細かくしないと、完全に塗りつぶせないので注意。
  if( r > 50 ) deg = 0.5;
  if( r > 110) deg = 0.25;

  int16_t dummy_x = -1, dummy_y = -1;
  for( i = 0; i < 360; i = i + deg ){
    x1 = round( (float)(x0 + (r * cos(radians(i)))) );
    y1 = round( (float)(y0 + (r * sin(radians(i)))) );
    if((dummy_x != x1) || (dummy_y != y1)){
      ESP32_LCD_ILI9341_SPI_V2::Draw_Vertical_Line(x1, y0, y1, red, green, blue);
      dummy_x = x1;
      dummy_y = y1;
    }
  }
}
//********* LCD Display LED Brightness **************
void ESP32_LCD_ILI9341_SPI_V2::Brightness(uint8_t brightness){
  uint8_t ledc_ch = 0;
  uint32_t valueMax = 255;
  uint32_t duty = (8191 / valueMax) * brightness;
  ledcSetup(ledc_ch, 5000, 13);
  ledcAttachPin(m_ledpin, 0);
  ledcWrite(ledc_ch, duty);
}
//************************************************
//******** 以下、フォント関数 ********************
//************************************************
uint8_t ESP32_LCD_ILI9341_SPI_V2::X_Scrolle_Font_2Array_Init( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_width, uint8_t Xsize, uint8_t Ysize){
  scl_set.txt_width = txt_width;
  scl_set.txt_height = 1;
  font.Xsize = Xsize;
  font.Ysize = Ysize;
  font.PrevXsize = Xsize;
  font.PrevYsize = Ysize;

  ESP32_LCD_ILI9341_SPI_V2::Font_Param_MaxClip( font, scl_set );  
  scl_set.Scl_ArraySize1 = scl_set.txt_width * 8 * font.Xsize * 2;
  scl_set.Scl_ArraySize2 = 16;
  Serial.printf("X.Scl_ArraySize1=%d, ary_size2=%d\r\n", scl_set.Scl_ArraySize1, scl_set.Scl_ArraySize2);

  return ESP32_LCD_ILI9341_SPI_V2::Array2_New_Create( scl_set, scl_set.Scl_ArraySize1, scl_set.Scl_ArraySize2 );
}
//********************************************************************
uint8_t ESP32_LCD_ILI9341_SPI_V2::X_Scrolle_Font_2Array_Init_MAX( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_width, uint8_t Xsize, uint8_t Ysize){
  scl_set.txt_width = txt_width; 
  scl_set.txt_height = 1;
  font.Xsize = Xsize;
  font.Ysize = Ysize;
  font.PrevXsize = Xsize;
  font.PrevYsize = Ysize;

  ESP32_LCD_ILI9341_SPI_V2::Font_Param_MaxClip( font, scl_set );
  txt_width = 40; //最大配列サイズにする
  scl_set.Scl_ArraySize1 = txt_width * 8 * font.Xsize * 2;
  scl_set.Scl_ArraySize2 = 16;

  return ESP32_LCD_ILI9341_SPI_V2::Array2_New_Create( scl_set, scl_set.Scl_ArraySize1, scl_set.Scl_ArraySize2 );
}
//********************************************************************
uint8_t ESP32_LCD_ILI9341_SPI_V2::Y_Scrolle_Font_2Array_Init( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_height, uint8_t Xsize, uint8_t Ysize){
  scl_set.txt_width = 2;
  scl_set.txt_height = txt_height;
  font.Xsize = Xsize;
  font.Ysize = Ysize;
  font.PrevXsize = Xsize;
  font.PrevYsize = Ysize;
  //txt_heightは少数を切り上げて多めに領域確保

  ESP32_LCD_ILI9341_SPI_V2::Font_Param_MaxClip( font, scl_set );
  scl_set.Scl_ArraySize1 = 16 * font.Xsize * 2;
  scl_set.Scl_ArraySize2 = 16 * scl_set.txt_height;
  Serial.printf("Y.Scl_ArraySize1=%d, ary_size2=%d\r\n", scl_set.Scl_ArraySize1, scl_set.Scl_ArraySize2);

  return ESP32_LCD_ILI9341_SPI_V2::Array2_New_Create( scl_set, scl_set.Scl_ArraySize1, scl_set.Scl_ArraySize2 );
}
//********************************************************************
uint8_t ESP32_LCD_ILI9341_SPI_V2::Y_Scrolle_Font_2Array_Init_MAX( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t txt_height, uint8_t Xsize, uint8_t Ysize){
  scl_set.txt_width = 2;
  scl_set.txt_height = txt_height;
  font.Xsize = Xsize;
  font.Ysize = Ysize;
  font.PrevXsize = Xsize;
  font.PrevYsize = Ysize;

  ESP32_LCD_ILI9341_SPI_V2::Font_Param_MaxClip( font, scl_set );
  txt_height = 15; //最大配列サイズにする
  scl_set.Scl_ArraySize1 = 16 * font.Xsize * 2;
  scl_set.Scl_ArraySize2 = 16 * txt_height;

  return ESP32_LCD_ILI9341_SPI_V2::Array2_New_Create( scl_set, scl_set.Scl_ArraySize1, scl_set.Scl_ArraySize2 );
}
//********************************************************************
uint8_t ESP32_LCD_ILI9341_SPI_V2::Array2_New_Create( ScrolleSET_PARAM &scl_set, uint16_t ary_size1, uint16_t ary_size2){
  if( scl_set.isHeap_Create == true ){
    Serial.printf("Cannot create new heap memory.\r\n heap size = %d\r\n", esp_get_free_heap_size());
    ESP32_LCD_ILI9341_SPI_V2::Array2_Delete( scl_set );
  }
  Serial.printf("Before Create New Free Heap Size = %d\r\n", esp_get_free_heap_size());

  //ヒープ領域２次元配列動的確保
  scl_set.Scl_Array = new uint8_t *[ ary_size2 ];
  //if(scl_set.Scl_Array == NULL) return 0;
  int i, j;
  for(i = 0; i < ary_size2; i++) {
    scl_set.Scl_Array[ i ] = new uint8_t [ ary_size1 ];
    //if(scl_set.Scl_Array[ i ] == NULL) return 0;
  }

  for(i = 0; i < ary_size2; i++){
    for(j = 0; j < ary_size1; j++){
      scl_set.Scl_Array[i][j] = 0;
    }
  }

  scl_set.isHeap_Create = true;
  Serial.printf("After Create New Free Heap Size = %d\r\n", esp_get_free_heap_size());
  return 1;
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Array2_Delete( ScrolleSET_PARAM &scl_set ){
  if( scl_set.isHeap_Create == false ){
    Serial.printf("Cannot create delete heap memory.\r\n heap size = %d\r\n", esp_get_free_heap_size());
    Serial.println("First, you should create heap memory!");
    return;
  }
  Serial.printf("Before Delete Free Heap Size = %d\r\n", esp_get_free_heap_size());

  for (int i = 0; i < scl_set.Scl_ArraySize2; i++){
    delete[] scl_set.Scl_Array[i];
  }
  delete[] scl_set.Scl_Array;

  scl_set.isHeap_Create = false;
  Serial.printf("After Delete Free Heap Size = %d\r\n", esp_get_free_heap_size());
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Font_Param_MaxClip( FONT_PARAM &font, ScrolleSET_PARAM &scl_set ){
  if( font.x0 >= m_MAX_DISP_WIDTH_X || font.x0 < 0 ) font.x0 = 0;
  if( font.y0 >= m_MAX_DISP_HEIGHT_Y || font.y0 < 0 ) font.y0 = 0;

  scl_set.Max_Disp_x0_X = m_MAX_DISP_WIDTH_X - font.x0;
  if( scl_set.Max_Disp_x0_X < font.Xsize * 8 ){
    font.x0 = 0;
    scl_set.Max_Disp_x0_X = m_MAX_DISP_WIDTH_X;
  }
  scl_set.Max_Disp_y0_Y = m_MAX_DISP_HEIGHT_Y - font.y0;
  if( scl_set.Max_Disp_y0_Y < font.Ysize * 16 ){
    font.y0 = 0;
    scl_set.Max_Disp_y0_Y = m_MAX_DISP_HEIGHT_Y;
  }

  if(font.Xsize >= m_FNT8x16_X_MAX){
    font.Xsize = m_FNT8x16_X_MAX;
  }
  if(font.Ysize >= m_FNT8x16_Y_MAX){
    font.Ysize = m_FNT8x16_Y_MAX;  //under 15
  }

  //scl_set.txt_widthは少数を下げて文字欠けを防ぐ
  uint8_t txt_wd_max = floor((float)scl_set.Max_Disp_x0_X / (8 * font.Xsize));
  if(scl_set.txt_width > txt_wd_max){
    scl_set.txt_width = txt_wd_max;
  }
  uint8_t txt_ht_max = floor((float)scl_set.Max_Disp_y0_Y/ (16 * font.Ysize));
  if(scl_set.txt_height > txt_ht_max){
    scl_set.txt_height = txt_ht_max;
  }

  scl_set.SclPixSizeX = scl_set.txt_width * ( 8 * font.Xsize );
  if( scl_set.SclPixSizeX > scl_set.Max_Disp_x0_X ){
    scl_set.SclPixSizeX = scl_set.Max_Disp_x0_X;
  }
  scl_set.SclPixSizeY = scl_set.txt_height * ( font.Ysize * 16 );
  if( scl_set.SclPixSizeY > scl_set.Max_Disp_y0_Y ){
    scl_set.SclPixSizeY = scl_set.Max_Disp_y0_Y;
  }

  if( scl_set.XsclSendBytesNum > scl_set.Scl_ArraySize1 ){
    scl_set.XsclSendBytesNum = scl_set.Scl_ArraySize1;
  }
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Scrolle_Font_SetUp( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, bool xy_range_set ){
  //void ESP32_LCD_ILI9341_SPI_V2::Scrolle_Font_SetUp(const ILI9341_FONT8X16& font, bool xy_range_set){

  //Default bg_red = 0, bg_green = 0, bg_blue = 0, x0 = 0, y0 = 0, Xsize = 1, Ysize = 1, xy_range_set = false
  
  ESP32_LCD_ILI9341_SPI_V2::Scrolle_Font_Color_Set( font );
  ESP32_LCD_ILI9341_SPI_V2::Scrolle_Form_Set( font, scl_set, scl_set.txt_width, scl_set.txt_height, font.Xsize, font.Ysize);

  if( xy_range_set ){
    ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
    ESP32_LCD_ILI9341_SPI_V2::XY_Range( font.x0, font.y0, font.x0 + scl_set.SclPixSizeX - 1, font.y0 + scl_set.SclPixSizeY - 1 );
    ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
    if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
  }
  //Serial.printf("--------------------scl_set.SclPixSizeX= %d\r\n", scl_set.SclPixSizeX);
  //Serial.printf("--------------------scl_set.SclPixSizeY= %d\r\n", scl_set.SclPixSizeY);
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Scrolle_Font_Color_Set( FONT_PARAM &font ){
  //Default bg_red = 0, bg_green = 0, bg_blue = 0
  font.Dot_MSB = (font.red << 3) | (font.green >> 3);
  font.Dot_LSB = (font.green << 5) | font.blue;

  font.BG_Dot_MSB = (font.bg_red << 3) | (font.bg_green >> 3);
  font.BG_Dot_LSB = (font.bg_green << 5) | font.bg_blue;
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Scrolle_Form_Set(FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint8_t txt_width, uint8_t txt_height, uint8_t Xsize, uint8_t Ysize){
  scl_set.SclPixSizeX = txt_width * ( 8 * Xsize );
  scl_set.SclPixSizeY = txt_height * ( Ysize * 16 );
  font.Xsize = Xsize;
  font.Ysize = Ysize;
  font.PrevXsize = Xsize;
  font.PrevYsize = Ysize;

  ESP32_LCD_ILI9341_SPI_V2::Font_Param_MaxClip( font, scl_set );

  scl_set.YsclSendBytesNum = font.Xsize * 16 * 2;
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Scrolle_XYrange_Com_Change( FONT_PARAM &font, ScrolleSET_PARAM &scl_set ){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();
  ESP32_LCD_ILI9341_SPI_V2::XY_Range(font.x0, font.y0, font.x0 + scl_set.SclPixSizeX - 1, font.y0 + scl_set.SclPixSizeY - 1 );
  ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Scrolle_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t *fnt_cnt, const uint16_t &FontSJ_Length, uint8_t Fnt[][16], bool xy_range_set){
  //Default xy_range_set = true;
  if( millis() - scl_set.SclLastTime > scl_set.SclSpeed){
    scl_set.FontSJ_Length = FontSJ_Length;
    if( (font.Xsize != font.PrevXsize) || (font.Ysize != font.PrevYsize) ){
      ESP32_LCD_ILI9341_SPI_V2::Scrolle_Form_Set( font, scl_set,  scl_set.txt_width, scl_set.txt_height, font.Xsize, font.Ysize );
    }

    ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();

    if( xy_range_set ){
      ESP32_LCD_ILI9341_SPI_V2::XY_Range(font.x0, font.y0, font.x0 + scl_set.SclPixSizeX - 1, font.y0 + scl_set.SclPixSizeY - 1 );
      ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
    }

    int i, j;
    uint16_t inc_ary;
    uint8_t bt1 = 0b10000000;
    uint16_t array_max = scl_set.SclPixSizeX * 2;

    for(i = 0; i < 16; i++){
      inc_ary = scl_set.XsclSendBytesNum;
      if( Fnt[ *fnt_cnt ][ i ] & (bt1 >> scl_set.SingleFontSclCount) ){
        for( j = font.Xsize; j > 0; j-- ){
          scl_set.Scl_Array[ i ][ inc_ary++ ] = font.Dot_MSB;
          scl_set.Scl_Array[ i ][ inc_ary++ ] = font.Dot_LSB;
          if(inc_ary >= array_max) break;;
        }
      }else{
        for( j = font.Xsize; j > 0; j-- ){
          scl_set.Scl_Array[ i ][ inc_ary++ ] = font.BG_Dot_MSB;
          scl_set.Scl_Array[ i ][ inc_ary++ ] = font.BG_Dot_LSB;
          if(inc_ary >= array_max) break;
        }
      }
    }

    if( (inc_ary + 2 * font.Xsize) > array_max ) inc_ary = 0;

    int16_t now_dec_send_byteNum = array_max - inc_ary;
//Serial.printf("arry_max=%d, inc_ary=%d, now_dec_send_byteNum=%d\r\n",array_max, inc_ary, now_dec_send_byteNum);
//Serial.printf("scl_cnt=%d, fnt_cnt=%d\r\n", scl_set.SingleFontSclCount, *fnt_cnt);
    for(i = 0; i < 16; i++){
      for(j = font.Ysize; j > 0; j--){
        ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ inc_ary ], now_dec_send_byteNum );
        ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ 0 ], inc_ary );
      }
    }

    scl_set.XsclSendBytesNum = inc_ary;

    scl_set.SingleFontSclCount++;
    if( scl_set.SingleFontSclCount > 7 ){
      scl_set.SingleFontSclCount = 0;
      (*fnt_cnt)++;
      if( *fnt_cnt >= scl_set.FontSJ_Length ) {
        *fnt_cnt = 0;
      }
    }

    scl_set.SclLastTime = millis();
    if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
  }
}
//********************************************************************
boolean ESP32_LCD_ILI9341_SPI_V2::Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set ){
  boolean ret = false;
  scl_set.FontSJ_Length = fontSJ_Length;

  ESP32_LCD_ILI9341_SPI_V2::Scrolle_8x16_Font( font, scl_set, &scl_set.ZenOrHan_cnt, fontSJ_Length, Fnt, xy_range_set );
  if((scl_set.Zen_or_Han == 2) && (scl_set.ZenOrHan_cnt == 2)){
    scl_set.ZenOrHan_cnt = 0;
    ret = true;
  }else if((scl_set.Zen_or_Han == 1) && (scl_set.ZenOrHan_cnt == 1)){
    scl_set.ZenOrHan_cnt = 0;
    ret = true;
  }
  return ret;
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::Rev_Scrolle_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, uint16_t *fnt_cnt, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set ){
  //Default xy_range_set = true;
  //半角２文字連続の場合は、東雲フォントライブラリで全角として扱われるので、逆スクロールの場合は読み取れなくなる。
  //しかし、基本的に半角英語の逆スクロールは無いものと考え、敢えて修正しない。
  if( millis() - scl_set.SclLastTime > scl_set.SclSpeed){
    scl_set.FontSJ_Length = fontSJ_Length;
    if( (font.Xsize != font.PrevXsize) || (font.Ysize != font.PrevYsize) ){
      ESP32_LCD_ILI9341_SPI_V2::Scrolle_Form_Set( font, scl_set,  scl_set.txt_width, scl_set.txt_height, font.Xsize, font.Ysize );
    }
    ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();

    if( xy_range_set ){
      ESP32_LCD_ILI9341_SPI_V2::XY_Range( font.x0, font.y0, font.x0 + scl_set.SclPixSizeX - 1, font.y0 + scl_set.SclPixSizeY - 1 );
      ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
    }

    int i, j;
    uint8_t bt1 = 0b00000001;
    int dec_ary;
    uint8_t f_cnt = 0;
    uint16_t array_max = ( scl_set.SclPixSizeX ) * 2 - 1;

    if( scl_set.Zen_or_Han == 2 && (*fnt_cnt) == 0) f_cnt = 1;

    for(i = 0; i < 16; i++){
      dec_ary = array_max - scl_set.XsclSendBytesNum;
      if( Fnt[ f_cnt ][ i ] & (bt1 << scl_set.SingleFontSclCount) ){
        for( j = font.Xsize; j > 0; j-- ){
          scl_set.Scl_Array[ i ][ dec_ary-- ] = font.Dot_LSB; //正スクロールと逆のバイト配列になり、LSBが先行
          scl_set.Scl_Array[ i ][ dec_ary-- ] = font.Dot_MSB;
          if(dec_ary < 0) break;
        }
      }else{
        for( j = font.Xsize; j > 0; j-- ){
          scl_set.Scl_Array[ i ][ dec_ary-- ] = font.BG_Dot_LSB; //正スクロールと逆のバイト配列になり、LSBが先行
          scl_set.Scl_Array[ i ][ dec_ary-- ] = font.BG_Dot_MSB;
          if(dec_ary < 0) break;
        }
      }
    }

    scl_set.XsclSendBytesNum = scl_set.XsclSendBytesNum + font.Xsize * 2;
    if( scl_set.XsclSendBytesNum + 2 > array_max ) scl_set.XsclSendBytesNum = 0;

    uint16_t send_byte_max = array_max + 1;
    int16_t now_dec_send_byteNum = send_byte_max - scl_set.XsclSendBytesNum;

    if( now_dec_send_byteNum < 0 ){
      for(i = 0; i < 16; i++){
        for(j = font.Ysize; j > 0; j--){
          ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ 0 ], send_byte_max );
        }
      }
    }else{
      for(i = 0; i < 16; i++){
        for(j = font.Ysize; j > 0; j--){
          ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ now_dec_send_byteNum ], scl_set.XsclSendBytesNum );
          ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ 0 ], now_dec_send_byteNum );
        }
      }
    }

    if( ++scl_set.SingleFontSclCount > 7 ){
      scl_set.SingleFontSclCount = 0;
      if( ++(*fnt_cnt) >= scl_set.FontSJ_Length ) {
        *fnt_cnt = 0;
      }
    }

    scl_set.SclLastTime = millis();
    if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
  }
}
//********************************************************************
boolean ESP32_LCD_ILI9341_SPI_V2::Rev_Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set ){
  boolean ret = false;
  scl_set.FontSJ_Length = fontSJ_Length;

  ESP32_LCD_ILI9341_SPI_V2::Rev_Scrolle_8x16_Font( font, scl_set, &scl_set.ZenOrHan_cnt, fontSJ_Length, Fnt, xy_range_set );

  if((scl_set.Zen_or_Han == 2) && (scl_set.ZenOrHan_cnt == 2)){
    scl_set.ZenOrHan_cnt = 0;
    ret = true;
  }else if((scl_set.Zen_or_Han == 1) && (scl_set.ZenOrHan_cnt == 1)){
    scl_set.ZenOrHan_cnt = 0;
    ret = true;
  }
  return ret;
}
//********************************************************************
boolean ESP32_LCD_ILI9341_SPI_V2::Y_Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set ){
  //Default xy_range_set = true
  boolean ret = false;
  if( millis() - scl_set.SclLastTime > scl_set.SclSpeed){
    if( (font.Xsize != font.PrevXsize) || (font.Ysize != font.PrevYsize) ){
      ESP32_LCD_ILI9341_SPI_V2::Scrolle_Form_Set( font, scl_set,  scl_set.txt_width, scl_set.txt_height, font.Xsize, font.Ysize );
    }
    ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();

    if( xy_range_set ){
      ESP32_LCD_ILI9341_SPI_V2::XY_Range(font.x0, font.y0, font.x0 + scl_set.SclPixSizeX - 1, font.y0 + scl_set.SclPixSizeY - 1 );
      ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
    }

    int i, j;
    uint16_t x_count = 0;
    int bit_count = 0;
    uint8_t bt1 = 0b10000000;
    uint16_t array_Y_max = scl_set.SclPixSizeY / font.Ysize;

    for(i = 0; i < 2; i++){
      if( (i == 1) && (scl_set.Zen_or_Han == 1) ){ //半角の場合、右半分はゼロで埋める
        for( bit_count = 0; bit_count < 8; bit_count++){
          for( j = font.Xsize; j > 0; j-- ){
            scl_set.Scl_Array[ scl_set.scl_Y_cnt ][ x_count++ ] = font.BG_Dot_MSB;
            scl_set.Scl_Array[ scl_set.scl_Y_cnt ][ x_count++ ] = font.BG_Dot_LSB;
          }
        }
      }else{
        for( bit_count = 0; bit_count < 8; bit_count++){
          if( Fnt[ i ][ scl_set.SingleFontSclCount ] & (bt1 >> bit_count) ){
            for( j = font.Xsize; j > 0; j-- ){
              scl_set.Scl_Array[ scl_set.scl_Y_cnt ][ x_count++ ] = font.Dot_MSB;
              scl_set.Scl_Array[ scl_set.scl_Y_cnt ][ x_count++ ] = font.Dot_LSB;
            }
          }else{
            for( j = font.Xsize; j > 0; j-- ){
              scl_set.Scl_Array[ scl_set.scl_Y_cnt ][ x_count++ ] = font.BG_Dot_MSB;
              scl_set.Scl_Array[ scl_set.scl_Y_cnt ][ x_count++ ] = font.BG_Dot_LSB;
            }
          }
        }
      }
    }

    scl_set.scl_Y_cnt++; 
    if( scl_set.scl_Y_cnt >= array_Y_max ) scl_set.scl_Y_cnt = 0;

    for(i = scl_set.scl_Y_cnt; i < array_Y_max; i++){
      for(j = 0; j < font.Ysize; j++){
        ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ 0 ], scl_set.YsclSendBytesNum);
      }
    }
    for(i = 0; i < scl_set.scl_Y_cnt; i++){
      for(j = 0; j < font.Ysize; j++){
        ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][ 0 ], scl_set.YsclSendBytesNum);
      }
    }

    scl_set.SingleFontSclCount++;
    if( scl_set.SingleFontSclCount > 15 ){
      scl_set.SingleFontSclCount = 0;
      ret = true;
    }

    scl_set.SclLastTime = millis();
    if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
  }
  return ret;
}
//********************************************************************
boolean ESP32_LCD_ILI9341_SPI_V2::Ydown_Scrolle_Inc_8x16_Font( FONT_PARAM &font, ScrolleSET_PARAM &scl_set, const uint16_t &fontSJ_Length, uint8_t Fnt[][16], bool xy_range_set ){
  //Default xy_range_set = true
  boolean ret = false;
  if( (millis() - scl_set.SclLastTime) > scl_set.SclSpeed){
    if( (font.Xsize != font.PrevXsize) || (font.Ysize != font.PrevYsize) ){
      ESP32_LCD_ILI9341_SPI_V2::Scrolle_Form_Set( font, scl_set,  scl_set.txt_width, scl_set.txt_height, font.Xsize, font.Ysize );
    }
    ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();

    if( xy_range_set ){
      ESP32_LCD_ILI9341_SPI_V2::XY_Range(font.x0, font.y0, font.x0 + scl_set.SclPixSizeX - 1, font.y0 + scl_set.SclPixSizeY - 1 );
      ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
    }

    int i, j;
    uint8_t bt1 = 0b10000000;
    uint16_t x_count = 0;
    int bit_count = 0;
    uint16_t array_Y_max = scl_set.SclPixSizeY / font.Ysize - 1;
    uint16_t scl_rev_Y_cnt = array_Y_max - scl_set.scl_Y_cnt;

    for(i = 0; i < 2; i++){
      if( (i == 1) && (scl_set.Zen_or_Han == 1) ){ //半角の場合、右半分はゼロで埋める
        for( bit_count = 0; bit_count < 8; bit_count++){
          for( j = font.Xsize; j > 0; j-- ){
            scl_set.Scl_Array[ scl_rev_Y_cnt ][ x_count++ ] = font.BG_Dot_MSB;
            scl_set.Scl_Array[ scl_rev_Y_cnt ][ x_count++ ] = font.BG_Dot_LSB;
          }
        }
      }else{
        for( bit_count = 0; bit_count < 8; bit_count++){
          if( Fnt[i][ 15 - scl_set.SingleFontSclCount ] & (bt1 >> bit_count) ){
            for( j = font.Xsize; j > 0; j-- ){
              scl_set.Scl_Array[ scl_rev_Y_cnt ][ x_count++ ] = font.Dot_MSB;
              scl_set.Scl_Array[ scl_rev_Y_cnt ][ x_count++ ] = font.Dot_LSB;
            }
          }else{
            for( j = font.Xsize; j > 0; j-- ){
              scl_set.Scl_Array[ scl_rev_Y_cnt ][ x_count++ ] = font.BG_Dot_MSB;
              scl_set.Scl_Array[ scl_rev_Y_cnt ][ x_count++ ] = font.BG_Dot_LSB;
            }
          }
        }
      }
    }

    for(i = scl_rev_Y_cnt; i <= array_Y_max; i++){
      for(j = 0; j < font.Ysize; j++){
        ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][0], scl_set.YsclSendBytesNum);
      }
    }
    for(i = 0; i < scl_rev_Y_cnt; i++){
      for(j = 0; j < font.Ysize; j++){
        ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes( &scl_set.Scl_Array[ i ][0], scl_set.YsclSendBytesNum);
      }
    }

    scl_set.scl_Y_cnt++; 
    if( scl_set.scl_Y_cnt > array_Y_max ) scl_set.scl_Y_cnt = 0;

    scl_set.SingleFontSclCount++;
    if( scl_set.SingleFontSclCount > 15 ){
      scl_set.SingleFontSclCount = 0;
      ret = true;
    }

    scl_set.SclLastTime = millis();
    if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
  }
  return ret;
}
//********************************************************************
void ESP32_LCD_ILI9341_SPI_V2::XYsizeUp_8x16_Font_DisplayOut( FONT_PARAM &font, uint16_t fontSJ_Length, uint8_t Fnt[][16] ){
  ESP32_LCD_ILI9341_SPI_V2::SPI_set_change();

  if(font.Xsize > 16) font.Xsize = 16;
  if(font.Ysize > 16) font.Ysize = 16;

  uint8_t X_txt_MAX = (uint8_t)ceilf( (float)m_FNT8x16_X_MAX / (float)font.Xsize );
  if( fontSJ_Length > X_txt_MAX) fontSJ_Length = X_txt_MAX;

  uint8_t bt = 0b00000001;
  uint8_t dot_MSB = (font.red << 3) | (font.green >> 3);
  uint8_t dot_LSB = (font.green << 5) | font.blue;

  uint8_t bg_dot_MSB = 0;
  uint8_t bg_dot_LSB = 0;
  if( (font.bg_red + font.bg_green + font.bg_blue) != 0 ){
    bg_dot_MSB = (font.bg_red << 3) | (font.bg_green >> 3);
    bg_dot_LSB = (font.bg_green << 5) | font.bg_blue;
  }

  uint8_t disp_byte[(fontSJ_Length * 8 * font.Xsize) * 2] = {};
  uint16_t byte_cnt = 0;
  uint16_t X_pix_cnt = font.x0, Y_pix_cnt = font.y0;
  uint16_t Y_tmp_range = 0;

  int i, j, k, ii, jj;
  for(i = 0; i < 16; i++){
    for(j = 0; j < fontSJ_Length; j++){
      for( k = 7; k >= 0; k--){
        if( Fnt[j][i] & (bt << k) ){
          for(ii = font.Xsize; ii > 0; ii--){
            disp_byte[byte_cnt++] = dot_MSB;
            disp_byte[byte_cnt++] = dot_LSB;
            X_pix_cnt++;
            if(X_pix_cnt > ESP32_LCD_ILI9341_SPI_V2::m_Max_Pix_X) goto EXIT_max_pixX;
          }
        }else{
          for(ii = font.Xsize; ii > 0; ii--){
            disp_byte[byte_cnt++] = bg_dot_MSB;
            disp_byte[byte_cnt++] = bg_dot_LSB;
            X_pix_cnt++;
            if(X_pix_cnt > ESP32_LCD_ILI9341_SPI_V2::m_Max_Pix_X) goto EXIT_max_pixX;
          }
        }
      }
    }

EXIT_max_pixX:

    Y_tmp_range = Y_pix_cnt + font.Ysize -1;
    if(Y_tmp_range > ESP32_LCD_ILI9341_SPI_V2::m_Max_Pix_Y) Y_tmp_range = ESP32_LCD_ILI9341_SPI_V2::m_Max_Pix_Y;
    ESP32_LCD_ILI9341_SPI_V2::XY_Range(font.x0, Y_pix_cnt, X_pix_cnt - 1, Y_tmp_range);
    ESP32_LCD_ILI9341_SPI_V2::CommandWrite( 0x2C ); //RAM write
    for(jj = 0; jj < font.Ysize; jj++){
      ESP32_LCD_ILI9341_SPI_V2::DataWriteBytes(disp_byte, byte_cnt);
      Y_pix_cnt++;
      if(Y_pix_cnt > ESP32_LCD_ILI9341_SPI_V2::m_Max_Pix_Y) goto EXIT;
    }
    X_pix_cnt = font.x0;
    byte_cnt = 0;
  }

EXIT:
  if(!m_useHw_Cs) digitalWrite(m_cs, HIGH);
}
