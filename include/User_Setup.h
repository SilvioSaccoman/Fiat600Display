#ifndef USER_SETUP_H
#define USER_SETUP_H

#define ILI9341_DRIVER

// TFT Display (HSPI)
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  -1

// Backlight (può essere 21 o 27)
#define TFT_BL   27

#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// Touchscreen (XPT2046 via VSPI)
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CLK  25

#define SPI_FREQUENCY 27000000

#endif
