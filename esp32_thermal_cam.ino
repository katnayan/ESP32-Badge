/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch makes a 64 pixel thermal camera with the GridEYE sensor
  and a 128x128 tft screen https://www.adafruit.com/product/2088

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "Arduino_GFX_Library.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// Define the pins of the ESP32 connected to the LCD
#define LCD_MOSI 23  // SDA Pin on ESP32 D23
#define LCD_SCLK 18  // SCL Pin on ESP32 D18
#define LCD_CS   15  // Chip select control pin on ESP32 D15
#define LCD_DC    2  // Data Command control pin on ESP32 D2
#define LCD_RST   4  // Reset pin (could connect to RST pin) on ESP32 D4
#define LCD_BLK   32  // Black Light Pin on ESP32 D32

// Screen dimensions
#define SCREEN_WIDTH 135
#define SCREEN_HEIGHT 240

//low range of the sensor (this will be blue on the screen)
#define MINTEMP 22

//high range of the sensor (this will be red on the screen)
#define MAXTEMP 34

const int HUMAN_TEMP = 23;

const uint16_t TEXT_COLOR = gfx->color565(255, 125, 242);

//const String lines[6] = {
//"   ____  _                   _ _                _    _____      ",
//"  / __ \\| |__  _ __ __ _  __| | |__   __ _  ___| | _|___ / _ __ ",
//" / / _` | '_ \\| '__/ _` |/ _` | '_ \\ / _` |/ __| |/ / |_ \\| '__|",
//"| | (_| | |_) | | | (_| | (_| | | | | (_| | (__|   < ___) | |   ",
//" \\ \\__,_|_.__/|_|  \\__,_|\\__,_|_| |_|\\__,_|\\___|_|\\_\\____/|_|   ",
//"  \\____/                                                        "
//};
const String lines[6] = {
"   ____  _         _                     ",
"  / __ \\| | ____ _| |_ _ __  _   _ _ __  ",
" / / _` | |/ / _` | __| '_ \\| | | | '_ \\ ",
"| | (_| |   < (_| | |_| | | | |_| | | | |",
" \\ \\__,_|_|\\_\\__,_|\\__|_| |_|\\__, |_| |_|",
"  \\____/                     |___/       "
};
const String website = "katherinenayan.com ";

//the colors we will be using
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI);

Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */, SCREEN_WIDTH, SCREEN_HEIGHT, 52, 40, 0, 0);

Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

Adafruit_AMG88xx amg;
unsigned long delayTime;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint16_t displayPixelWidth = SCREEN_HEIGHT / 8;
uint16_t displayPixelHeight = SCREEN_WIDTH / 8;

void setup() {
  Serial.begin(9600);
  Serial.println(F("AMG88xx thermal camera!"));
  
//  // Initialize the button pin
//  pinMode(BUTTON_PIN, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, RISING);

  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  //Serial.println("WiFi Viz");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

#if defined(LCD_PWR_PIN)
  pinMode(LCD_PWR_PIN, OUTPUT);     // sets the pin as output
  digitalWrite(LCD_PWR_PIN, HIGH);  // power on
#endif

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
//  w = gfx->width();
//  h = gfx->height();
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);
  gfx->fillScreen(BLACK);
  // init banner
  //Set screen rotation in lanscape mode
  gfx->setRotation(1);
  gfx->setTextSize(1.5);
  gfx->setTextColor(WHITE);
  gfx->setCursor(0, 60);
  gfx->print("Instructions and Source Code\nAvailable at:");
  gfx->setCursor(0, 80);
  gfx->setTextSize(2);
  gfx->setTextColor(GREEN);
  gfx->print("Polarity.io/Shmoo");
  // Add in my name
  gfx->setCursor(0, 100);
  gfx->setTextSize(1.5);
  gfx->setTextColor(WHITE);
  gfx->print("With modifications by:");
  gfx->setCursor(0, 110);
  gfx->setTextSize(2);
  gfx->setTextColor(GREEN);
  gfx->print("@bradhack3r");
  delay(2000);
  gfx->fillScreen(BLACK);

  bool status;
  // default settings
  status = amg.begin();
  if (!status) {
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->print("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }
  
  Serial.println("-- Thermal Camera Test --");
  delay(100); // let sensor boot up
}

void loop() {
  float avgPixel = getTempAvg();
  Serial.printf("Avg pixel val: %f\n", avgPixel);
  

  // Check if a human is detected
  if (avgPixel > HUMAN_TEMP) {
    Serial.println("Human Detected!");
    displayHelloMsg();
  } else {
    // Display ASCII name tag if no human
    displayASCIIName();
  }
  
  delay(100);
}

void showThermalGrid() {
  //read all the pixels
  amg.readPixels(pixels);

  // Record sum for avg
  float pixelSum = 0;

  // Iterate over pixels
  for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    uint8_t colorIndex = map(pixels[i], MINTEMP, MAXTEMP, 0, 255);
    colorIndex = (uint8_t)constrain((int16_t)colorIndex, (int16_t)0, (int16_t)255);

    //draw the pixels!
    gfx->fillRect(displayPixelHeight * floor(i / 8), displayPixelWidth * (i % 8),
      displayPixelHeight, displayPixelWidth, camColors[colorIndex]);
    pixelSum += pixels[i];
  }
}

float getTempAvg() {
  //read all the pixels
  amg.readPixels(pixels);

  // Record sum for avg
  float pixelSum = 0;

  // Iterate over pixels
  for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    pixelSum += pixels[i];
  }

  return pixelSum / AMG88xx_PIXEL_ARRAY_SIZE;
}

void displayHelloMsg() {
  // Clear the display
  gfx->fillScreen(BLACK);
  // Write Hello to screen
  gfx->setCursor(0, 60);
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE);
  gfx->print("Hi, I'm Kat!");
}

int cursor = 0;
int websiteCursor = 0;
int fontSize = 0.5;
int lineSpacing = 12;
int kerning = 8;

const int asciiWidth = lines[0].length();
const int websiteWidth = website.length();

void displayASCIIName() {
  // Clear the screen
  gfx->fillScreen(BLACK);
  // Set text settings
  gfx->setTextColor(TEXT_COLOR);
  gfx->setTextSize(2);
  // Draw characters
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 29; j++) {
      gfx->setCursor(j * kerning, 60 + i * lineSpacing);
      gfx->print(lines[i][(j + cursor) % asciiWidth]);
    }
  }
  // Draw website scrolling
  for (int j = 0; j < 29; j++) {
    gfx->setCursor(j * kerning, 55);
    gfx->setTextSize(2);
    gfx->print(website[((j + websiteCursor - 1) + websiteWidth) % websiteWidth]);
  }
  cursor = (cursor + 1) % asciiWidth;
  websiteCursor = (websiteCursor - 1) % websiteWidth;
  delay(100);
}
