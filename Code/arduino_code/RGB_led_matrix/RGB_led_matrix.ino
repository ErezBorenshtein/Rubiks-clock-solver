#define USE_ADAFRUIT_GFX_LAYERS
#include <MatrixHardware_Teensy4_ShieldV5.h> 
// uncomment one line to select your MatrixHardware configuration - configuration header needs to be included before <SmartMatrix.h>
//#include <MatrixHardware_Teensy3_ShieldV4.h>        // SmartLED Shield for Teensy 3 (V4)
//#include <MatrixHardware_Teensy4_ShieldV5.h>        // SmartLED Shield for Teensy 4 (V5)
//#include <MatrixHardware_Teensy3_ShieldV1toV3.h>    // SmartMatrix Shield for Teensy 3 V1-V3
//#include <MatrixHardware_Teensy4_ShieldV4Adapter.h> // Teensy 4 Adapter attached to SmartLED Shield for Teensy 3 (V4)
//#include <MatrixHardware_ESP32_V0.h>                // This file contains multiple ESP32 hardware configurations, edit the file to define GPIOPINOUT (or add #define GPIOPINOUT with a hardcoded number before this #include)
//#include "MatrixHardware_Custom.h"                  // Copy an existing MatrixHardware file to your Sketch directory, rename, customize, and you can include it like this
#include <SmartMatrix.h>
#include "my_font.h"

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define INTERRUPT_PIN 34

#define COLOR_DEPTH 24                  // Choose the color depth used for storing pixels in the layers: 24 or 48 (24 is good for most sketches - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24)
const uint16_t kMatrixWidth = 64;       // Set to the width of your display, must be a multiple of 8
const uint16_t kMatrixHeight = 32;      // Set to the height of your display
const uint8_t kRefreshDepth = 36;       // Tradeoff of color quality vs refresh rate, max brightness, and RAM usage.  36 is typically good, drop down to 24 if you need to.  On Teensy, multiples of 3, up to 48: 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48.  On ESP32: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save RAM, more to keep from dropping frames and automatically lowering refresh rate.  (This isn't used on ESP32, leave as default)
const uint8_t kPanelType = SM_PANELTYPE_HUB75_32ROW_MOD16SCAN;   // Choose the configuration that matches your panels.  See more details in MatrixCommonHub75.h and the docs: https://github.com/pixelmatix/SmartMatrix/wiki
const uint32_t kMatrixOptions = (SM_HUB75_OPTIONS_NONE);        // see docs for options: https://github.com/pixelmatix/SmartMatrix/wiki
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);


const int defaultBrightness = (100*255)/100;      // full (100%) brightness

volatile unsigned long startTime = 0;

volatile bool runTimer = false;

int offsetX = 28;
int offsetY = 7;


void timer(){
  if(digitalRead(INTERRUPT_PIN) ==HIGH){
    runTimer = true;
    startTime = millis();
  }
  else{
    runTimer = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("DS1307RTC Read Test");
  Serial.println("-------------------");

  // setup matrix
  matrix.addLayer(&indexedLayer); 
  matrix.begin();

  indexedLayer.fillScreen(0);
  indexedLayer.setFont(&DSEG7_Modern_Bold_16);
  indexedLayer.setTextColor(WHITE);

  matrix.setBrightness(defaultBrightness);

  indexedLayer.fillScreen(0);
  indexedLayer.drawString(kMatrixWidth/2-offsetX, kMatrixHeight/2-offsetY , 0, "0.000");
  indexedLayer.swapBuffers(false);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), timer, CHANGE);

}

long totalTime = 0;

void loop() {

  if(runTimer){
    totalTime = (millis() - startTime);
    float tmp2 = ((float)totalTime) / 1000;
    String value = String(tmp2, 3);
    const char* charArray = value.c_str();
    indexedLayer.fillScreen(0);
    indexedLayer.drawString(kMatrixWidth/2-offsetX, kMatrixHeight/2-offsetY , 0, charArray);
    indexedLayer.swapBuffers(false);
  }
}


