#define USE_ADAFRUIT_GFX_LAYERS
#include <MatrixHardware_Teensy4_ShieldV5.h>
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

#define COLOR_DEPTH 24                  
const uint16_t kMatrixWidth = 64;      
const uint16_t kMatrixHeight = 32;     
const uint8_t kRefreshDepth = 36;      
const uint8_t kDmaBufferRows = 4;      
const uint8_t kPanelType = SM_PANELTYPE_HUB75_32ROW_MOD16SCAN;
const uint32_t kMatrixOptions = (SM_HUB75_OPTIONS_NONE);
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);

const int defaultBrightness = (100*255)/100;

volatile unsigned long startTime = 0;
volatile bool runTimer = false;
int offsetX = 28;
int offsetY = 7;

long totalTime = 0;

void setup() {
  Serial.begin(115200);
  Serial8.begin(115200);
  delay(200);
  
  Serial.println("Timer Control via Serial8");
  Serial.println("-------------------------");

  // Setup matrix
  matrix.addLayer(&indexedLayer); 
  matrix.begin();

  indexedLayer.fillScreen(0);
  indexedLayer.setFont(&DSEG7_Modern_Bold_16);
  indexedLayer.setTextColor(WHITE);

  matrix.setBrightness(defaultBrightness);

  indexedLayer.fillScreen(0);
  indexedLayer.drawString(kMatrixWidth/2 - offsetX, kMatrixHeight/2 - offsetY, 0, "0.000");
  indexedLayer.swapBuffers(false);
}

void loop() {
  // Check for serial input
  if (Serial8.available() > 0) {
    String command = Serial8.readStringUntil('\n');
    command.trim(); // Remove any extra whitespace

    Serial.print("Received command: ");
    Serial.println(command);

    if (command.equalsIgnoreCase("start")) {
      Serial.println("Timer started");
      startTime = millis();
      runTimer = true;
    } 
    else if (command.equalsIgnoreCase("stop")) {
      Serial.println("Timer stopped");
      runTimer = false;
    } 
    else if (command.equalsIgnoreCase("reset")) {
      Serial.println("Timer reset");
      indexedLayer.fillScreen(0);
      indexedLayer.drawString(kMatrixWidth/2 - offsetX, kMatrixHeight/2 - offsetY, 0, "0.000");
      indexedLayer.swapBuffers(false);
      runTimer = false;
      totalTime = 0;
    }
  }

  // Update the display if the timer is running
  if (runTimer) {
    totalTime = millis() - startTime;
    float timeInSeconds = ((float)totalTime) / 1000;
    String value = String(timeInSeconds, 3);
    const char* charArray = value.c_str();

    indexedLayer.fillScreen(0);
    indexedLayer.drawString(kMatrixWidth/2 - offsetX, kMatrixHeight/2 - offsetY, 0, charArray);
    indexedLayer.swapBuffers(false);
  }

  delay(2);
}
