#include <Wire.h>
#include <Adafruit_TCS34725.h>

// sensor instance 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor");
  } else {
    Serial.println("No TCS34725 sensor found ...");
    while (1);
  }
}

void loop() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  // Normalize the RGB values to the sum of all three
  float total = red + green + blue;
  float red_ratio = red / total;
  float green_ratio = green / total;
  float blue_ratio = blue / total;

  // green detection 
  if (isGreen(red_ratio, green_ratio, blue_ratio)) {
    Serial.println("Green stone detected!");
  } else {
    Serial.println("No green stone detected.");
  }

  // Debugging output
  Serial.print("Red Ratio: ");
  Serial.print(red_ratio, 4);
  Serial.print(" Green Ratio: ");
  Serial.print(green_ratio, 4);
  Serial.print(" Blue Ratio: ");
  Serial.println(blue_ratio, 4);

  // Delay before the next reading
  delay(1000);
}

bool isGreen(float red_ratio, float green_ratio, float blue_ratio) {
  // Adjust thresholds based on environment
  return green_ratio > red_ratio && green_ratio > blue_ratio && green_ratio > 0.33;
}