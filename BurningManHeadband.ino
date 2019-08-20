#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
//#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 40 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    10
#define CLOCKPIN   6
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

#define DATAPIN2    5
#define CLOCKPIN2   3
Adafruit_DotStar strip2(NUMPIXELS, DATAPIN2, CLOCKPIN2, DOTSTAR_BRG);

#include <EEPROM.h>

float t = 0;

void opposing_sin() {
  for (int i = 0; i < 40; i++) {
    float intensity = max(0, sin((float) i/4.0 + t*0.1));
    intensity = 32.0 * intensity * intensity * intensity;
    strip.setPixelColor(i, 0xFF & ((int) intensity));
    strip2.setPixelColor(40 - i - 1, 0xFF & ((int) intensity));
  }
}

void oscillating_grid() {
  for (int i = 0; i < 40; i++) {
    float intensity = (sin(t*0.1) + 1.0) / 2.0;
    intensity = 32.0 * intensity * intensity * intensity;
    
    float intensity2 = (cos(t*0.1 + 3.14159/2.0) + 1.0) / 2.0;
    intensity2 = 32.0 * intensity2 * intensity2 * intensity2;
    
    strip.setPixelColor(i, 0xFF & ((int) ((i % 2) == 0 ? intensity : intensity2)));
    strip2.setPixelColor(i, 0xFF & ((int) ((i % 2) == 1 ? intensity : intensity2)));
  }
}

void oscillating_lines() {
  for (int i = 0; i < 40; i++) {
    float intensity = (sin(t*0.1) + 1.0) / 2.0;
    intensity = 32.0 * intensity * intensity * intensity;
    
    float intensity2 = (cos(t*0.1 + 3.14159/2.0) + 1.0) / 2.0;
    intensity2 = 32.0 * intensity2 * intensity2 * intensity2;
    
    strip.setPixelColor(i, 0xFF & ((int) intensity));
    strip2.setPixelColor(i, 0xFF & ((int) intensity2));
  }
}

void the_worm() {
  float divisor = 80 / (3.14159*2.0*2.0);
  for (int i = 0; i < 80; i++) {
    float intensity = max(0, sin((float) i/divisor + t*0.1));
    intensity = 32.0 * intensity * intensity * intensity;

    if (i < 40) {
      strip.setPixelColor(i, 0xFF & ((int) intensity));
    } else {
      strip2.setPixelColor(40 - (i - 40) - 1, 0xFF & ((int) intensity));
    }
  }
}

int minSound = 1024;
int numBuckets = 5;

int last100Samples[100];
int sampleIndex = 0;
int last50MaxBuckets[50];
int maxBucketIndex = 0;

void visualizer() {
  int sound = analogRead(A0);
  if (sound < minSound) {
    minSound = sound;
  }

  last100Samples[sampleIndex] = sound;
  sampleIndex = (sampleIndex + 1) % 100;
  if (sampleIndex == 0) {
    int maxSample = 0;
    for(int i = 0; i < 100; i++) {
      if (last100Samples[i] > maxSample) {
        maxSample = last100Samples[i];
      }
    }
    last50MaxBuckets[maxBucketIndex] = maxSample;
    Serial.print(maxBucketIndex);
    Serial.print(" -> ");
    Serial.println(maxSample);
    maxBucketIndex = (maxBucketIndex + 1) % numBuckets;
  }

  int maxSound = 0;
  for (int i = 0; i < numBuckets; i++) {
    if (last50MaxBuckets[i] > maxSound) {
      maxSound = last50MaxBuckets[i];
    }
  }
  
  float soundRatio = 0.5;
  if (minSound != maxSound) {
    soundRatio = (float) (sound - minSound) / (float) (maxSound - minSound);
  }
  for (int i = 0; i < 40; i++) {
    float ratio = ((float) i)/40.0;
    if (ratio < soundRatio) {
      strip.setPixelColor(i, 0x11);
      strip2.setPixelColor(i, 0x11);
    } else {
      strip.setPixelColor(i, 0x00);
      strip2.setPixelColor(i, 0x00);
    }
  }

  /*
  Serial.print(sound);
  Serial.print(" ");
  Serial.print(minSound);
  Serial.print(" ");
  Serial.print(maxSound);
  Serial.print(" ");
  Serial.println(soundRatio);
  Serial.print(" ");
  Serial.println(maxBucketIndex);
  */
  
}

#define VBATPIN A9

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2),buttonPressed,CHANGE);  

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  strip2.begin(); // Initialize pins for output
  strip2.show();  // Turn all LEDs off ASAP

  for (int i = 0; i < 50; i++) {
    last50MaxBuckets[i] = 0;
  }
  for (int i = 0; i < 100; i++) {
    last100Samples[i] = 0;
  }
}

int lastButton = 1;
int lastInterrupt = 0;
int activeProgram = 0;
int numPrograms = 5;

void buttonPressed()
{
  int pin = digitalRead(2);
  int ts = millis();

  // Debounce
  if (ts - lastInterrupt <= 100) {
    return;
  }
  
  // More debouncing
  if (pin == lastButton) {
    return;
  }

  lastInterrupt = ts;
  lastButton = pin;
  if (pin == 0) {
    Serial.println("Down");
    activeProgram = (activeProgram + 1);
  } else {
    Serial.println("Up");
  }
}

void rainbow() {
   for (int i = 0; i < 40; i++) {
    float intensity = max(0, sin((float) i/4.0 + t*0.1));
    intensity = 32.0 * intensity * intensity * intensity;
    strip.setPixelColor(i, Brightness(Wheel(i*3 + t*2), 0.1));
    strip2.setPixelColor(i, 0x000011);
  }
}

int min_voltage_addr = 0;
int max_voltage_addr = sizeof(float);

void battery() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  // Keep track of min and max voltage
  float minimum_voltage = EEPROM.read(min_voltage_addr);
  if ((measuredvbat < minimum_voltage) || minimum_voltage < 3.0 || minimum_voltage > 5.0) {
    EEPROM.write(min_voltage_addr, measuredvbat);
  }

  float maximum_voltage = EEPROM.read(min_voltage_addr);
  if ((measuredvbat > maximum_voltage) || maximum_voltage < 3.0 || maximum_voltage > 5.0) {
    EEPROM.write(maximum_voltage, measuredvbat);
  }

  minimum_voltage = 3.2;
  maximum_voltage = 4.2;
  float voltageRatio = (maximum_voltage - measuredvbat) / (maximum_voltage - minimum_voltage);
  for (int i = 0; i < 40; i++) {
    float ratio = ((float) i)/40.0;
    if (ratio > voltageRatio) {
      strip.setPixelColor(i, 0x000011);
      strip2.setPixelColor(i, 0x11);
    } else {
      strip.setPixelColor(i, 0x001100);
      strip2.setPixelColor(i, 0x11);
    }
  }

  //Serial.print("VBat: " ); Serial.print(voltageRatio); Serial.print(" "); Serial.println(measuredvbat);
}

void itsthepolice() {
  for (int i = 0; i < 40; i++) {
    if (((i / 10) % 2) == 0) {
      if ((((int)t) % 2) == 0) {
        strip.setPixelColor(i, 0x000011);
      } else {
        strip.setPixelColor(i, 0x001100);
      }
    } else {
      if ((((int)t) % 2) == 0) {
        strip.setPixelColor(i, 0x001100);
      } else {
        strip.setPixelColor(i, 0x000011);
      }
    }
    if ((((int)t) % 2) == 0) {
      strip2.setPixelColor(i, 0x000011);
    } else {
      strip2.setPixelColor(i, 0x000000);
    }
  }
}

typedef void (* Cycle)();
Cycle CYCLES[] = {battery, rainbow, opposing_sin, oscillating_grid, oscillating_lines, the_worm, visualizer};

void loop() {
  int num = sizeof(CYCLES)/ sizeof(CYCLES[0]);
  CYCLES[activeProgram % num]();
  
  t += 1.0;
  strip.show();                     // Refresh strip
  strip2.show();                     // Refresh strip
  //delay(10);                        // Pause 20 milliseconds (~50 FPS)

  /*
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  */
}

// Utility Functions

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

uint32_t Brightness(uint32_t input, float level) {
  uint32_t c;
  byte b = input & 0xFF;
  byte g = (input & 0xFF00) >> 8;
  byte r = (input & 0xFF0000) >> 16;

  return Color(r*level, g*level, b*level);
}
