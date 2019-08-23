/*** LED Configuration ***/

#include <FastLED.h>
#define NUM_LEDS 40

#define COLOR_DATA_PIN    10
#define COLOR_CLOCK_PIN   6
CRGB colorLeds[NUM_LEDS];

#define WHITE_DATA_PIN    5
#define WHITE_CLOCK_PIN   3
CRGB whiteLeds[NUM_LEDS];

/*** Radio Configuration ***/

#include <SPI.h>
#include <RH_RF69.h>
#define RF69_FREQ 915.0

#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13

RH_RF69 rf69(RFM69_CS, RFM69_INT);
long clientId;




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

/*
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
*/

int activeProgram = 0;
long lastProgramChange = 0;
void changeProgram()
{
  long ts = millis();
  if (ts - lastProgramChange < 200)  {
    return;
  }
  lastProgramChange = ts;
  activeProgram++;
}

int minSound = 1024;
int numBuckets = 5;

int lastSamples[100];
int sampleIndex = 0;
int lastMaxBuckets[50];
int maxBucketIndex = 0;


void setup() {
  // Initialize LEDs
  FastLED.addLeds<APA102, COLOR_DATA_PIN, COLOR_CLOCK_PIN, BGR>(colorLeds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<APA102, WHITE_DATA_PIN, WHITE_CLOCK_PIN, BGR>(whiteLeds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( 20 );

  // Reset the Radio
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  // Wait for initialization
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  // Set Frequency and initialize LED for transmission
  Serial.println("RFM69 radio init OK!");
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  pinMode(LED, OUTPUT);
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  rf69.setTxPower(20, true);

  // Initialize a random number generator to generate client ids for automatic master/slave setup
  randomSeed(analogRead(A11));
  clientId = random(1024);

  // Initialize interrupts for button that changes program
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2),changeProgram,LOW);  

  for (int i = 0; i < 50; i++) {
    lastMaxBuckets[i] = 0;
  }
  for (int i = 0; i < 100; i++) {
    lastSamples[i] = 0;
  }
}


// This is used for time synchronization
long lastPing = 0;
long timeDelta = 0;
long lastRSSI = 0;

struct message {
  char kind[4];
  unsigned long source_id;
  long query_ts;
  long response_ts;
  unsigned long response_id;
  long lastProgramChange;
  int currentProgram;
};

void send_ping() {
  struct message msg;
  msg.kind[0] = 'P';
  msg.kind[1] = 'I';
  msg.kind[2] = 'N';
  msg.kind[3] = 'G';
  msg.source_id = clientId;
  msg.query_ts = millis() + timeDelta;
  msg.lastProgramChange = lastProgramChange + timeDelta;
  msg.currentProgram = activeProgram;
  rf69.send((char *) &msg, sizeof(msg));
  Serial.println("Sending ping");
};

void send_pong(struct message ping) {
  ping.kind[1] = 'O';
  ping.response_ts = millis() + timeDelta;
  ping.response_id = clientId;
  rf69.send((char *) &ping, sizeof(ping));
}

void handle_messages() {
  while (rf69.available()) {
    struct message msg;
    uint8_t len = sizeof(msg);
    if (rf69.recv((char *) &msg, &len)) {
      /*
      Serial.print("Got packet: ");
      Serial.print(msg.kind);
      Serial.print(" ");
      Serial.print(len);
      Serial.print(" @ ");
      Serial.println(rf69.lastRssi(), DEC);
      */
      lastRSSI = rf69.lastRssi();

      if (!len) return;

      if (strncmp(msg.kind, "PING", 4) == 0) {
        if (msg.lastProgramChange > (lastProgramChange + timeDelta) && msg.currentProgram != activeProgram) {
          activeProgram = msg.currentProgram;
        }
        send_pong(msg);
      }
      if (strncmp(msg.kind, "PONG", 4) == 0) {
        long now = millis() + timeDelta;

        // Update offsets
        Serial.print("Got pong: ");
        Serial.print(msg.source_id);
        Serial.print(" -> ");
        Serial.print(msg.response_id);
        Serial.print(": ");
        Serial.print(msg.query_ts);
        Serial.print(" ");
        Serial.print(msg.response_ts);
        Serial.print(" ");
        Serial.println(now);

        if (msg.source_id < msg.response_id) {
          long change = (2*msg.response_ts - msg.query_ts - now) / 2;
          Serial.print("Updating by ");
          Serial.println(change);
          timeDelta += change;
        }
      }
    } 
  }
}

void uniform_rainbow(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    colorLeds[i] = CHSV((int) (t * 120.0) % 255, 255, 255);
    whiteLeds[i] = CRGB::Black;
  }
}

void rainbow_stripes(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    colorLeds[i] = CHSV(((i*255/NUM_LEDS) + (int) (t * 120.0)) % 255, 255, 255);
    whiteLeds[i] = CRGB::Black;
  }
}

void white_bar(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    colorLeds[i] = CRGB::Black;
    whiteLeds[i] = CRGB::White;
  }
}

void red_bar(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    colorLeds[i] = CRGB::Red;
    whiteLeds[i] = CRGB::Black;
  }
}

#define CENTER_PIXEL 17

void rainbow_mirror_stripes(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    int j;
    if (i < CENTER_PIXEL) {
      j = CENTER_PIXEL - i - 1;
    } else {
      j = i - CENTER_PIXEL;
    }
    colorLeds[i] = CHSV(((j*255/NUM_LEDS) - (int) (t * 120.0)) % 255, 255, 255);
    whiteLeds[i] = CRGB::Black;
  }
}

void white_mirror_comet(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    int j;
    if (i < CENTER_PIXEL) {
      j = CENTER_PIXEL - i - 1;
    } else {
      j = i - CENTER_PIXEL;
    }

    float intensity = max(0, sin((float) j/4.0 - t*10));
    intensity = 255.0 * intensity * intensity * intensity;

    colorLeds[i] = CRGB::Black;
    whiteLeds[i] = CRGB::Black;
    whiteLeds[i].r = 0xFF & ((int) intensity);
  }
}

void twinkle(float t) {
  for (int i = 0; i < NUM_LEDS; i++) {
    colorLeds[i] = CRGB::Black;
    whiteLeds[i] = CRGB::Black; 
    whiteLeds[i].r = dim8_raw(sin8(inoise8_raw(i*NUM_LEDS*7) + inoise8_raw(i*NUM_LEDS)*10*t));
  }
}

long blendedRSSI = 0;

void hot_and_cold(float t) {
  blendedRSSI = (long) (((float) blendedRSSI)*0.99 + ((float) lastRSSI)*0.01);
  float ratio = (blendedRSSI - -20.0) / (-70.0 - -20.0);

  for (int i = 0; i < NUM_LEDS; i++) {
    colorLeds[i] = CRGB::Black;
    float b = 255*(ratio);
    float r = 255*(1.0 - ratio);
    colorLeds[i].r = max(0, min(255, r));
    colorLeds[i].b = max(0, min(255, b));
    whiteLeds[i] = CRGB::Black; 
  }
}

#define VBATPIN A9

void showVoltage() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  float minimum_voltage = 3.2;
  float maximum_voltage = 4.2;
  float voltageRatio = (maximum_voltage - measuredvbat) / (maximum_voltage - minimum_voltage);
  for (int i = 0; i < 40; i++) {
    float ratio = ((float) i)/40.0;
    if (ratio > voltageRatio) {
      colorLeds[i] = CRGB::Blue;
    } else {
      colorLeds[i] = CRGB::Red;
    }
    whiteLeds[i] = CRGB::Black;
  }
}

void visualizer() {
  int sound = analogRead(A0);
  if (sound < minSound) {
    minSound = sound;
  }

  lastSamples[sampleIndex] = sound;
  sampleIndex = (sampleIndex + 1) % 100;
  if (sampleIndex == 0) {
    int maxSample = 0;
    for(int i = 0; i < 100; i++) {
      if (lastSamples[i] > maxSample) {
        maxSample = lastSamples[i];
      }
    }
    lastMaxBuckets[maxBucketIndex] = maxSample;
    Serial.print(maxBucketIndex);
    Serial.print(" -> ");
    Serial.println(maxSample);
    maxBucketIndex = (maxBucketIndex + 1) % numBuckets;
  }

  int maxSound = 0;
  for (int i = 0; i < numBuckets; i++) {
    if (lastMaxBuckets[i] > maxSound) {
      maxSound = lastMaxBuckets[i];
    }
  }
  
  float soundRatio = 0.5;
  if (minSound != maxSound) {
    soundRatio = (float) (sound - minSound) / (float) (maxSound - minSound);
  }
  for (int i = 0; i < 40; i++) {
    int j;
    if (i < CENTER_PIXEL) {
      j = CENTER_PIXEL - i - 1;
    } else {
      j = i - CENTER_PIXEL;
    }

    float ratio = ((float) j)/20.0;
    colorLeds[i] = CRGB::Black;
    whiteLeds[i] = CRGB::Black; 
    if (ratio < soundRatio) {
      colorLeds[i] = CHSV((j*255/30) % 255, 255, 255);
    } else {
      whiteLeds[i].r = 0x11;
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

typedef void (* Program)(float);
Program PROGRAMS[] = {hot_and_cold, rainbow_stripes, rainbow_mirror_stripes, white_mirror_comet, twinkle, uniform_rainbow, white_bar, red_bar, visualizer};

void loop() {
  // Respond to any pending messages
  handle_messages();

  // Send a time sync ping twice a second
  if (millis() - lastPing > 500) {
    send_ping();
    lastPing = millis();
  }

  if (millis() > 1000) {
    float t = ((float) (millis() + timeDelta)) / 1000.0;
    int num = sizeof(PROGRAMS)/ sizeof(PROGRAMS[0]);
    FastLED.setBrightness(((activeProgram % 3) + 1)*15);
    PROGRAMS[(activeProgram/3) % num](t);
  } else {
    showVoltage();
  }
  FastLED.show();
}