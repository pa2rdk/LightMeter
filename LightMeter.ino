#include "Arduino.h"
#include <math.h>
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "LowPower.h"
#include <TimerOne.h>
#include <ClickEncoder.h>

#define   M_LOG2E   1.4426950408889634074
#define   OLED_RESET 4
#define   SSD1306_128_32
#define   lcdPower 10

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

byte iso_c = 0;
byte f_c = 0;
float speedx;
float ev = 2;
float ev2;
int filmSpeed;
float diafragma;
unsigned int voltage = 0;
boolean batteryInfo = false;
long poweroffTimer = 0;
unsigned int powersaveAmount = 20000;
byte setMenu = 0;
float theLux = 0;
int shutterspeedDigits = 0;
int shutterspeedSelector = 10;

const unsigned int PROGMEM isoArray[] = {100, 200, 400, 800, 1600, 3200};
const float fArray[] = {1.4, 2, 2.8, 5.6, 8, 11, 16, 22};
const unsigned int PROGMEM shutterspeedArray[] = {
  3000, 2000, 1500, 1000, 800, 500, 400, 250, 200, 125, 100, 50, 25, 10, 8, 5, 4, 2, // 18 Fraction of a second
  1, 2, 3, 4, 5, 8, 10, 15, 20, 30, 45, // 16 Over a second
  1, 2, 3, 4, 5, 10, 15, 20, 30, 45, 60, // Over a minute
  1, 2, 4, 8, 12, 16, 18, 24, 25, 36, 48, 50, 64, 72, 128, 256, 300 // 17 fps
};

int16_t theClick, held, timesAround;

ClickEncoder *encoder;
int16_t rawValue, debounceEncoder; //, last;

void timerIsr() {
  encoder->service();
}

static const unsigned char PROGMEM battery_full [] =
{ 0x7F, 0xE0, 0x40, 0x20, 0x5F, 0xB8, 0x5F, 0xB8, 0x5F, 0xB8, 0x5F, 0xB8, 0x40, 0x20, 0x7F, 0xE0 };

static const unsigned char PROGMEM battery_threequarters [] =
{ 0x7F, 0xE0, 0x40, 0x20, 0x5C, 0x38, 0x5E, 0x38, 0x5E, 0x38, 0x5F, 0x38, 0x40, 0x20, 0x7F, 0xE0 };

static const unsigned char PROGMEM battery_half [] =
{ 0x7F, 0xE0, 0x40, 0x20, 0x58, 0x38, 0x58, 0x38, 0x5C, 0x38, 0x5C, 0x38, 0x40, 0x20, 0x7F, 0xE0 };

static const unsigned char PROGMEM battery_low [] =
{ 0x7F, 0xE0, 0x40, 0x20, 0x50, 0x38, 0x50, 0x38, 0x50, 0x38, 0x50, 0x38, 0x40, 0x20, 0x7F, 0xE0 };

static const unsigned char PROGMEM battery_empty [] =
{ 0x7F, 0xE0, 0x40, 0x20, 0x40, 0x38, 0x40, 0x38, 0x40, 0x38, 0x40, 0x38, 0x40, 0x20, 0x7F, 0xE0 };

void displaySensorDetails(void) {
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void wakeUp() {

}


void configureSensor(void) {
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}


// end of setup

// end of loop

//void lightSensor () {
//  //  configureSensor();
//  uint32_t lum = tsl.getFullLuminosity();
//  uint16_t ir, full;
//  ir = lum >> 16;
//  full = lum & 0xFFFF;
//  thelux = tsl.calculateLux(full, ir);
//  thelux *= 4.25;  // x4.25 for Calbration
//#ifdef doDebug
//  Serial.print(F("Light:"));
//  Serial.println(thelux);
//#endif
//}

void lightSensor() {
  sensors_event_t event;
  tsl.getEvent(&event);
  theLux = event.light;
}

void readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(1); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  voltage = (high << 8) | low;
  voltage = 1125300L / voltage; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
#ifdef doDebug
  Serial.println(voltage);
#endif
}

void printVcc() {
  if (voltage != 0) {
    Serial.println(voltage);
    if (batteryInfo) {
      display.setCursor(90, 0);
      display.print(voltage);
    }

    if (voltage >= 4050) {
      display.drawBitmap(115, 0, battery_full, 16, 8, 1);
    } else if (voltage >= 3800 && voltage < 4050) {
      display.drawBitmap(115, 0, battery_threequarters, 16, 8, 1);
    } else if (voltage >= 3500 && voltage < 3800) {
      display.drawBitmap(115, 0, battery_half, 16, 8, 1);
    } else if (voltage >= 3100 && voltage < 3500) {
      display.drawBitmap(115, 0, battery_low, 16, 8, 1);
    } else if (voltage < 3100) {
      display.drawBitmap(115, 0, battery_empty, 16, 8, 1);
    }
  }
}
void xprintdisplay() {
  readVcc();
  lightSensor();
  float ev_ = (log(theLux / 2.5)) / (log(2));
  ev = ev_;
  int lgt = (int)theLux;

  filmSpeed = pgm_read_word_near(isoArray+iso_c);
  diafragma = fArray[f_c];
  int ev2 = (ev + (log(filmSpeed / 100)) / (log(2)));
  speedx = (pow(diafragma, 2) / pow (2, ev2));

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  if (theLux < 65000) {
    display.print(lgt);
  } else {
    display.print(F("max"));
  }
  display.println(F(" Lux"));
  display.setTextSize(1);
  display.print(F("ISO:"));
  display.print(filmSpeed);
  if (setMenu == 1) display.print(F("*"));

  display.setCursor(64, 16);
  display.print(F(" F#:"));
  display.print(diafragma);
  if (setMenu == 2) display.print(F("*"));

  display.setCursor(0, 24);
  display.print(F(" EV:"));
  display.print(ev);

  if (speedx < 1) {
    //I want to turn this into a function
    display.setCursor(58, 24);
    display.print(" 1/");
    display.print(1 / speedx);
  } else {
    display.print(speedx);
  }
  printVcc();
  display.display();

  Serial.print(F("LUX:"));
  Serial.print(theLux);
  Serial.print("  ISO");
  Serial.print(iso_c);
  Serial.print("--");
  Serial.print(filmSpeed);
  Serial.print("  F:");
  Serial.print(f_c);
  Serial.print("--");
  Serial.print(diafragma);
  Serial.print(" EV:");
  Serial.print(ev);
  Serial.print("..");
  Serial.print(ev2);
  Serial.print(" Speed:");
  Serial.println(speedx);
}

void setup ()
{
  Serial.begin (9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setCursor(6, 0);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println(F("   RDK"));
  display.print(F("LightMeter"));
  display.display();
  delay(1000);
  Serial.println("Light Sensor Test"); Serial.println("");

  encoder = new ClickEncoder(A1, A0, 3, 4);
  encoder->setAccelerationEnabled(false);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  /* Initialise the sensor */
  if (!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  displaySensorDetails();
  configureSensor();
  poweroffTimer = millis();
}
void loop () {
  rawValue = encoder->getValue();
  if (rawValue != 0) {
    Serial.println(rawValue);

    if (rawValue == 1) {
      if (setMenu == 1) {
        iso_c++;
        if (iso_c > 5) iso_c = 0;
      }
      if (setMenu == 2) {
        f_c++;
        if (f_c > 7) f_c = 0;
      }
    }
    if (rawValue == -1) {
      if (setMenu == 1) {
        iso_c--;
        if (iso_c > 5) iso_c = 4;
      }
      if (setMenu == 2) {
        f_c--;
        if (f_c > 7) f_c = 6;
      }
    }
    poweroffTimer = millis();
  }

  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Pressed:
        break;
      case ClickEncoder::Held:
        break;
      case ClickEncoder::Released:
        break;
      case ClickEncoder::Clicked:
        setMenu++;
        if (setMenu > 2) setMenu = 0;
        break;
    }
    poweroffTimer = millis();
    debounceEncoder = 0;
  }

  xprintdisplay();

  long currentTime = millis();
  if (currentTime >= (poweroffTimer + powersaveAmount)) {
    display.clearDisplay();
    display.display();
    digitalWrite(lcdPower, LOW);
    //  eepromWriteSpeeds(); // write ISO and shutter speeds to EEPROM
    delay(100);
    attachInterrupt(1, wakeUp, FALLING);
    attachInterrupt(digitalPinToInterrupt(3), wakeUp, FALLING);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(1);
    digitalWrite(lcdPower, HIGH);
    delay(100);
    readVcc();
    configureSensor();
    delay(10);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    poweroffTimer = millis();
    debounceEncoder = 0;
  }
  delay(1000);
}
