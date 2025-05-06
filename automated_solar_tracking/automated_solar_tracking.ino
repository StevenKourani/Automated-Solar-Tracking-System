/**
 * @file automated_solar_tracking.ino
 * @authors Blake, Steven, Garrett
 * @date 04/07/2025 (updated 05/02/2025)
 * 
 * 
 */

#include <Servo.h>

/* DEBUG Defines */
#define DEBUG_MODE         1  
#define DEBUG_MODE_VERBOSE 1

/* GPIO Defines */
#define AZIMUTH_SERVO_PIN   6
#define ALTITUDE_SERVO_PIN  9

#define LDR_TL_PIN A0
#define LDR_TR_PIN A1
#define LDR_BL_PIN A2
#define LDR_BR_PIN A3

/* LDR Defines */
#define LDR_COUNT 4
#define LDR_TL    0
#define LDR_TR    1
#define LDR_BL    2
#define LDR_BR    3

// PD Gains
const float Kp_az  = 0.05;
const float Kd_az  = 0.005;
const float Kp_alt = 0.05;
const float Kd_alt = 0.005;

uint16_t ldr_array[LDR_COUNT];
int16_t altitude_error = 0, azimuth_error = 0;
int16_t prevAltError  = 0, prevAzError   = 0;
Servo altitude_servo, azimuth_servo;

// Interrupt based timer
volatile uint16_t tickCounter = 0;
volatile bool oneSecondElapsed = false;
unsigned long secondsCounter = 0;
const unsigned long intervalSeconds = 1200; //20 min
bool timerActive = false; // disable timer for demo

void setup() {
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif

  pinMode(LDR_TL_PIN, INPUT);
  pinMode(LDR_TR_PIN, INPUT);
  pinMode(LDR_BL_PIN, INPUT);
  pinMode(LDR_BR_PIN, INPUT);

  altitude_servo.attach(ALTITUDE_SERVO_PIN);
  azimuth_servo.attach(AZIMUTH_SERVO_PIN);

  // Run calibration
  altitude_servo.write(90);
  azimuth_servo.write(90);
  
  delay(5000);

  setupTimer2();
}

void loop() {

  if (timerActive && oneSecondElapsed) {
    oneSecondElapsed = false;
    secondsCounter++;

    if (secondsCounter >= intervalSeconds) {
      timerActive = false;

      checkError(&altitude_error, &azimuth_error);
      repositionPanel(altitude_error, azimuth_error);
      #if DEBUG_MODE
      Serial.println(F("-------------------------"));
      #endif

      secondsCounter = 0;
      timerActive = true;
    }
  } else if (!timerActive) {
      checkError(&altitude_error, &azimuth_error);
      repositionPanel(altitude_error, azimuth_error);
      #if DEBUG_MODE
      Serial.println(F("-------------------------"));
      #endif
  }
}

void ldr_array_read(uint16_t *arr) {
  arr[LDR_TL] = analogRead(LDR_TL_PIN);
  arr[LDR_TR] = analogRead(LDR_TR_PIN);
  arr[LDR_BL] = analogRead(LDR_BL_PIN);
  arr[LDR_BR] = analogRead(LDR_BR_PIN);
}

void checkError(int16_t *altErr, int16_t *aziErr) {
  ldr_array_read(ldr_array);

  #if DEBUG_MODE
    Serial.print(F("LDR raw TL:")); Serial.print(ldr_array[LDR_TL]);
    Serial.print(F(" TR:"));     Serial.print(ldr_array[LDR_TR]);
    Serial.print(F(" BL:"));     Serial.print(ldr_array[LDR_BL]);
    Serial.print(F(" BR:"));     Serial.println(ldr_array[LDR_BR]);
  #endif

  //averages 
  int topAvg   = (ldr_array[LDR_TL] + ldr_array[LDR_TR]) / 2;
  int botAvg   = (ldr_array[LDR_BL] + ldr_array[LDR_BR]) / 2;
  int leftAvg  = (ldr_array[LDR_TL] + ldr_array[LDR_BL]) / 2;
  int rightAvg = (ldr_array[LDR_TR] + ldr_array[LDR_BR]) / 2;

  // error
  *altErr = topAvg - botAvg;    // tilting up and down
  *aziErr = rightAvg - leftAvg; // tilting left and right

  #if DEBUG_MODE
    Serial.print(F("Error Alt: ")); Serial.print(*altErr);
    Serial.print(F("  Azimuth: "));  Serial.println(*aziErr);
  #endif
}

void repositionPanel(int16_t altErr, int16_t aziErr) {
  int16_t oldAltAng = altitude_servo.read();
  int16_t oldAziAng = azimuth_servo.read();

  // derivative 
  int16_t dAlt = altErr - prevAltError;
  int16_t dAzi = aziErr - prevAzError;

  // PD outputs
  float outAlt = Kp_alt * altErr + Kd_alt * dAlt;
  float outAzi = Kp_az  * aziErr + Kd_az  * dAzi;

  // angle deltas
  int16_t deltaAlt = (int16_t)round(outAlt);
  int16_t deltaAzi = (int16_t)round(outAzi);

  // Logic for backwards movement
  deltaAzi = (oldAltAng < 90) ? -1 * deltaAzi : deltaAzi;

  // new angles
  int16_t newAlt = constrain(oldAltAng + deltaAlt,  0, 180);
  int16_t newAzi = constrain(oldAziAng + deltaAzi, 0, 180);


  #if DEBUG_MODE
    Serial.print(F("Servo→ Altitude: ")); Serial.print(newAlt);
    Serial.print(F("  Azimuth: "));       Serial.println(newAzi);
  #endif

  altitude_servo.write(newAlt);
  azimuth_servo.write(newAzi);

  prevAltError = altErr;
  prevAzError  = aziErr;

  delay(500);
}

void setupTimer2() {
  noInterrupts();
  TCCR2A = 0;
  TCCR2B = 0;

  // Set CTC mode
  TCCR2A |= (1 << WGM21);

  // Set compare match value for ~2ms tick (500 Hz)
  OCR2A = 249; // (16MHz / 128 / (249 + 1)) = ~500 Hz

  // Enable interrupt on match
  TIMSK2 |= (1 << OCIE2A);

  // Set prescaler to 128 and start Timer2
  TCCR2B |= (1 << CS22) | (1 << CS20); // CS22 + CS20 = 128 prescaler
  interrupts();
}

ISR(TIMER2_COMPA_vect) {
  tickCounter++;
  if (tickCounter >= 500) { // 500 ticks = 1 second
    tickCounter = 0;
    oneSecondElapsed = true;
  }
}
