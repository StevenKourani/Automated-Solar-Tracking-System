/**
 * @file automated_solar_tracking.ino
 * @authors Blake, Steven, Garrett
 * @date 4/7/2025
 * 
 *  Revisions/Comments/TODOs/FIXMEs
 *  04/07/2025    -   Initial Creation, Create setup, loop, checkError, repositionPanel (Blake)
 *  04/07/2025    -   TODO: Look into reading voltage of battery/battery charge amount so we can demo and show battery charging (Blake)
 *  04/07/2025    -   FIXME: The servo GPIO pin macros are wrong, need to pick 2 D pins to use.
 */
#include <stdint.h>
#include <Servo.h>

/* MODE Defines */
#define DEBUG_MODE 1            // Prints out debug info to serial monitor (errors & angles)
#define DEBUG_MODE_VERBOSE 1    // Prints out extra debug info to serial monitor if DEBUG_MODE == 1 (LDR values)

/* GPIO DEFINES */
#define AZIMUTH_SERVO_PIN 0
#define ALTITUDE_SERVO_PIN 0

#define LDR_TL_PIN A0
#define LDR_TR_PIN A1
#define LDR_BL_PIN A2
#define LDR_BR_PIN A3

/* LDR Defines */
#define LDR_COUNT 4
#define LDR_TL 0
#define LDR_TR 1
#define LDR_BL 2
#define LDR_BR 3

/* Servo Defines */


/* LDR Global Vars */
uint16_t ldr_array[LDR_COUNT];

/* Servo Global Vars */
Servo altitude_servo;
Servo azimuth_servo;

int16_t altitude_error;
int16_t azimuth_error;

void setup() {
  #if DEBUG_MODE
    Serial.begin(9600); // Start serial so we can print to ArduinoIDE serial viewer
  #endif
  
  // LDR Setup
  pinMode(LDR_TL_PIN, INPUT);
  pinMode(LDR_TR_PIN, INPUT);
  pinMode(LDR_BL_PIN, INPUT);
  pinMode(LDR_BR_PIN, INPUT);

  // Servo Setup
  altitude_servo.attach(ALTITUDE_SERVO_PIN);
  azimuth_servo.attach(AZIMUTH_SERVO_PIN);

  // Initial Angles. Necessary?
  altitude_servo.write(0);
  azimuth_servo.write(0);

  // Movement delay time.
  delay(10);
}

void loop() {
  // According to our FSM/Feedback loop
  while (abs(altitude_error) > 1 || abs(azimuth_error) > 2) {   /*We need to figure out this*/
    checkError(&altitude_error, &azimuth_error);
    repositionPanel(altitude_error, azimuth_error);
  }

  // Go to idle for some time
  delay(1000);  // delay 1s
  // ideally we change this to
  //  1. Set timer for 1hr (is this even possible without external RTC)
  //  2. Deep sleep mode waiting for interrupt

}


void ldr_array_read(uint16_t *ldr_array) {
  *(ldr_array + LDR_TL) = analogRead(LDR_TL_PIN);
  *(ldr_array + LDR_TR) = analogRead(LDR_TR_PIN);
  *(ldr_array + LDR_BL) = analogRead(LDR_BL_PIN);
  *(ldr_array + LDR_BR) = analogRead(LDR_BR_PIN);
}

void checkError(int16_t *altitude_error, int16_t *azimuth_error) {
  ldr_array_read(ldr_array);

  // Possibly a better algorithm for this? I think max works well because there is 2 LDRs per direction, but depending on the diagional
  // aspect of the sun, one of the LDRs for each direction will always see the sun better and the other will not be able to tell us much.

  // To compute the altitude error, lets take the maximum brightness from the top/bottom photoresistors 
  // our error will be TOP - BOTTOM, since a brighter light means a lower resistance thus higher voltage then,
  //     a + altitude error means we need to move UP since top photoresistor had brigher light
  //     a - altitude error means we need to move DOWN since bottom photoresistor had a brigher light
  *altitude_error = max(ldr_array[LDR_TL], ldr_array[LDR_TR]) - max(ldr_array[LDR_BL], ldr_array[LDR_BR]);

  // Do something similar for azimuth
  // our error will be RIGHT - LEFT (as if you are the solar panel facing the sun)
  //     a + azimuth error means we need to move RIGHT since right photoresistor had brigher light
  //     a - azimuth error means we need to move LEFT since left photoresistor had a brigher light
  *azimuth_error = max(ldr_array[LDR_TR], ldr_array[LDR_BR]) - max(ldr_array[LDR_TL], ldr_array[LDR_BL]);

  #if DEBUG_MODE
    #if DEBUG_MODE_VERBOSE
      Serial.print("LDR TR:\t\t"); Serial.println(ldr_array[LDR_TR]);
      Serial.print("LDR TL:\t\t"); Serial.println(ldr_array[LDR_TL]);
      Serial.print("LDR BR:\t\t"); Serial.println(ldr_array[LDR_BR]);
      Serial.print("LDR BL:\t\t"); Serial.println(ldr_array[LDR_BL]);
    #endif

    Serial.print("Altitude Error:\t\t"); Serial.println((int)altitude_error);
    Serial.print("Azimuth Error:\t\t"); Serial.println((int)azimuth_error);
  #endif

}

void repositionPanel(int16_t altitude_error, int16_t azimuth_error) {

  // need to think about how to compute the new angle based on the error, should the increase/decrease in angle be reletive to the error amount
  // also what do we do if for azimuth new_angle > 180 or < 0? we would need to swing both azimuth and altitude to get the other 180* of azimuth.
  // altitude angle should never be > 180 or < 0 because sun rises and sets around the angle of the horizon which is a "line" so 180* max angle.
  uint8_t old_altitude_angle = (uint8_t)altitude_servo.read();
  uint8_t old_azimuth_angle = (uint8_t)azimuth_servo.read();
  uint8_t new_altitude_angle = old_altitude_angle;
  uint8_t new_azimuth_angle = old_azimuth_angle;


  if (altitude_error > 0) {
    
  } else {

  }

  if (azimuth_error > 0) {

  } else {

  }

  #if DEBUG_MODE
    Serial.print("New Altitude:\t\t"); Serial.println(new_altitude_angle);
    Serial.print("New Azimuth:\t\t"); Serial.println(azimuth_error);
  #endif

  altitude_servo.write(new_altitude_angle);
  azimuth_servo.write(new_azimuth_angle);

  delay(15); // wait for servos to move into proper position.
}