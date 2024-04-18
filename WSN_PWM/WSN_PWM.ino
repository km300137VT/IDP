#include <SoftwareSerial.h>

#define PWM_PIN 9
#define SUPPLY_VOLTAGE_PIN A1
#define OUTPUT_VOLTAGE_PIN A5
#define BATTERY_VOLTAGE_PIN A4

#define CLOCK_FREQUENCY 16000000
#define PWM_FREQUENCY 50000

#define OUTPUT_VOLTAGE_DIVIDER_SCALE 3.060606060606      //Should be *roughly* (330000 + 680000) / 330000 or whatever other voltage divider you use
#define OUTPUT_VOLTAGE_ACCEPTABLE_ERROR 0.05
#define PWM_KP 0.005
#define MIN_DUTY_CYCLE 0
#define MAX_DUTY_CYCLE 1

#define BATTERY_VOLTAGE_DIVIDER_SCALE 3.060606060606
#define SUPPLY_VOLTAGE_ACCEPTABLE_MINIMUM 3.0
#define BATTERY_CHARGING_CURRENT_mA 75         //Maximum is 150
#define BATTERY_CHARGING_RESISTOR_VALUE 10        //in Ohms
#define BATTERY_CHARGE_VOLTAGE_THRESHOLD 10.3
#define BATTERY_CHARGED_SUSTAIN_VOLTAGE 10.5
#define BATTERY_CHARGING_MINIMUM_DUTY_CYCLE 0.4

typedef enum {
  DISCHARGING,
  CHARGING,
  CHARGED  
} BatteryState;
BatteryState batteryState;

//PMW signal global variables
double dutyCycle;
double voltageTarget = 10.0;  //Initial, state machine implementation later
const int numOutputVoltageSamples = 5;
double outputVoltageSamples[numOutputVoltageSamples];
double supplyVoltageMeasured;
double voltageError;

//Thermistor global variables
#define ntc_pin A0                // Pin, to which the voltage divider is connected
#define vd_power_pin 2            // 5V for the voltage divider
#define nominal_resistance 10000  // Nominal resistance at 25⁰C
#define nominal_temeprature 25    // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5            // Number of samples
#define beta 3989                 // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000                // Value of  resistor used for the voltage divider
int samples = 0;                  // array to store the samples

// Define the RX and TX pins for the SoftwareSerial connection
// RX on pin 10, TX on pin 11
SoftwareSerial HM10(10, 11);

void setup() {
  //Pin configurations
  pinMode(SUPPLY_VOLTAGE_PIN, INPUT);   //Supply ADC, for determining charging mode
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);  //Battery ADC, used to monitor battery charging state
  pinMode(OUTPUT_VOLTAGE_PIN, INPUT);   //Boost Output ADC, for feedback to PWM signal
  pinMode(PWM_PIN, OUTPUT);             //PWM Pin, outputs to gate on MOSFET
  pinMode(vd_power_pin, OUTPUT);        //Thermistor 
  
  // HM10.write("SUPPLY VOLTAGE: ");
  // HM10.write(supplyVoltage);
  // HM10.write("STARTING DUTY CYCLE: ");
  // HM10.write(dutyCycle);


  //COM1A1 selects output comparison mode
  TCCR1A = _BV(COM1A1) | _BV(WGM11); 
  //WGM 11 and 13 are intentionally split across two registers as that's the way the microcontroller works, they set the Wave Generation Mode
  //CS10 sets the clock prescaler to 1, cancelling it out
  TCCR1B = _BV(CS10) | _BV(WGM13);

  //ICR1 stores the "top" value of the counter; when it reaches this value, it flips around and starts counting downward.
  ICR1 = (CLOCK_FREQUENCY / (2 * PWM_FREQUENCY));

  dutyCycle = 0;  //Set to 0 (converter off) by default, will be incremented during the main loop
  //OCR1A Stores the counter threshold; when the timer reaches this threshold, if it's counting upward it toggles the output on, and
  //if it's counting downward and drops below it it turns the output off.
  OCR1A = ICR1 * dutyCycle;

  /* 
  The logic of the timer works like this:
  - Begin timer register (internal) at 0
  - Count upward by 1 each clock cycle
  - When the timer reaches the threshold value in OCR1A, it toggles the output pin on
  - When it reaches the top value stored in ICR1, it starts counting downward
  - Once the counter drops below the OCR1A threshold again, it toggles the output pin off
  - Finally, when the counter hits 0, it begins counting upward again, repeating the cycle

  Thus, the frequency can be calculated by 16,000,000 (the clock frequency) / ICR1 / 2 (since it counts up and then back down instead of resetting at the top)
  and the duty cycle can be calculated by OCR1A / ICR1 (threshold divided by top value)
  */

  //Now repeat the above but without the output comparison or middle value, we simply need 
  //an interrupt flag thrown every 30 seconds when in sleep mode so we'll use Timer0
  TCCR0A = _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS02) | _BV(CS00);
  TIMSK0 = TOIE1;
  OCR0A = 255;  //Maximum value for 8 bit output compare register


  //Initialize voltage sample array
  for (int i = 0; i < numOutputVoltageSamples; i++) outputVoltageSamples[i] = 0;

  //Bluetooth code 
  // Start communication with the HM10 Bluetooth module at 9600 baud
  HM10.begin(9600);
  // Start the serial communication with the computer at 9600 baud
  Serial.begin(9600);
}

void loop() {

  double outputVoltageSampleSum = 0;
  bool outputVoltageReadingsInitialized = true;
  for (int j = numOutputVoltageSamples - 1; j > 0; j--) {
    outputVoltageSamples[j] = outputVoltageSamples[j-1];                          //Shift all voltage readings one index right, deleting oldest value
    if (outputVoltageSamples[j] == 0) outputVoltageReadingsInitialized = false;   //Zero check, don't update duty cycle if average is dragged down by zeros
    outputVoltageSampleSum += outputVoltageSamples[j];
  }
  outputVoltageSamples[0] = analogRead(OUTPUT_VOLTAGE_PIN);
  outputVoltageSampleSum += outputVoltageSamples[0];
  double outputVoltage = (outputVoltageSampleSum/numOutputVoltageSamples) * OUTPUT_VOLTAGE_DIVIDER_SCALE / 1023.0f * 5.0f;
  double batteryVoltage = analogRead(BATTERY_VOLTAGE_PIN) * BATTERY_VOLTAGE_DIVIDER_SCALE / 1023.0f * 5.0f;
  double supplyVoltage = analogRead(SUPPLY_VOLTAGE_PIN) / 1023.0f * 5.0f;

  if (supplyVoltage < SUPPLY_VOLTAGE_ACCEPTABLE_MINIMUM) {
    batteryState = DISCHARGING;
  } else if(batteryVoltage > BATTERY_CHARGE_VOLTAGE_THRESHOLD) {
    batteryState = CHARGED;
  } else {
    batteryState = CHARGING;
  }
  //Battery charge management
  switch(batteryState) {
  case DISCHARGING:
    Serial.print("State: DISCHARGING");
    voltageTarget = 0;
    dutyCycle = 0;
    break;
  case CHARGING:
    if (dutyCycle < BATTERY_CHARGING_MINIMUM_DUTY_CYCLE) dutyCycle = BATTERY_CHARGING_MINIMUM_DUTY_CYCLE + 0.05;
    voltageTarget = batteryVoltage + (BATTERY_CHARGING_CURRENT_mA*BATTERY_CHARGING_RESISTOR_VALUE / 1000.0f);
    Serial.print("State: CHARGING");
    break;
  case CHARGED:
    voltageTarget = BATTERY_CHARGED_SUSTAIN_VOLTAGE;
    Serial.print("State: CHARGED");
    break;
  }

  if (outputVoltageReadingsInitialized) {   //Only start modifying stuff once the average is set, gives time for steady state response to stabilize
    if (voltageTarget != 0) {
      voltageError = voltageTarget - outputVoltage;
    } else {
      voltageError = 0;
    }

    Serial.print(" | Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print(" | Output Voltage: ");
    Serial.print(outputVoltage);
    Serial.print(" | Voltage Target: ");
    Serial.print(voltageTarget);
    Serial.print(" | Current Voltage Error: ");
    Serial.print(voltageError);

    if (voltageError > OUTPUT_VOLTAGE_ACCEPTABLE_ERROR || voltageError < -OUTPUT_VOLTAGE_ACCEPTABLE_ERROR) {
      if (dutyCycle + PWM_KP * voltageError > MIN_DUTY_CYCLE && dutyCycle + PWM_KP * voltageError < MAX_DUTY_CYCLE) {
        dutyCycle += voltageError * PWM_KP;
        OCR1A = ICR1 * dutyCycle;
        
        Serial.print(" | Adjusting duty cycle to: ");
        Serial.println(dutyCycle);
      } else {
        Serial.println(" | Converter tried to adjust duty cycle but fell outside bounds.");
      }
    } else {
      Serial.println(" | Converter output nominal.");
    }
  }
  delay(25);
  HM10.write("");

  //THERMISTOR READING
  uint8_t i;
  float average;
  samples = 0;

  // take voltage readings from the voltage divider
  digitalWrite(vd_power_pin, HIGH);
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead(ntc_pin);
    delay(10);
  }

  digitalWrite(vd_power_pin, LOW);
  average = 0;
  average = samples / samplingrate;
  HM10.write("\n \n");
  HM10.write("\nADC readings  ");
  HM10.write(average);

  // Calculate NTC resistance
  average = samples / samplingrate;
  average = Rref * (1023.0 / average - 1.0);
  HM10.write("\nThermistor resistance ");
  HM10.write(average);
  float temperature;
  temperature = average / nominal_resistance;     // (R/Ro)
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= beta;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C
  HM10.write("\n Temperature ");
  HM10.write(temperature);
  HM10.write(" *C\n");

   // Check if data is available to read from the HM10 module
  if (HM10.available()) {
    // Read the data from the HM10 module
    char btData = HM10.read();
    // Send the data to the serial monitor
    Serial.write(btData);
  }
  
  delay(2000);
}
