#define PWM_PIN 9
#define SUPPLY_VOLTAGE_PIN A1
#define OUTPUT_VOLTAGE_PIN A5

#define CLOCK_FREQUENCY 16000000
#define PWM_FREQUENCY 50000

#define SUPPLY_VOLTAGE_ACCEPTABLE_MINIMUM 3.0
#define OUTPUT_VOLTAGE_DIVIDER_SCALE 3.43      //Should be *roughly* (330000 + 680000) / 330000 or whatever other voltage divider you use
#define OUTPUT_VOLTAGE_ACCEPTABLE_ERROR 0.05
#define PWM_KP 0.005

#define MIN_DUTY_CYCLE 0
#define MAX_DUTY_CYCLE 1

//PMW signal global variables
double dutyCycle;
double voltageTarget = 10.0;  //Initial, state machine implementation later
const int numOutputVoltageSamples = 5;
double voltageOutputSamples[numOutputVoltageSamples];

//Thermistor global variables
#define ntc_pin A0                // Pin, to which the voltage divider is connected
#define vd_power_pin 2            // 5V for the voltage divider
#define nominal_resistance 10000  // Nominal resistance at 25⁰C
#define nominal_temeprature 25    // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5            // Number of samples
#define beta 3989                 // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000                // Value of  resistor used for the voltage divider
int samples = 0;                  // array to store the samples

void setup() {
  Serial.begin(9600);  // initialize serial communication at a baud rate of 9600

  pinMode(SUPPLY_VOLTAGE_PIN, INPUT);
  pinMode(OUTPUT_VOLTAGE_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);

  //Do preliminary duty cycle setting, will be close loop incremented towards more accurately during runtime
  double supplyVoltage = analogRead(SUPPLY_VOLTAGE_PIN) / 1023.0f * 5.0f;
  if (supplyVoltage > SUPPLY_VOLTAGE_ACCEPTABLE_MINIMUM) {
    dutyCycle = (voltageTarget - supplyVoltage) / voltageTarget;
  } else {
    dutyCycle = 0;
    Serial.println("SUPPLY TOO LOW, CONVERTER DISABLED");
  }
  
  Serial.print("SUPPLY VOLTAGE: ");
  Serial.println(supplyVoltage);
  Serial.print("STARTING DUTY CYCLE: ");
  Serial.println(dutyCycle);


  //COM1A1 selects output comparison mode
  TCCR1A = _BV(COM1A1) | _BV(WGM11); 
  //WGM 11 and 13 are intentionally split across two registers as that's the way the microcontroller works, they set the Wave Generation Mode
  //CS10 sets the clock prescaler to 1, cancelling it out
  TCCR1B = _BV(CS10) | _BV(WGM13);

  //ICR1 stores the "top" value of the counter; when it reaches this value, it flips around and starts counting downward.
  ICR1 = (CLOCK_FREQUENCY / (2 * PWM_FREQUENCY));

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

  //Initialize voltage sample array
  for (int i = 0; i < numOutputVoltageSamples; i++) voltageOutputSamples[i] = 0;

  ///THERMISTOR CODE
  pinMode(vd_power_pin, OUTPUT);
}

// int counter = 0;
// int increment = 1;

void loop() {
  //Tech Demo cycle code
  // dutyCycle = counter / 100.0f;
  // OCR1A = ICR1 * dutyCycle;
  // if (counter == 70) increment = -1;
  // if (counter == 50) increment = 1;
  // counter += increment;
  double voltageSampleSum = 0;
  bool voltageReadingsInitialized = true;
  for (int j = numOutputVoltageSamples - 1; j > 0; j--) {
    voltageOutputSamples[j] = voltageOutputSamples[j-1];                    //Shift all voltage readings one index right, deleting oldest value
    if (voltageOutputSamples[j] == 0) voltageReadingsInitialized = false;   //Zero check, don't update duty cycle if average is dragged down by zeros
    voltageSampleSum += voltageOutputSamples[j];
  }
  voltageOutputSamples[0] = analogRead(OUTPUT_VOLTAGE_PIN);
  voltageSampleSum += voltageOutputSamples[0];
  // Serial.print("Current Duty Cycle: ");
  // Serial.println(dutyCycle);

  if (voltageReadingsInitialized) {   //Only start modifying stuff once the average is set, gives time for steady state response to stabilize
    double outputVoltage = (voltageSampleSum/numOutputVoltageSamples) * OUTPUT_VOLTAGE_DIVIDER_SCALE / 1023.0f * 5.0f;
    double voltageError = voltageTarget - outputVoltage;

    Serial.print("Current Output Voltage: ");
    Serial.print(outputVoltage);
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
  Serial.println("");

  // uint8_t i;
  // float average;
  // samples = 0;

  // // take voltage readings from the voltage divider
  // digitalWrite(vd_power_pin, HIGH);
  // for (i = 0; i < samplingrate; i++) {
  //   samples += analogRead(ntc_pin);
  //   delay(10);
  // }

  // digitalWrite(vd_power_pin, LOW);
  // average = 0;
  // average = samples / samplingrate;
  // Serial.println("\n \n");
  // Serial.print("ADC readings ");
  // Serial.println(average);

  // // Calculate NTC resistance
  // average = samples / samplingrate;
  // average = Rref * (1023.0 / average - 1.0);
  // Serial.print("Thermistor resistance ");
  // Serial.println(average);
  // float temperature;
  // temperature = average / nominal_resistance;     // (R/Ro)
  // temperature = log(temperature);                  // ln(R/Ro)
  // temperature /= beta;                   // 1/B * ln(R/Ro)
  // temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
  // temperature = 1.0 / temperature;                 // Invert
  // temperature -= 273.15;                         // convert absolute temp to C
  // Serial.print("Temperature ");
  // Serial.print(temperature);
  // Serial.println(" *C");
  // delay(2000);
}
