// GPS_PPS_Sim
// Justin Gravett (ESAero), 8/22/19
//
// Simple program to generate a GPS PPS signal with
// frequency of 1 Hz and 50% duty cycle
//

const int output_pin = 23;
const int led_pin = 13;

void setup() {
  analogWriteFrequency(output_pin, 1);
  analogWrite(output_pin, 127);

  pinMode(led_pin, OUTPUT);

  digitalWrite(led_pin, HIGH);
}

void loop() {}
