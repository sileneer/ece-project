#include <MeMCore.h>
MeBuzzer buzzer; // create the buzzer object
void celebrate() {
 // Each of the following "function calls" plays a single tone. 
 // The numbers in the bracket specify the frequency and the duration (ms)
 buzzer.tone(392, 200);
 buzzer.tone(523, 200);
 buzzer.tone(659, 200);
 buzzer.tone(784, 200);
 buzzer.tone(659, 150);
 buzzer.tone(784, 400);
 buzzer.noTone();
} 
void setup() {
 // Any setup code here runs only once:
} 
void loop() {
 // The main code here will run repeatedly (i.e., looping):
// celebrate(); // play the tune specified in the function celebrate()
// delay(1000); // pauses for 1000 ms before repeating the loop
}
