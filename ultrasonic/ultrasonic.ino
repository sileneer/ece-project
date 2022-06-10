#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12
// If you are using Port 1 of mCore, the ultrasonic sensor uses digital pin 12
// If you are using Port 2 of mCore, the ultrasonic sensor uses digital pin 10
void setup() {
 Serial.begin(9600); // to initialize the serial monitor
}
void loop() {
 pinMode(ULTRASONIC, OUTPUT);
 digitalWrite(ULTRASONIC, LOW);
 delayMicroseconds(2);
 digitalWrite(ULTRASONIC, HIGH);
 delayMicroseconds(10);
 digitalWrite(ULTRASONIC, LOW);
 
 pinMode(ULTRASONIC, INPUT);
 long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
 if (duration > 0) {
 Serial.print("distance(cm) = ");
 Serial.println(duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100);
 }
 else {
 Serial.println("out of range");
 }
 delay(500);
}
