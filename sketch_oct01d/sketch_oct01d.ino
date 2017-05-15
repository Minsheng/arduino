int seconds;
int minutes;

void setup() {
  seconds = 0;
  minutes = 0;
  Serial.begin(9600);
}

void loop() {
  Serial.println(minutes);
  Serial.print(":");
  Serial.println(seconds);
  delay(1000);
  seconds++;
  if (minutes == 60) {
    minutes++;
    seconds = 0;
  }
}
