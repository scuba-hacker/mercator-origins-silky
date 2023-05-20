// Slinky is the audio module for Mercator Origins
// This is going to turn into the code for running on the Beetle ESP32-C3 (Risc-V Core Development Board)
// https://www.dfrobot.com/product-2566.html

int led = 10;
void setup() {
  pinMode(led,OUTPUT);
}

void loop() {
  digitalWrite(led,HIGH);
  delay(2000);
  digitalWrite(led,LOW);
  delay(2000);
}
