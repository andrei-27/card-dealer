#define ENABLE 5
#define DIRA 3
#define DIRB 4

void setup() {
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  analogWrite(ENABLE, 220);   // 220 of 255 = 86%

  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
  delay(5000);

  digitalWrite(DIRA, HIGH);   // forward
  digitalWrite(DIRB, LOW);
  delay(74);

  digitalWrite(DIRA, LOW);    // backward
  digitalWrite(DIRB, HIGH);
  delay(62);
}
   
