 void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}
void loop() {
  int val1 = analogRead(A0);
  int val2 = analogRead(A1);
  int val3 = analogRead(A2);
  int val4 = analogRead(A3);

  // Serial.println(val);
  if (val1 < 400){
    Serial.print("AP1 : OFF, ");
    digitalWrite(8, LOW);
  }else{
    Serial.print("AP1 : ON, ");
    digitalWrite(8, HIGH);
  }
  if (val2 < 400){
    Serial.print("AP2 : OFF, ");
    digitalWrite(9, LOW);
  }else{
    Serial.print("AP2 : ON, ");
    digitalWrite(9, HIGH);
  }
  if (val3 < 400){
    Serial.print("BP1 : OFF, ");
    digitalWrite(10, LOW);
  }else{
    Serial.print("BP1 : ON, ");
    digitalWrite(10, HIGH);
  }
  if (val4 < 400){
    Serial.println("BP2 : OFF");
    digitalWrite(11, LOW);
  }else{
    Serial.println("BP2 : ON");
    digitalWrite(11, HIGH);
  }

  delay(100);
}