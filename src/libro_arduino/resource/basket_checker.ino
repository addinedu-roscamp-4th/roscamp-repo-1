void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
}
void loop() {
  int val1 = analogRead(A0);
  int val2 = analogRead(A1);
  // Serial.println(val);
  if (val1 < 400){
    Serial.print("B1 : OFF, ");
    digitalWrite(8, LOW);
  }else{
    Serial.print("B1 : ON, ");
    digitalWrite(8, HIGH);
  }
  if (val2 < 400){
    Serial.println("B2 : OFF");
    digitalWrite(9, LOW);
  }else{
    Serial.println("B2 : ON");
    digitalWrite(9, HIGH);
  }
}