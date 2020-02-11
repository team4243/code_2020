void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(5000);
}

byte val = 0;
bool reverse = false;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("#");
  Serial.print(val);
  Serial.println();
  if (reverse)
  {
    val -= 1;
    if (val == 0) reverse = false;
  }
  else
  {
    val += 1;
    if (val == 100) reverse = true;
  }

  delay(100);
}
