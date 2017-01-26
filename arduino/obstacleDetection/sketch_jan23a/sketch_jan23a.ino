int IR_VIN = 2;
int IR_VOUT = 3;
 
void setup()
{
  pinMode(IR_VIN, OUTPUT);
  pinMode(IR_VOUT, INPUT);
  Serial.begin(9600);
}
 
void loop()
{  
  int range = getGP2D02_Range(); 
  Serial.print(range);
  Serial.println("");
  delay(25);
}
 
int getGP2D02_Range()
{
  int val = 0;
 
  digitalWrite(IR_VIN, LOW);
  delay(70);
 
  for(int i = 7; i >= 0; i--)
  {
    digitalWrite(IR_VIN, HIGH);
    delayMicroseconds(100);  
    digitalWrite(IR_VIN, LOW);
    delayMicroseconds(100);
 
    val |= (digitalRead(IR_VOUT) << i);
    //Serial.print(digitalRead(IR_VOUT));
   }
 
   digitalWrite(IR_VIN, HIGH);
   delay(2);
   digitalWrite(IR_VIN, LOW);
  return val;
}
