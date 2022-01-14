// Motor 1
int RPWM1=5;
int LPWM1=6;
int L_EN1=7; 
int R_EN1=8; 

void setup()
{
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(R_EN1, OUTPUT);
  pinMode(L_EN1, OUTPUT);
}
void loop() {
   if (Serial.available() > 0){
      inByte = Serial.read();
      if(inByte == 'w'){
         forwardmotorsCall()
  } 
      elif(inByte == 's'){
         backwardmotorsCall()
  } 
      elif(inByte == 'd'){
         frightdirCall()
  } 

      elif(inByte == 'a'){
         leftdirCall()
  } 
 }
}

void forwardmotorsCall()
{
  digitalWrite(R_EN1,HIGH);
  digitalWrite(L_EN1,LOW);
  analogWrite(RPWM1,255);
  analogWrite(LPWM1,0);
}
void backwardmotorsCall()
{
  digitalWrite(R_EN1,LOW);
  digitalWrite(L_EN1,HIGH);
  analogWrite(RPWM1,0);
  analogWrite(LPWM1,255);
}
void rightdirCall()
{
  digitalWrite(R_EN1,HIGH);
  digitalWrite(L_EN1,LOW);
  analogWrite(RPWM1,255);
  analogWrite(LPWM1,0);
}
void leftdirCall()
{
  digitalWrite(R_EN1,LOW);
  digitalWrite(L_EN1,HIGH);
  analogWrite(RPWM1,0);
  analogWrite(LPWM1,255);
}
