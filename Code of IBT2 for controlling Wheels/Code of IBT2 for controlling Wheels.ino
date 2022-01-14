//For act1
int RPWM_act1 = 1;
int LPWM_act1 = 2;
//For act2
int RPWM_act2 = 3;
int LPWM_act2 = 4;
//For act3
int RPWM_act3 = 5;
int LPWM_act3 = 6;
//For act4
int RPWM_act4 = 7;
int LPWM_act4 = 8;
//For claw
int RPWM_claw = 9;
int LPWM_claw = 10;
//For Base
int RPWM_base = 11;
int LPWM_base = 12;
//For Wrist
int RPWM_wrist = 13;
int LPWM_wrist = 14;

void setup() {
  pinMode(RPWM_act1, OUTPUT);
  pinMode(LPWM_act1, OUTPUT);
  pinMode(RPWM_act2, OUTPUT);
  pinMode(LPWM_act2, OUTPUT);
  pinMode(RPWM_act3, OUTPUT);
  pinMode(LPWM_act3, OUTPUT);
  pinMode(RPWM_act4, OUTPUT);
  pinMode(LPWM_act4, OUTPUT);
  pinMode(RPWM_claw, OUTPUT);
  pinMode(LPWM_claw, OUTPUT);
  pinMode(RPWM_base, OUTPUT);
  pinMode(LPWM_base, OUTPUT);
  pinMode(RPWM_wrist, OUTPUT);
  pinMode(LPWM_wrist, OUTPUT);
}
void stop_all(){
  analogWrite(RPWM_act1, 0);
  analogWrite(LPWM_act1, 0);
  analogWrite(RPWM_act2, 0);
  analogWrite(LPWM_act2, 0);
  analogWrite(RPWM_act3, 0);
  analogWrite(LPWM_act3, 0);
  analogWrite(RPWM_act4, 0);
  analogWrite(LPWM_act4, 0);
  analogWrite(RPWM_claw, 0);
  analogWrite(LPWM_claw, 0);
  analogWrite(RPWM_base, 0);
  analogWrite(LPWM_base, 0);
  analogWrite(RPWM_wrist, 0);
  analogWrite(LPWM_wrist, 0);
}
void act1_up(){
   analogWrite(RPWM_act1,255);
   analogWrite(LPWM_act1,0);
}
void act1_down(){
  analogWrite(RPWM_act1,0);
  analogWrite(LPWM_act1,255);
}
void act2_up(){
   analogWrite(RPWM_act2,255);
   analogWrite(LPWM_act2,0);
}
void act2_down(){
  analogWrite(RPWM_act2,0);
  analogWrite(LPWM_act2,255);
}
void act3_up(){
  analogWrite(RPWM_act3,255);
  analogWrite(LPWM_act3,0);
}
void act3_down(){
  analogWrite(RPWM_act3,0);
  analogWrite(LPWM_act3,255);
}
void act4_up(){
   analogWrite(RPWM_act4,255);
   analogWrite(LPWM_act4,0);
}
void act4_down(){
  analogWrite(RPWM_act4,0);
  analogWrite(LPWM_act4,255);
}
void claw_open(){
  analogWrite(RPWM_claw,255);
  analogWrite(LPWM_claw,0);
}
void claw_close(){
  analogWrite(RPWM_claw,0);
  analogWrite(LPWM_claw,255);
}
void wrist_clockwise(){
  analogWrite(RPWM1,255);
  analogWrite(LPWM1,0);
}
void wrist_anticlockwise(){
  analogWrite(RPWM1,0);
  analogWrite(LPWM1,255);
}
void loop(){
  if (Serial.available() > 0){
    inByte = Serial.read();

    if(inByte == 'r'){
      act1_up();
    }
    else if (inByte == 'f'){
      act1_down();
    }
    if(inByte == 't'){
      act2_up();
    }
    else if (inByte == 'g'){
      act2_down();
    }
    if(inByte == 'y'){
      act3_up();
    }
    else if (inByte == 'h'){
      act3_down();
    }
}
