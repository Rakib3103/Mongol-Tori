// ARM
const int ACT_1_up = 17;
const int ACT_1_down = 18;

const int ACT_2_up = 19;
const int ACT_2_down = 20;

const int ACT_3_up = 10;
const int ACT_3_down = 11;

const int BASE_left = 12;
const int BASE_right = 13;

const int CLAW_open = 4;
const int CLAW_close = 5;

const int WRIST_left = 6;
const int WRIST_right = 7;

// WHEEL
const int WH_forward = 13;
const int WH_backward = 14;

const int WH_left = 15;
const int WH_right = 16;


void setup() {
  Serial.begin(9600);
  // Remove Keyboard.begin();
  pinMode(ACT_1_up, OUTPUT);
  pinMode(ACT_1_down, OUTPUT);
  pinMode(ACT_2_up, OUTPUT);
  pinMode(ACT_2_down, OUTPUT);
  pinMode(ACT_3_up, OUTPUT);
  pinMode(ACT_3_down, OUTPUT);
  pinMode(BASE_left, OUTPUT);
  pinMode(BASE_right, OUTPUT);
  pinMode(CLAW_open, OUTPUT);
  pinMode(CLAW_close, OUTPUT);
  pinMode(WRIST_left, OUTPUT);
  pinMode(WRIST_right, OUTPUT);

  pinMode(WH_forward, OUTPUT);
  pinMode(WH_backward, OUTPUT);
  pinMode(WH_left, OUTPUT);
  pinMode(WH_right, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    char val = Serial.read();

    if (val == 'r') {
      digitalWrite(ACT_1_up, LOW);
      digitalWrite(ACT_1_down, HIGH);
    }

    if (val == 'f') {
      digitalWrite(ACT_1_up, HIGH);
      digitalWrite(ACT_1_down, LOW);
    }

    if (val == 't') {
      digitalWrite(ACT_2_up, LOW);
      digitalWrite(ACT_2_down, HIGH);
    }

    if (val == 'g') {	
      digitalWrite(ACT_2_up, HIGH);
      digitalWrite(ACT_2_down, LOW);
    }

    if (val == 'v') {
      digitalWrite(BASE_left, LOW);
      digitalWrite(BASE_right, HIGH);
    }

    if (val == 'b') {
      digitalWrite(BASE_left, HIGH);
      digitalWrite(BASE_right, LOW);
    }
    
    if (val == 'y') {
      digitalWrite(CLAW_open, LOW);
      digitalWrite(CLAW_close, HIGH);
    }

    if (val == 'h') {	
      digitalWrite(CLAW_open, HIGH);
      digitalWrite(CLAW_close, LOW);
    }

    if (val == 'm') {
      digitalWrite(WRIST_left, LOW);
      digitalWrite(WRIST_right, HIGH);
    }

    if (val == 'n') {	
      digitalWrite(WRIST_left, HIGH);
      digitalWrite(WRIST_right, LOW);
    }
    
    // ------------------------------------------------------------------------------- WHEEL ---------------------------------------------------------------------------------
    if (val == 'w') { //Forward
      digitalWrite(WH_forward, LOW);
      digitalWrite(WH_backward, HIGH);
    }

    if (val == 'a') { //LEFT
      digitalWrite(WH_left, HIGH);
      digitalWrite(WH_right, LOW);
    }

    if (val == 's') { //Backward
      digitalWrite(WH_forward, HIGH);
      digitalWrite(WH_backward, LOW);
    }

    if (val == 'd') { // Right
      digitalWrite(WH_left, LOW);
      digitalWrite(WH_right, HIGH);
    }
    if (val == '-') { // Right
      digitalWrite(WH_left, HIGH);
      digitalWrite(WH_right, HIGH);
      digitalWrite(WH_forward, HIGH);
      digitalWrite(WH_backward, HIGH);



      digitalWrite(ACT_1_up, HIGH);
      digitalWrite(ACT_1_down, HIGH);
      digitalWrite(ACT_2_up, HIGH);
      digitalWrite(ACT_2_down, HIGH);
      digitalWrite(ACT_3_up, HIGH);
      digitalWrite(ACT_3_down, HIGH);
      digitalWrite(BASE_left, HIGH);
      digitalWrite(BASE_right, HIGH);
      digitalWrite(CLAW_open, HIGH);
      digitalWrite(CLAW_close, HIGH);
      digitalWrite(WRIST_left, HIGH);
      digitalWrite(WRIST_right, HIGH);
    }

    // Send the received key over Serial
    Serial.write(val);
  }
}
