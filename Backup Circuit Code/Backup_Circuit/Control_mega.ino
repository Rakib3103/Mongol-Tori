#include <ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <rover_control/ControlStatus.h>

#include "CytronMotorDriver.h"
#include "Cytron.h"
#include "CytronArm.h"
#include "CmdReference.h"
#include "3dof_ik.h"

#define ARM_ACT_1_PWM 17
#define ARM_ACT_2_PWM 19
#define ARM_ACT_3_PWM 40
#define CLAW_SPIN_PWM 11
#define ARM_BASE_PWM 36
#define CLAW_GRIP_PWM 8

#define ARM_ACT_1_DIR 18
#define ARM_ACT_2_DIR 20
#define ARM_ACT_3_DIR 38
#define CLAW_SPIN_DIR 10
#define ARM_BASE_DIR 34
#define CLAW_GRIP_DIR 9

#define ACT1_SEN A6
#define ACT2_SEN A7
#define BASE_SEN A5
// #define BASE_LIM_MIN 34
// #define BASE_LIM_MAX 36

#define WHEEL_LEFT_PWM 44
#define WHEEL_RIGHT_PWM 45

#define WHEEL_LEFT_DIR 46
#define WHEEL_RIGHT_DIR 47

#define FPV_TOGGLE_1 51
#define FPV_TOGGLE_2 53

float SKID_MULTIPLIER = 0.35;

int RATE = 150;

int SPEED_INC_MULTIPLIER = 6;

#define DEBUG

void pinModeSetup();
void updateWheel();
void stop();

void postStatus();
void postDebug(const char *);

void keycmdCb(const std_msgs::String &);
void cmdvelCb(const geometry_msgs::Twist &);
void armIkCb(const geometry_msgs::Vector3 &);

ros::NodeHandle nh;

rover_control::ControlStatus controlStatusMsg;
std_msgs::String debugMsg;

ros::Subscriber<std_msgs::String> keycmdTopic("rover_control", &keycmdCb);
ros::Subscriber<geometry_msgs::Twist> cmdvelTopic("cmd_vel", &cmdvelCb);
ros::Subscriber<geometry_msgs::Vector3> armIkTopic("arm_target", &armIkCb);

ros::Publisher controlStatusTopic("control_status", &controlStatusMsg);
ros::Publisher debugTopic("debug", &debugMsg);

int maxSpeedArm = 255;
int maxSpeedWheel = 100;

// Record Velocities for synchronous movement
float act1_vel = 0.0;
float act2_vel = 0.0;
float base_vel = 0.0;
float max_vel = 0.0;

int leftWheelTarget = 0;
int rightWheelTarget = 0;

float act1Target = 0;
float act2Target = 0;
float baseTarget = 0;

char currentWheelCmd = '-';
char currentArmCmd = '-';

String allowedWheelCmds = "wsadqezc";

// MODE, PWM Pin, DIR Pin, Sensor Pin, Min Angle, Max Angle
CytronArm ACT_1(PWM_PWM, ARM_ACT_1_PWM, ARM_ACT_1_DIR, ACT1_SEN, 11, 95); // Read: 163, 491
CytronArm ACT_2(PWM_PWM, ARM_ACT_2_PWM, ARM_ACT_2_DIR, ACT2_SEN, 30, 132); // Read: 169, 573

CytronArm BASE(PWM_PWM, ARM_BASE_PWM, ARM_BASE_DIR); //, BASE_SEN, BASE_LIM_MIN, BASE_LIM_MAX, -90, 90); // Read: 169, 858

CytronArm ACT_3(PWM_PWM, ARM_ACT_3_PWM, ARM_ACT_3_DIR);
CytronArm CLAW_SPIN(PWM_PWM, CLAW_SPIN_PWM, CLAW_SPIN_DIR);
CytronArm CLAW_GRIP(PWM_PWM, CLAW_GRIP_PWM, CLAW_GRIP_DIR);

Cytron WHEEL_LEFT(PWM_DIR, WHEEL_LEFT_PWM, WHEEL_LEFT_DIR);
Cytron WHEEL_RIGHT(PWM_DIR, WHEEL_RIGHT_PWM, WHEEL_RIGHT_DIR);

CartesianArm cartArm(BASE, ACT_1, ACT_2);

const CmdArmRef arm_cmds[6] = {
    {ACT_1,
     {'r', 'f'},
     {1, -1}},
    {ACT_2,
     {'t', 'g'},
     {1, -1}},
    {ACT_3,
     {'y', 'h'},
     {1, -1}},
    {CLAW_SPIN,
     {'v', 'b'},
     {1, -1}},
    {BASE,
     {'n', 'm'},
     {1, -1}},
    {CLAW_GRIP,
     {'o', 'p'},
     {1, -1}}};

const String cart_cmds = "ikjl]'";

const String fpv_cmds = ",.";

CmdWheelRef wheel_cmds[8] = {
    {'w', {1, 1}},
    {'s', {-1, -1}},
    {'a', {-1, 1}},
    {'d', {1, -1}}};

/* ========================================================================== */
/*                          ROS CALLBACKS & FUNCTIONS                         */
/* ========================================================================== */

void keycmdCb(const std_msgs::String &msg)
{
  if (msg.data[0] == '-') // Stop Command
  {
    stop();
    return;
  }

  if (msg.data[0] == currentArmCmd || msg.data[0] == currentWheelCmd) // if the same command is sent, ignore it
  {
    return;
  }

  int fpv_cmd_idx = fpv_cmds.indexOf(msg.data[0]);

  if (fpv_cmd_idx != -1) {
    toggleFPV(fpv_cmd_idx);
  }

  for (int i = 0; i < 6; i++) // iterate through arm commands for a match
  {
    for (int j = 0; j < 2; j++)
    {
      if (msg.data[0] == arm_cmds[i].cmds[j])
      {
        currentArmCmd = arm_cmds[i].cmds[j];
        arm_cmds[i].motor.setTarget(arm_cmds[i].motor.getCurrentAngle() + arm_cmds[i].multiplier[j] * 10);

        postStatus();
        break;
      }
    }
  }

  if (cart_cmds.indexOf(msg.data[0]) != -1) {
    // iterate through cartesian commands for a match
    int* targets;
    switch(msg.data[0]) {
      case 'l':
        targets = cartArm.goCartesian('x', 1); // x+
        break;
      case 'j':
        targets = cartArm.goCartesian('x', -1); // x-
        break;
      case 'i':
        targets = cartArm.goCartesian('y', 1); // y+
        break;
      case 'k':
        targets = cartArm.goCartesian('y', -1); // y-
        break;
      case ']':
        targets = cartArm.goCartesian('z', 1); // z+
        break;
      case '\'':
        targets = cartArm.goCartesian('z', -1); // z-
        break;
    }

    char buff[50];
    sprintf(buff, "Cartesian Arm: %d, %d, %d", targets[0], targets[1], targets[2]);    
    nh.loginfo(buff);
  }


  for (int i = 0; i < 8; i++) // iterate through wheel commands for a match
  {
    if (msg.data[0] == wheel_cmds[i].cmd)
    {
      currentWheelCmd = wheel_cmds[i].cmd;

      leftWheelTarget = maxSpeedWheel * wheel_cmds[i].multiplier[0];
      rightWheelTarget = maxSpeedWheel * wheel_cmds[i].multiplier[1];

      leftWheelTarget = constrain(leftWheelTarget, -255, 255);
      rightWheelTarget = constrain(rightWheelTarget, -255, 255);

      break;
    }
  }

  switch (msg.data[0])
  {
  case '1' + 0:
    maxSpeedWheel = 50; // Base speed
    break;
  case '1' + 1:
    maxSpeedWheel = 100; // Inc by 65
    break;
  case '1' + 2:
    maxSpeedWheel = 150; // Inc by 50
    break;
  case '1' + 3:
    maxSpeedWheel = 255; // Inc by 40
    break;
  }
}

void cmdvelCb(const geometry_msgs::Twist &msg)
{
  float linear = constrain(msg.linear.x, -10.0, 10.0);
  float angular = constrain(msg.angular.z, -PI, PI);

  int straight = linear / 10.0 * 255; // 10.0 is the max linear velocity
  int turn = angular / PI * 255;      // PI is the max angular velocity

  leftWheelTarget = straight - turn;
  rightWheelTarget = straight + turn;

  leftWheelTarget = constrain(leftWheelTarget, -255, 255);
  rightWheelTarget = constrain(rightWheelTarget, -255, 255);
}

void armIkCb(const geometry_msgs::Vector3 &msg) // Handle XYZ Target message for arm
{
  // stop();
  
  // ArmAngles angles;
  // if (msg.x == 999 && msg.y == 999 && msg.z == 999)
  // {
  //   calibrateArms();
  //   return;
  // }
  // calculateArmAngles(msg.x, msg.y, msg.z, angles);
  // ACT_1.setTargetAngle(angles.act1);
  // ACT_2.setTargetAngle(angles.act2);
  // BASE.setTargetAngle(angles.base);

  // // Debug message stuff below this

  // char a1[14];
  // char a2[14];
  // char ba[14];

  // dtostrf(angles.act1, 7, 2, a1);
  // dtostrf(angles.act2, 7, 2, a2);
  // dtostrf(angles.base, 7, 2, ba);

  // char buff[20];
  // sprintf(buff, "ACT1: %s, ACT2: %s, BASE: %s", a1, a2, ba);
  // nh.loginfo(buff);
  // postDebug(buff);
}

void postStatus() // Publish wheel and arm pwm status to ROS
{
  controlStatusMsg.act1_speed = ACT_1.getSpeed();
  controlStatusMsg.act2_speed = ACT_2.getSpeed();
  controlStatusMsg.act3_speed = ACT_3.getSpeed();
  controlStatusMsg.claw_spin_speed = CLAW_SPIN.getSpeed();
  controlStatusMsg.base_speed = BASE.getSpeed();
  controlStatusMsg.claw_grip_speed = CLAW_GRIP.getSpeed();

  controlStatusMsg.left_speed = WHEEL_LEFT.getSpeed();
  controlStatusMsg.right_speed = WHEEL_RIGHT.getSpeed();

  // postDebug("status");

  controlStatusTopic.publish(&controlStatusMsg);
}

void postDebug(const char *msg) // Publish debug message to ROS
{
#ifdef DEBUG
  debugMsg.data = msg;
  debugTopic.publish(&debugMsg);
#endif
}

/* ========================================================================== */
/*                          MAIN SKETCH SETUP & LOOP                          */
/* ========================================================================== */

void setup()
{
  pinModeSetup();

  nh.getHardware()->setBaud(9600);

  nh.initNode();

  nh.subscribe(keycmdTopic);
  nh.subscribe(cmdvelTopic);
  nh.subscribe(armIkTopic);
  nh.advertise(controlStatusTopic);
  nh.advertise(debugTopic);

  while (!nh.connected())
  {
    nh.spinOnce();
    delay(100);
  }

  // act1_vel = ACT_1.getVelocity();
  // nh.spinOnce();
  // act2_vel = ACT_2.getVelocity();
  // nh.spinOnce();
  // base_vel = BASE.getVelocity();
  // nh.spinOnce();

  // max_vel = max(max(act1_vel, act2_vel), base_vel);

  // cartArm.init();

  // postDebug(String(act1_vel).c_str());
  // postDebug(String(act2_vel).c_str());
  // postDebug(String(base_vel).c_str());

  if (!nh.getParam("~skid_multiplier", &SKID_MULTIPLIER, 1, 300))
  {
    SKID_MULTIPLIER = 0.35;
  }
  if (!nh.getParam("~increment_speed", &SPEED_INC_MULTIPLIER, 1, 300))
  {
    SPEED_INC_MULTIPLIER = 2;
  }
  if (!nh.getParam("~rate", &RATE, 1, 300))
  {
    RATE = 175;
  }
  if (!nh.getParam("~arm_max", &maxSpeedArm, 1, 300))
  {
    maxSpeedArm = 255;
  }

  // ACT_1.setAnalogRange(163, 491);
  // ACT_2.setAnalogRange(169, 573);
  // BASE.setAnalogRange(169, 858);

  wheel_cmds[4] = {'q', {SKID_MULTIPLIER, 1}};
  wheel_cmds[5] = {'e', {1, SKID_MULTIPLIER}};
  wheel_cmds[6] = {'z', {-SKID_MULTIPLIER, -1}};
  wheel_cmds[7] = {'c', {-1, -SKID_MULTIPLIER}};

  postDebug(String(SKID_MULTIPLIER).c_str());
  postDebug(String(SPEED_INC_MULTIPLIER).c_str());
  postDebug(String(RATE).c_str());
}

void loop()
{
  unsigned long start = millis();

  nh.spinOnce();
  // if (spinRet == ros::REQU)

  if (!nh.connected() && (currentArmCmd != '-' || currentWheelCmd != '-'))
  { // Stop rover if connection lost with serial_node
    stop();
  }

  // updateArms();
  updateWheel();

  char buff[50];
  sprintf(buff, "A1: %d, A2: %d, BA: %d, T1: %d, T2: %d, TB: %d", ACT_1.read(), ACT_2.read(), BASE.read(), (int)ACT_1.getTargetAngle(), (int)ACT_2.getTargetAngle(), (int)BASE.getTargetAngle());
  // postDebug(buff);

  short passed = millis() - start;
  delay(max(1000 / RATE - passed, 0)); // Only delay if tasks finished before cycle time ran out
}

/* ========================================================================== */
/*                              CONTROL FUNCTIONS                             */
/* ========================================================================== */

void pinModeSetup()
{
  pinMode(ARM_ACT_1_PWM, OUTPUT);
  pinMode(ARM_ACT_1_DIR, OUTPUT);
  pinMode(ARM_ACT_2_PWM, OUTPUT);
  pinMode(ARM_ACT_2_DIR, OUTPUT);
  pinMode(ARM_ACT_3_PWM, OUTPUT);
  pinMode(ARM_ACT_3_DIR, OUTPUT);
  pinMode(CLAW_SPIN_PWM, OUTPUT);
  pinMode(CLAW_SPIN_PWM, OUTPUT);
  pinMode(ARM_BASE_PWM, OUTPUT);
  pinMode(ARM_BASE_PWM, OUTPUT);
  pinMode(CLAW_GRIP_PWM, OUTPUT);
  pinMode(CLAW_GRIP_DIR, OUTPUT);
  pinMode(WHEEL_LEFT_PWM, OUTPUT);
  pinMode(WHEEL_LEFT_DIR, OUTPUT);
  pinMode(WHEEL_RIGHT_PWM, OUTPUT);
  pinMode(WHEEL_RIGHT_DIR, OUTPUT);

  pinMode(ACT1_SEN, INPUT);
  pinMode(ACT2_SEN, INPUT);
  pinMode(BASE_SEN, INPUT);

  pinMode(BASE_LIM_MIN, INPUT);
  pinMode(BASE_LIM_MIN, INPUT);

  pinMode(FPV_TOGGLE_1, OUTPUT);
  pinMode(FPV_TOGGLE_2, OUTPUT);
}

void toggleFPV(uint8_t i) {
  switch(i) {
    case 0:
      digitalWrite(FPV_TOGGLE_1, HIGH - digitalRead(FPV_TOGGLE_1));
      break;
    case 1:
      digitalWrite(FPV_TOGGLE_2, HIGH - digitalRead(FPV_TOGGLE_2));
      break;
  }
}

void updateWheel() // Non-blocking way to gradually update speed
{
  const int lspeed = WHEEL_LEFT.getSpeed();
  const int rspeed = WHEEL_RIGHT.getSpeed();

  if (lspeed == leftWheelTarget && rspeed == rightWheelTarget)
  {
    return;
  }

  byte lsign = leftWheelTarget - lspeed < 0 ? 1 : 0;
  byte rsign = rightWheelTarget - rspeed < 0 ? 1 : 0;

  // Find ratio of speed delta and set increment values to reach both target at same time
  float ldelta = abs(leftWheelTarget - lspeed);
  float rdelta = abs(rightWheelTarget - rspeed);
  float ratio = ldelta / rdelta;
  float linc = 1;
  float rinc = 1;

  if (ratio > 1)
  {
    rinc = 1 / ratio;
  }
  else
  {
    linc = ratio;
  }

  // Multiplier to control acceleration

  linc *= SPEED_INC_MULTIPLIER;
  rinc *= SPEED_INC_MULTIPLIER;

  // Ensure that increment is at least 1

  linc = max(linc, 1);
  rinc = max(rinc, 1);

  // Increment in the direction of the target speed
  // By taking min of increment and delta, we can ensure that the speed will not go over target

  if (lspeed != leftWheelTarget)
  {
    WHEEL_LEFT.setSpeed(lspeed + min(linc, ldelta) * (lsign ? -1 : 1));
  }

  if (rspeed != rightWheelTarget)
  {
    WHEEL_RIGHT.setSpeed(rspeed + min(rinc, rdelta) * (rsign ? -1 : 1));
  }

  postStatus();
}

void updateArms()
{
  ACT_1.update(max_vel);
  ACT_2.update(max_vel);
  BASE.update(max_vel);
}

void calibrateArms()
{
  BASE.calibrateROF();
  ACT_2.calibrateROF();
  ACT_1.calibrateROF();
}

void stop()
{
  currentArmCmd = '-';
  currentWheelCmd = '-';

  leftWheelTarget = 0;
  rightWheelTarget = 0;

  ACT_1.setSpeed(0);
  ACT_2.setSpeed(0);
  ACT_3.setSpeed(0);
  CLAW_SPIN.setSpeed(0);
  BASE.setSpeed(0);
  CLAW_GRIP.setSpeed(0);

  postStatus();
}
