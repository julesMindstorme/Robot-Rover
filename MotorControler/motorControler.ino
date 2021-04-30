#include <Servo.h>
#include <FastPID.h>

#define Kp 10// Proportional constant
#define Ki 1.25// Integral Constant
#define Kd 0.8// Derivative Constant
#define Hz 60
#define MaxOutput 5000 //can go  up to 20k

#define M1_ENCODER_A D1
#define M1_ENCODER_B D2
#define M1_PWM_Pin D3
#define M1_DIR_Pin D4
volatile long int M1_encoder_count = 0; // stores the current encoder count
long int M1_motor_pwm_value = 0; // after PID computation data is stored in this variable.
FastPID M1_pid_controller(Kp, Ki, Kd, Hz, 10, true);
int M1_pid_SetPoint;
Servo M1_PWM;


#define M2_ENCODER_A D5
#define M2_ENCODER_B D6
#define M2_PWM_Pin D7
#define M2_DIR_Pin D8
volatile long int M2_encoder_count = 0; // stores the current encoder count
long int M2_motor_pwm_value = 0; // after PID computation data is stored in this variable.
FastPID M2_pid_controller(Kp, Ki, Kd, Hz, 16, true);
int M2_pid_SetPoint;
Servo M2_PWM;


void ICACHE_RAM_ATTR M1_encoder();
void ICACHE_RAM_ATTR M2_encoder();


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  // for the M1 motor
  pinMode(M1_ENCODER_A, INPUT);
  pinMode(M1_ENCODER_B, INPUT);
  pinMode(M1_DIR_Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), M1_encoder, RISING);
  M1_pid_SetPoint = 0;
  M1_PWM.attach(M1_PWM_Pin);

  // for the M2 motor
  pinMode(M2_ENCODER_A, INPUT);
  pinMode(M2_ENCODER_B, INPUT);
  pinMode(M2_DIR_Pin, OUTPUT);
  pinMode(M2_PWM_Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), M2_encoder, RISING);
  M2_pid_SetPoint = 0;
  M2_PWM.attach(M2_PWM_Pin);
}


void loop() {

  M1_motor_pwm_value = map(M1_pid_controller.step(M1_pid_SetPoint, M1_encoder_count), -500, 500, -MaxOutput, MaxOutput);//can go up to 20k
  if (M1_motor_pwm_value > 0 ) {
    M1_PWM.writeMicroseconds(abs(M1_motor_pwm_value));
    digitalWrite(M1_DIR_Pin, LOW);

  }
  else {
    M1_PWM.writeMicroseconds(abs(M1_motor_pwm_value));
    digitalWrite(M1_DIR_Pin, HIGH);
  }



  M2_motor_pwm_value = map(M2_pid_controller.step(M2_pid_SetPoint, M2_encoder_count), -500, 500, -MaxOutput, MaxOutput);
  if (M2_motor_pwm_value > 0 ) {
    M2_PWM.writeMicroseconds(abs(M2_motor_pwm_value));
    digitalWrite(M2_DIR_Pin, LOW);

  }
  else {
    analogWrite(M2_PWM_Pin, abs(M2_motor_pwm_value));
     M2_PWM.writeMicroseconds(abs(M2_motor_pwm_value));
    digitalWrite(M2_DIR_Pin, HIGH);
  }


  Serial.print(M1_encoder_count);
  Serial.print("\t");
  Serial.println(M1_motor_pwm_value);
  /*
    Serial.print("\t");
    Serial.print(M2_encoder_count);
    Serial.print("\t");
    Serial.println(M2_motor_pwm_value);
  */


  if (Serial.available() > 0) { // this is supposed to be its own interupt routine
    String MotorName = Serial.readStringUntil(',');
    String FunctionName = Serial.readStringUntil(',');
    //Serial.println(MotorName);

    if (MotorName == "M1") {
      if (FunctionName == "ENCODER_REQUEST") {
        Serial.println(M1_encoder_count);
      }
      else if (FunctionName == "RESET_ENCODER") {
        //Serial.println("OK");
        M1_encoder_count = 0;
      }
      else if (FunctionName == "SET_POS") {
        //Serial.println("OK");
        M1_pid_SetPoint =  Serial.readStringUntil(';').toInt();
        //Serial.print(M1_pid_SetPoint);
        //Serial.println(";");
      }
    }

    else if (MotorName == "M2") {
      if (FunctionName == "ENCODER_REQUEST") {
        Serial.println(M2_encoder_count);
      }
      else if (FunctionName == "RESET_ENCODER") {
        //Serial.println("OK");
        M2_encoder_count = 0;
      }
      else if (FunctionName == "SET_POS") {
        //Serial.println("OK");
        M2_pid_SetPoint =  Serial.readStringUntil(';').toInt();
        //Serial.print(M2_pid_SetPoint);
        //Serial.println(";");
      }
    }
  }


}


/*
  void serialEvent() {// this is the serial interupt, it will handle the Serial Data

  String MotorName = Serial.readStringUntil(',');
  String FunctionName = Serial.readStringUntil(',');
  Serial.println(MotorName);

  if (MotorName == "M1") {
    if (FunctionName == "ENCODER_REQUEST") {
      Serial.println(M1_encoder_count);
    }
    else if (FunctionName == "RESET_ENCODER") {
      //Serial.println("OK");
      M1_encoder_count = 0;
    }
    else if (FunctionName == "SET_POS") {
      //Serial.println("OK");
      M1_pid_SetPoint =  Serial.readStringUntil(';').toInt();
      //Serial.print(M1_pid_SetPoint);
      //Serial.println(";");
    }
  }

  else if (MotorName == "M2") {
    if (FunctionName == "ENCODER_REQUEST") {
      Serial.println(M2_encoder_count);
    }
    else if (FunctionName == "RESET_ENCODER") {
      //Serial.println("OK");
      M2_encoder_count = 0;
    }
    else if (FunctionName == "SET_POS") {
      //Serial.println("OK");
      M2_pid_SetPoint =  Serial.readStringUntil(';').toInt();
      //Serial.print(M2_pid_SetPoint);
      //Serial.println(";");
    }
  }


  }

*/
//{Name},ENCODER_REQUEST,
//{Name},RESET_ENCODER,
//{Name},SET_POS,{pos};





void M1_encoder() {
  if (digitalRead(M1_ENCODER_B) == HIGH)
    M1_encoder_count--; // increment the count
  else // else decrease the count
    M1_encoder_count++; // decrement the count
}


void M2_encoder() {
  if (digitalRead(M2_ENCODER_B) == HIGH)
    M2_encoder_count--; // increment the count
  else // else decrease the count
    M2_encoder_count++; // decrement the count
}
