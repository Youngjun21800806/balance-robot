#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "esp_task_wdt.h"
//-------------------------------------------------------
int16_t mpu[7];
//float offset[7];
int N = 500; //testing number
int i, j; //for loop integer
//-------------------------------------------------------
float accAng[3], gyroAng[3], mdata[7];
float xhm0, xhm1, xh0, xh1, Q, R, K0, K1, Pm00, Pm01, Pm10, Pm11, P00, P01, P10, P11;
float dTime, cTime, pTime, dt;
int turn;
//-------------------------------------------------------
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_BASE_FREQ 500//2ms
#define freq_time 1000//1ms->10ms
//------------------------------------------------------------Timer
float stopflag;
volatile int interruptCounter;
int totalinterruptCounter;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
float REFERENCE_ERR=0;
double PD_pwm=0;
int speedcc =0;
int offset[7]={0,0,0,0,0,0,0};
//--------------------pulse calculation-----------------
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int pulseright,pulseleft;
int pwm1 = 0;
int pwm2 = 0;
float Turn;
const int PWM1 = 2;//L
const int PWM2 = 15;//R
const int IN1M = 13;//R
const int IN2M = 12;//R
const int IN3M = 4;//L
const int IN4M = 5;//L
const int PL = 34;//R
const int PR = 35;//L
volatile long count_right = 0;//Used to calculate the pulse value calculated by the Hall encoder (the volatile long type is to ensure the value is valid)
volatile long count_left = 0;

String buf;
const char* ssid = "Hwang";
const char* password = "50875087";
const char* server ="192.168.124.100";
const char* clientID = "capstone";
const char* topic_action = "robot/action";
const char* topic_data = "robot/data";
const char* topic_info = "robot/info";



void rxMsg(char* topic, byte* payload, unsigned int payloadLength);
TaskHandle_t Task1,Task2;
WiFiClient wifiClient;
PubSubClient client (wifiClient);
//-------------------------------------------------------
float Kp =30.0, Ki=13.0, Kd=0.6;
float kp_speed = 3.7, ki_speed = 0.17, kd_speed=1.8;
float kp_turn = 0.0, ki_turn = 0.0, kd_turn = 0.0;
//-------------------------------------------------------
float err,prev_err,derv,integ=0,integp,dervp,errp,pitch_dot;
//-------------------PI speed control-------------
float speeds_filterold=0;
float positions=0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;
//------------------PI direction control--------
int turnmax,turnmin,turnout; 
float Turn_pwm = 0;
int zz=0;
int turncc=0;
//-------------------------------python-----------------------------------------
float reward=0;
int sendflag=0;
int done=0;
float kpinc=2;
int timesteps;

int task2counter=0;
float xh0_avg;
float xh0_total=0;


void IRAM_ATTR Cal_Left() {
    count_left++;
}
void IRAM_ATTR Cal_Right() {
    count_right++;
}

void turnspin(){ //we just use accZ to compansate the error, not for turing itself
  Turn_pwm = mdata[6]*kd_turn;
}

void countpluse()
{
  lz = count_left;     //Assign the value counted by the code wheel to lz
  rz = count_right;

  count_left = 0;     //Clear the code counter count
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((pwm1 < 0) && (pwm2 < 0)) //backwards, negative number
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 > 0)) // front, positive number
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((pwm1 < 0) && (pwm2 > 0)) //turn left, R -> +, L -> -
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 < 0)) //turn right, R -> -, L -> +
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  //enter interrupts per 5ms; pulse number superposes
  pulseright += rpluse;
  pulseleft += lpluse;
}

void mpu6050_init(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
}

void getData() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.requestFrom(0x68, 14, true); // request a total of 14 registers
    mpu[0] = Wire.read()<<8|Wire.read();
    mpu[1] = Wire.read()<<8|Wire.read();
    mpu[2] = Wire.read()<<8|Wire.read();
    mpu[3] = Wire.read()<<8|Wire.read();
    mpu[4] = Wire.read()<<8|Wire.read();
    mpu[5] = Wire.read()<<8|Wire.read();
    mpu[6] = Wire.read()<<8|Wire.read();
}
  
void pwmOut(float stopflag) {
  if(stopflag==1) {
    pwm1=0;
    pwm2=0;
  }else{
  	pwm1 = PD_pwm - PI_pwm + Turn_pwm+turn;
  	pwm2 = PD_pwm - PI_pwm - Turn_pwm-turn;
  }
  
  if (pwm1 >= 0) {
    pwm1 += 0;
    if (pwm1 > 255)
      pwm1 = 255;
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    ledcWrite(LEDC_CHANNEL_0, pwm1);
  } 
  else{
    pwm1 -= 0;
    if (pwm1 < -255)
      pwm1 =-255;
    digitalWrite(IN1M, LOW);
    digitalWrite(IN2M, HIGH);
    ledcWrite(LEDC_CHANNEL_0, -pwm1);
  }

  if (pwm2 >= 0) {
    pwm2 += 0;
    if (pwm2 > 255)
      pwm2 = 255;
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
    ledcWrite(LEDC_CHANNEL_1, pwm2);
  }
  else {
    pwm2 -= 0;
    if (pwm2 < -255)
      pwm2 = -255;
    digitalWrite(IN3M, HIGH);
    digitalWrite(IN4M, LOW);
    ledcWrite(LEDC_CHANNEL_1, -pwm2);
  }
}


void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void speedpiout()
{
  float speeds = (pulseleft + pulseright) * 1.0;      //Vehicle speed  pulse value
  pulseright = pulseleft = 0;      //Clear
  speeds_filterold *= 0.7;         //first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550,3550);    //Anti-integral saturation
  PI_pwm = ki_speed * (0 - positions) + kp_speed * (0 - speeds_filter);      //speed loop control PI
}

float rewardmethod(){
  return 1/(pow(err,2)+pow(pitch_dot,2))+1/Kp;
}

void Angle(){
                cTime = millis();
                dt = (cTime - pTime)/1000.0;
                pTime = cTime;
                mdata[0] = ((float)mpu[0]*100) * 2.0 / ((float)100 *32768.0); // Ax
                mdata[1] = ((float)mpu[1]*100-offset[1]) * 2.0 / ((float)100 *32768.0); // Ay
                mdata[2] = ((float)mpu[2]*100) * 2.0 / ((float)100 *32768.0); // Az
                mdata[4] = ((float)mpu[4]*100-offset[4]) * 250.0 / ((float)100 * 32768.0); //Rx
                mdata[6] = ((float)mpu[6]*100) * 250.0 / ((float)100 * 32768.0); //Rx
                accAng[0] = atan2( mdata[1], sqrt(mdata[0] * mdata[0] + mdata[2] * mdata[2])) * 180 / PI; // roll
                gyroAng[0] += (mdata[4]) * dt; //roll angle  
                pitch_dot=mdata[4]-xh1;
                xhm0 = xh0 + dt * (pitch_dot);
                xhm1 = xh1;
                Pm00 = P00 - dt * (P01 + P10) + dt * dt * P11 + Q;//Q-angle
                Pm01 = P01 - dt * P11;
                Pm10 = P10 - dt * P11;
                Pm11 = P11 + Q;//q-gyro
                K0 = Pm00 / (Pm00 + R);//r-angle
                K1 = Pm10 / (Pm00 + R);
				
                xh0 = xhm0 + K0 * (accAng[0] - xhm0);
                xh1 = xhm1 + K1 * (accAng[0] - xhm0);
				
                P00 = Pm00 - K0 * Pm00;
                P01 = Pm01 - K0 * Pm01;
                P10 = Pm10 - K1 * Pm00;
                P11 = Pm11 - K1 * Pm01;
                /*
                Serial.print(xh0);
                Serial.printf(", ");
                Serial.print(accAng[0]);
                Serial.printf(", ");
                Serial.print(gyroAng[0]);
                Serial.printf("\n");*/
}

int counter=0;
void codeForTask1( void * parameter ) {
   for (;;) {
      client.loop();
      if(!client.loop()){
      mqttConnect();//연결 끊겨고  pwm out()넣음으로 유지
      }
        if(sendflag == 1){
            if(counter>=50){
              pub_info();
              counter=0;
              timesteps++;
            }
            else
            pub_data();
            sendflag=0;    
            counter++;
        }
        if(timesteps==1000){
          if(kpinc>=1) kpinc=kpinc/2;
          timesteps=0;
        }
  	  attachInterrupt(PL, Cal_Left, CHANGE);
  	  attachInterrupt(PR, Cal_Right, CHANGE);
}
}

void codeForTask2( void* parameter){
  for(;;){
    if(interruptCounter == 1){
                getData();
                sendflag=1;
                task2counter++;
			          if(task2counter==10){
                    Angle();
                    countpluse();
                    xh0_avg=xh0_total/10.0;
                    err=REFERENCE_ERR-xh0; 
                    integ+=err*0.01;
                    derv=(err-prev_err)/0.01;
                    prev_err = err;
                    PD_pwm = err*Kp+integ*Ki+derv*Kd;          
        				    speedcc++;
                    
                    if(speedcc >= 4){
                        speedpiout();
                        speedcc = 0;
                    }
                    turncc++;
                    if(turncc >= 2){
                      turnspin();
                      turncc = 0;
                    }
                    if(abs(err)>10) {
                      done=1;
                      stopflag=1;
                      integ=0;
                      positions=0;
                      speeds_filter=0;
                      speeds_filterold=0;
                    }
                    else {
                      done=0;
                      stopflag=0;
                    }
                  //--------------------------------------------------PWM_calc-------------------------------------------
                    pwmOut(stopflag);
                    task2counter=0;
                    xh0_total=0;
                } 
                portENTER_CRITICAL_ISR(&timerMux);
                interruptCounter=0;
                portEXIT_CRITICAL_ISR(&timerMux); 
        }
  }
}
   
void setup() {
  Serial.begin(115200);
  client.setServer(server, 1883);
  client.setCallback(rxMsg);
 

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, 8);
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, 8);
  ledcAttachPin(PWM1, LEDC_CHANNEL_0);
  ledcAttachPin(PWM2, LEDC_CHANNEL_1);
  pinMode(IN1M, OUTPUT);
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);
  pinMode(IN4M, OUTPUT);
  pinMode(PL,INPUT);
  pinMode(PR,INPUT);
  Wire.begin(21, 22);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);
  mpu6050_init();
  delay(100);

  for (int i = 0; i < 3; i++)
    gyroAng[i] = 0.0;
  //----------------------------------------------------------------------
  xh0 = 0; xh1 = 0;
  P00 = 1.0; P01 = 0; P10 = 0; P11 = 1.0;
  Q=0.0001; R = 0.05;//gyro covariance remeasure

  for (int i =0; i<100;i++){
    getData();
    delay(1);
    for(int j=0; j<7;j++){
      offset[j]+=mpu[j];
    }
  }
  
  wifiConnect();
  mqttConnect();
//-------------------------------------------------
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer,&onTimer,true);
  timerAlarmWrite(timer, freq_time, true);
  timerAlarmEnable(timer);

   xTaskCreatePinnedToCore(    codeForTask1,    "wifi",    5000,      NULL,    2,    &Task1,    0);
   xTaskCreatePinnedToCore(    codeForTask2,    "balance",    5000,    NULL,    2,    &Task2,    1);

  esp_task_wdt_init(30, false);
}

void loop() {
  
}


void rxMsg(char* rtopic, byte* payload, unsigned int payloadLength){
  Serial.print("Receive a message of a topic: ");
  Serial.println(rtopic);
  payload[payloadLength] = '\0';
  Serial.printf("Payload: %s\r\n", (int *)payload); 
  if(String(rtopic).equals(String(topic_action))){
     int cmd = atoi((char*)payload); 
    switch(cmd){
      case 0:
            if(Kp>=2)
            Kp-=0.5;
      break;
      case 1:
            Kp=Kp;
      break;
      case 2:
            if(Kp<=58)
            Kp+=0.5;
      break;
      default:
            Serial.print("rxMsg: unexpected topic: ");
            Serial.println(topic_action);
      break;   
    }
  }
}


void pub_data(){     //float를 int로 변환하여 값의 오류가 존재 ->실수를 정수로 변환 필요
        mdata[0] = ((float)mpu[0]*100) * 2.0 / ((float)100 *32768.0); // Ax
        mdata[1] = ((float)mpu[1]*100-offset[1]) * 2.0 / ((float)100 *32768.0); // Ax
        mdata[2] = ((float)mpu[2]*100) * 2.0 / ((float)100 *32768.0); // Ax
        mdata[4] = ((float)mpu[4]*100-offset[4]) * 250.0 / ((float)100 * 32768.0); //Rx
        buf=String(mdata[0]);
        buf+=",";
        buf+=String(mdata[1]);
        buf+=",";
        buf+=String(mdata[2]);
        buf+=",";
        buf+=String(mdata[4]);
        client.publish(topic_data,buf.c_str());// reward     
        //Serial.printf("pubdata"); 
}
void pub_info(){
    buf=String(Kp);
    buf+=",";
    buf+=String(done);
    client.publish(topic_info,buf.c_str());
    //Serial.printf("pubinfo\n");
}

void wifiConnect(){
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\r\nWiFi connected");
}

void mqttConnect(){
  if(!!!client.connected()){
    Serial.print("Reconnecting MQTT client to ");
    Serial.println(server);
    while(!!!client.connect(clientID)){
      Serial.print(".");
      delay(500);
    }
    Serial.println();
  }
  Serial.print("subscribe to ");
  Serial.print(topic_action);
  if(client.subscribe(topic_action)){
    Serial.println(" OK");
  }else{
    Serial.println(" FAILED");
  }
}
