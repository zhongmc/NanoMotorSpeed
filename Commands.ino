#include <Arduino.h>

void MoveAdfMotor(int leftSpeed, int rightSpeed );   //0-255 ;
extern bool mROSConnected;
extern double d_kp, d_ki, d_kd, v_kp, v_ki, v_kd;
extern volatile long count1, count2;

extern bool simulateMode;
extern double left_ticks, right_ticks;
extern long prev_left_ticks, prev_right_ticks;          

extern double v, w, drive_v, drive_w, ctrl_v, ctrl_w, ctrl_vel_l, ctrl_vel_r;

char comData[52];
int comDataCount = 0;

//ble nano 30 hm10 10
#define SEND_INTERVAL 30


uint8_t queueLen = 0;
bool queueIdle = true; 
long lastPkgMillis;


void initCommands()
{
    comDataCount = 0;
    queueIdle = true;
    queueLen = 0;
    lastPkgMillis = millis();
}

void checkCommands()
{

  //read speed setting from serial
  char ch;
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      ch = Serial.read();
      if (ch == ';' || ch == '\r' || ch == '\n') //new command
      {
        comData[comDataCount] = 0;
        processCommand(comData, comDataCount);
        comDataCount = 0;
        continue;
      }

      comData[comDataCount++] = ch;
      
      if (comDataCount > 50) //some error
      {
        SendMessages("CMD too long!");
        comDataCount = 0;
      }
    }
  }    

}


void processCommand(char *buffer, int bufferLen)
{
  *(buffer + bufferLen) = 0;
  if (bufferLen < 2)
  {
    if (buffer[0] == 'b') //get baundrate
      Serial.println(115200);
    else if (buffer[0] == 'p')
      Serial.println(100);
    return;
  }

  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);

  if (ch0 == 's' && ch1 == 'd') //set drive Goal
  {
    double v, w = 0;

    v = atof(buffer + 2);
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      w = atof(buf + 1);
    // Serial.print("_sd");
    // Serial.print(v);
    // Serial.print(',');
    // Serial.println(w);
    setDriveGoal(v, w);
    return;
  }

  else if (ch0 == 'g' && ch1 == 'r')
  {
    // SendMessages("== OK ==;\n");
    SendMessages("_OK_:XYQ:%s,%s,%s;\n", 
          floatToStr(0, x),
          floatToStr(1, y),
          floatToStr(2, theta)
          );

    return;
  }

  else if (ch0 == 'c' && ch1 == 'i') //count info
  {
    int c1,c2;
    c1 = count1;
    c2 = count2;
    SendMessages("-ci:%d,%d\n", c1, c2);
    return;
  }
  else if( ch0 == 'i' && ch1 =='c') //read i2c counter;
  {
    readCounter();
    int c1,c2;
    c1 = count1;
    c2 = count2;
    SendMessages("-ci:%d,%d\n", c1, c2);
    return;
  }


  else if (ch0 == 'm' && ch1 == 'm') // move motor mmpwml,pwmr
  {
    int pwml = atoi(buffer + 2);
    char *buf = strchr((buffer + 2), ',');

    int pwmr = pwml;
    if (buf != NULL)
      pwmr = atoi(buf + 1);

    if( !simulateMode )
      MoveAdfMotor(pwml, pwmr);

    motorSpeed(pwml, pwmr);
    MoveMotor(0);

    if( !simulateMode )
      MoveAdfMotor(0, 0);

    return;
  }

  else if( ch0 == 'c' && ch1 == 'r') //ros connected
  {
    SendMessages(" ROS Connected OK!\n");
    mROSConnected = true;
    return;
  }

  else if( ch0 == 'b' && ch1 == 'r') // ble connected
  {
    SendMessages(" BLE Connected OK! \n");
    mBleConnected = true;
    return;
  }
  else if(ch0 == 's' && ch1 == 'i') //settings info
  {
    SendSettings();
    return;
  }

  else if (ch0 == 'p' && ch1 == 'i') //set pid cmd: pi type kp,ki,kd;
  {
    setPID(buffer + 2);
    return;
  }

  else if (ch0 == 's' && ch1 == 'd') //set drive Goal
  {
    double v, w = 0;

    v = atof(buffer + 2);
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      w = atof(buf + 1);
    // Serial.print("_sd");
    // Serial.print(v);
    // Serial.print(',');
    // Serial.println(w);
    setDriveGoal(v, w);
    return;
  }

  else if (ch0 == 's' && ch1 == 'm') //simulate mode //simulate mode sm0 sm1 sm2; 0: cancel simulate mode 1:simulate with the obstacle; 2: simulate with obstacle plus motor
  {
    int val = atoi(buffer + 2);
    setSimulateMode( val );
    return;
  }

  else if (ch0 == 's' && ch1 == 't') //stop
  {
    stopRobot();
    return;
  }

  else if( ch0 == 'r' && ch1 == 's') //reset robot
  {
    resetRobot();
    resetController();
    Serial.println("_rs ok!;");
    return;
  }

  else if (ch0 == 'g' && ch1 == 'g') //set goto goal goal
  {
      int ival;
      double _x, _y, _v;
      int angle;
      ival = atoi((buffer+2));
      _x = (float)ival/10.0;
      char *buf = strchr(buffer, ',');
      ival = atoi((buf+1));
      _y = (float)ival/10.0;
      
      buf = strchr((buf + 1), ',');
      angle = atoi(buf + 1);
      
      buf = strchr((buf + 1), ',');
      ival = atoi((buf+1));
      _v = (float)ival/100.0;
      double _theta;
      if (angle <= 180)
        _theta = (angle * PI) / 180.0;
      else
      {
        angle = angle - 360;
        _theta = (angle * PI) / 180.0;
      }

    Serial.print("_gg:");
    Serial.print(_x);
    Serial.print(',');
    Serial.println(_y);

    setGTGGoal(_x, _y, _theta, _v);
    return;

  }
  else if( ch0 == 'g' && ch1 == 'o') //start goto goal
  {
    startGTG();
    return;
  }  

  else if (ch0 == 't' && ch1 == 'l') //turn around left/ right(-pwm) test tl0/1/2,360; tl dir, angle;
  {
    int dir = atoi(buffer + 2);
    int angle = 360;
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      angle = atof(buf + 1);

    turnAround(dir, angle);
    return;
  }

     Serial.print("Na:");
     Serial.println(buffer );
}




//cmd: pi type kp,ki,kd; type 2,3,4
void setPID(char *buffer)
{
  double p, i, d;
  int type = *buffer - '0';

  p = atof((buffer+1));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);
  //  SendMessages("PID:%d,%d,%d,%d\n",
  //   type, 
  //   (int)100*p,
  //   (int)1000*i,
  //   (int)10000*d
  // );

  Serial.print("_PID:");
  Serial.print(type);
  Serial.print(',');
  Serial.print(p);
  Serial.print(',');
  Serial.print(i);
  Serial.print(',');
  Serial.println(d);

  if( type == 1 )
  {
    d_kp = p;
    d_ki = i;
    d_kd = d;
  }
  else if( type == 4 )
  {
    v_kp = p;
    v_ki = i;
    v_kd = d;
  }
}


void motorSpeed(int pwml, int pwmr)
{
  long c1, c2, lt;
  MoveLeftMotor(pwml);
  MoveRightMotor(pwmr);

  delay(500);
  c1 = count1;
  c2 = count2;
  lt = millis();
  delay(1000);
  lt = millis() - lt;

  int ic1 = (int) (count1 - c1);
  int ic2 = (int) (count2 - c2);
  int lt1 = (int)lt;
  SendMessages("%d,%d,%d,%d,%d\n", pwml, pwmr, lt1, ic1, ic2);
}



void floatToByte(byte *arrayBuf, double val, double scale)
{
  int tmp = (int)(val * scale);
  if (tmp > 0)
  {
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp;
  }
  else
  {
    tmp = -tmp;
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp | 0x80;
  }
}


void log(const char *format, ...)
{
  char tmp[500];
  va_list vArgList;
  va_start(vArgList, format);
  // sniprintf(tmp, 490, format, vArgList);
  vsprintf(tmp, format, vArgList);
  va_end(vArgList);
  Serial.print(tmp);
}




void SendCtrlInfoValue()
{
  //0xA1, 19, x*1000,y*1000,theta*1000, leftCoder, rightCoder
  byte buf[20];
  memset(buf, 0, 20);
  buf[0] = 0xA3;  //A为二进制报文标志，0 为报文类型
  buf[1] = 16;   //长度

  floatToByte(buf + 2, v, 1000);
  floatToByte(buf + 4, w, 1000);
  floatToByte(buf + 6, drive_v, 1000);
  floatToByte(buf + 8, drive_w, 1000);
  floatToByte(buf + 10, ctrl_v, 1000);
  floatToByte(buf + 12, ctrl_w, 1000);
  floatToByte(buf + 14, ctrl_vel_l, 1000);
  floatToByte(buf + 16, ctrl_vel_r, 1000);
  if( queueIdle == true )
  {
    queueIdle = false;
    for( int i=0; i<18; i++)
    {
      Serial.write(buf[i]);
    }    
    Serial.flush();
    lastPkgMillis = millis();
  }
  else
  {
    addData(buf, 18);
  }
}



void SendRobotStateValue()
{
  //0xA1, 19, x*1000,y*1000,theta*1000, leftCoder, rightCoder
  byte buf[20];
  memset(buf, 0, 20);
  buf[0] = 0xA1;  //A为二进制报文标志，0 为报文类型
  buf[1] = 16;   //长度
  double scale = 1000;
  floatToByte(buf + 2, x, scale);
  floatToByte(buf + 4, y, scale);
  floatToByte(buf + 6, theta, scale);
  // floatToByte(buf + 8, pos.theta, scale);
  buf[8] = (int)(100.0 * v);
  buf[9] = 51; //(byte)(int)(10.0 * 5); //voltage

  long c1, c2;
  c1 = count1;
  c2 = count2;

  memcpy(buf + 10, &c1, 4);
  memcpy(buf + 14, &c2, 4);

  if( queueIdle == true )
  {
    queueIdle = false;
    for( int i=0; i<18; i++)
    {
      Serial.write(buf[i]);
    }    
    Serial.flush();
    lastPkgMillis = millis();
  }
  else
  {
    addData(buf, 18);
  }
}


char tmp[20][15];

const char *floatToStr(int idx, double val)
{

  return floatToStr(idx, (signed char)6, (unsigned char)3, val);
}

const char *floatToStr(int idx, signed char width, unsigned char prec, double val)
{
  if (idx >= 19)
    return NULL;
  return dtostrf(val, width, prec, tmp[idx]);
}


extern int minRPM, maxRPM;
extern double r, l;

void SendSettings()
{
      //sampleTime minRMPM maxRPM,r, l, atO,dfw, unsafe,maxW
    SendMessages("ROP30,%d,%d,%s,%s,0.0,0.0,0.0,0.0;\n", 
          minRPM, maxRPM, 
          floatToStr(0, r),
          floatToStr(1, l)
          );
//que len too small (only 3);
    delay(40);
    doSendRemainedPkg();
    delay(40);
    doSendRemainedPkg();
    delay(40);
    doSendRemainedPkg();

    SendMessages("PID1,%s,%s,%s;\n", 
      floatToStr(0, d_kp),
      floatToStr(1, d_ki),
      floatToStr(2, d_kd));
  
    delay(40);
    doSendRemainedPkg();
    delay(40);
    doSendRemainedPkg();

    SendMessages("PID4,%s,%s,%s;\n", 
      floatToStr(0, v_kp),
      floatToStr(1, v_ki),
      floatToStr(2, v_kd));

}


//发送数据，（透传时，同时写入BLE）,需要按19（20）字节分包，默认第一个包直接发送，
//剩下的发送到队列中


void SendMessages(const char *format, ...)
{
  char tmp[50];
  memset(tmp, 0, 50);
  
  va_list vArgList;
  va_start(vArgList, format);
  // vsniprintf(tmp, 20, format, vArgList);
  vsnprintf(tmp, 50, format, vArgList);
  va_end(vArgList);
  int len = strlen(tmp);
  if( !mBleConnected )  //return
  {
    for( int i=0; i<len; i++)
    {
      Serial.write((byte)*(tmp + i));
    }    
    Serial.flush();
    return;
  }

/////////////
  int ll;
  if( queueIdle == true )
  {
    queueIdle = false;
    if( len > 19 )
      ll = 19;
    else 
      ll = len;

    for( int i=0; i<ll; i++)
    {
      Serial.write((byte)*(tmp + i));
    }    
    Serial.flush();

    lastPkgMillis = millis();

  }
  else
  {
     ll = 0;
  }
  //将数据写入队列，等待发送
  while( ll < len )
  {
    uint8_t l = len - ll;
    if( l > 19 )
      l = 19;
    addData((tmp + ll), l );
    ll = ll + l; 
  }
  return;
}

void doSendRemainedPkg()
{
  if( millis() - lastPkgMillis < SEND_INTERVAL )
  {
    return;
  }
  if( isEmpty() )
  {
    queueIdle = true;
    return;
  }
  queueIdle = false;
  byte buf[20];
  int len = pullData( buf );
  if( len <= 0 )
    return;
  for (int i = 0; i < len; i++)
  {
    Serial.write( buf[i]);
  }

  Serial.flush();
  lastPkgMillis = millis();
}


void setSimulateMode( int val )
{

    if( val == 0 )
    {
      Serial.println("close sm mode!");
      prev_left_ticks = readLeftEncoder();
      prev_right_ticks = readRightEncoder();          
      simulateMode = false;
    }
    else
    {

      left_ticks = 0;
      right_ticks = 0;
      prev_left_ticks = 0;
      prev_right_ticks = 0;          
      simulateMode = true; 
      Serial.println("Simulate mode!");
    }

}

//BLE分包，待发送数据队列实现
// uint8_t queueLen = 0;
// bool queueIdle = true; 
char dataBuf[3][20];
uint8_t dataLens[3];

int addData(char *buf, uint8_t len )
{
  if( queueLen > 2 )
    return -1; //full

  memset(dataBuf[queueLen], 0, 20);
  memcpy( dataBuf[queueLen], buf, len );
  dataLens[queueLen] = len;
  queueLen++;
  return queueLen;
}

int pullData(char *buf )
{
  
    if( queueLen == 0 )
      return -1; //empty
    
    uint8_t len = dataLens[0];
    memcpy( buf, dataBuf[0], len );
    for( int i=0; i<queueLen - 1; i++ )
    {
      dataLens[i] = dataLens[i+1];
      memcpy(dataBuf[i], dataBuf[i+1], 20);
    }
    queueLen--;
    return len;
}

bool isEmpty()
{
  return (queueLen == 0);
}
