
/**********************************************
// Defalut  ROS Mode
// DEBUG->  Serial Mode
**************************************************/
//#define DEBUG

#define SERIAL_PORT Serial
#define USE_USBCON  // seeeduino xiao model needs this flag to be defined

#ifndef DEBUG
#include <ros.h>
#include <std_msgs/Bool.h>
#endif

#include <TimerTC3.h>
#define txDenPin 2
#define estopPin 3
#define resetPin 4
#define remotePin 5



/*
    status = status | (estop << 2) | (reset << 3) | (remote << 4);

status: normal(00),stop(10),reset(01)
            0x00     0x01     0x02     0x03     0x04     0x05     0x06     0x07
remote |       0|       0|       0|       0|       0|       0|       0|       0| 
reset  |     on0|     on0|     on0|     on0|     on0|     on0|     on0|     on0|
estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
status |      00|      01|      10|      11|      00|      01|      10|      11|
--------------------------------------------------------------------------------
       |  stop10|  stop10|  stop10|  stop10| reset01|normal00| reset01|normal00|
0x02,0x02,0x02,0x02,0x01,0x00,0x01,0x00,
            0x08     0x09     0x0a     0x0b     0x0c     0x0d     0x0e     0x0f
remote |       0|       0|       0|       0|       0|       0|       0|       0| 
reset  |    off1|    off1|    off1|    off1|    off1|    off1|    off1|    off1|
estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
status |      00|      01|      10|      11|      00|      01|      10|      11|
--------------------------------------------------------------------------------
       |  stop10|  stop10|  stop10|  stop10|normal00|normal00|  stop10|  stop10|
0x02,0x02,0x02,0x02,0x00,0x00,0x02,0x02,
            0x10     0x11     0x12     0x13     0x14     0x15     0x16     0x17
remote |       1|       1|       1|       1|       1|       1|       1|       1| 
reset  |     on0|     on0|     on0|     on0|     on0|     on0|     on0|     on0|
estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
status |      00|      01|      10|      11|      00|      01|      10|      11|
--------------------------------------------------------------------------------
       |  stop10|  stop10|  stop10|  stop10| reset01| reset01| reset01| reset01|
0x02,0x02,0x02,0x02,0x01,0x01,0x01,0x01,
            0x18     0x19     0x1a     0x1b     0x1c     0x1d     0x1e     0x1f
remote |       1|       1|       1|       1|       1|       1|       1|       1| 
reset  |    off1|    off1|    off1|    off1|    off1|    off1|    off1|    off1|
estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
status |      00|      01|      10|      11|      00|      01|      10|      11|
--------------------------------------------------------------------------------
       |  stop10|  stop10|  stop10|  stop10|  stop10|  stop10|  stop10|  stop10|
0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02
*/
const char mode[] = { 
0x02,0x02,0x02,0x02,0x01,0x00,0x01,0x00,
0x02,0x02,0x02,0x02,0x00,0x00,0x02,0x02,
0x02,0x02,0x02,0x02,0x01,0x01,0x01,0x01,
0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02};

int len0, len1, len2, val, i;
const char estop_cmd[] = { 0x01, 0x06, 0x20, 0x0E, 0x00, 0x05, 0x23, 0xCA };
const char reset_cmd[] = { 0x01, 0x06, 0x20, 0x0E, 0x00, 0x06, 0x63, 0xCB };
volatile int timercount = 0;
volatile int count = 0;
volatile int status = 1;
volatile int estop = 0;
volatile int remote = 0;
volatile int reset = 0;
#ifndef DEBUG

ros::NodeHandle nh;
std_msgs::Bool navbool_msg;
ros::Publisher pub_navbool("estop", &navbool_msg);

#endif


void TimerCnt() {
  timercount++;
  count++;
}
void setup() {

  SERIAL_PORT.begin(115200);  //  to USBC

#ifndef DEBUG
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(pub_navbool);
#endif

  Serial1.begin(115200);  //  to txDenPin
  pinMode(txDenPin, OUTPUT);
  pinMode(estopPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(remotePin, INPUT_PULLUP);
  status = 0x0c;// normal00
  digitalWrite(txDenPin, LOW);
  TimerTc3.initialize(1000);           //1000us=1ms
  TimerTc3.attachInterrupt(TimerCnt);  
  TimerTc3.start();
  pinMode(LED_BUILTIN, OUTPUT);
  while (SERIAL_PORT.available())
    SERIAL_PORT.read();
  delay(1000);
  
#ifndef DEBUG
  navbool_msg.data = false;
#endif
}
/*
  When txDenPin == 1 Transmit XIAO->ZLAC8015D
  When txDenPin == 0 Receive  XIAO<-ZLAC8015D
  status: normal(0),stop(1),reset(2)

*/

void loop() {
  if (count % 500 == 0) {
    remote = digitalRead(remotePin);
    estop = digitalRead(estopPin);
    reset = digitalRead(resetPin);
    status &= 0x03;
    status = mode[status | (estop << 2) | (reset << 3) | (remote << 4)];
    switch (status) {
      case 0:  // normal -> do nothing
#ifdef DEBUG
        SERIAL_PORT.print("NONE ");
#endif
        break;
      case 1:  // reset
        digitalWrite(txDenPin, HIGH);
        timercount = 0;
        Serial1.write(reset_cmd, sizeof(estop_cmd));
        while (timercount < 1) Serial1.flush();  // delay 1 mSec
        digitalWrite(txDenPin, LOW);
        while (timercount < 2) while (SERIAL_PORT.available()) SERIAL_PORT.read();
        digitalWrite(LED_BUILTIN, HIGH);  //LED OFF
#ifdef DEBUG
        SERIAL_PORT.print("RESET");
#else
        navbool_msg.data = false;
#endif
        break;
      case 2:  // stop
        digitalWrite(txDenPin, HIGH);
        timercount = 0;
        Serial1.write(estop_cmd, sizeof(estop_cmd));
        while (timercount < 1) Serial1.flush();  // delay 1 mSec
        digitalWrite(txDenPin, LOW);
        while (timercount < 2) while (SERIAL_PORT.available()) SERIAL_PORT.read();
        digitalWrite(LED_BUILTIN, LOW);  //LED ON
#ifdef DEBUG
        SERIAL_PORT.print("ESTOP");
#else
        navbool_msg.data = true;
#endif
        break;
      default:
#ifdef DEBUG
        SERIAL_PORT.println("ERROR!!");
#endif
        break;
    }
#ifdef DEBUG
    SERIAL_PORT.print("[");
    SERIAL_PORT.print(status,HEX);
    SERIAL_PORT.print("|");
    if (!estop) SERIAL_PORT.print("E");
    else SERIAL_PORT.print(" ");
    if (!reset) SERIAL_PORT.print("R");
    else SERIAL_PORT.print(" ");
    if (remote) SERIAL_PORT.print("W");
    else SERIAL_PORT.print(" ");
    SERIAL_PORT.print(remote);
    SERIAL_PORT.print(reset);
    SERIAL_PORT.print(estop);
    SERIAL_PORT.println("]");
    SERIAL_PORT.flush();
#else
    pub_navbool.publish(&navbool_msg);
    nh.spinOnce();
#endif
  }
}

