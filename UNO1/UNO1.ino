

/*
 * -*- coding: utf-8 -*- 
 * 
 * 带Xbee的UNO
 * 
 * UNO1 接收发送转速
 * 连线方式：
 *   A------------------B
 * (10) SS---------->(10) SS
 * (11) MOSI------->(11) MOSI
 * (12) MISO<-------(12) MISO
 * (13) SCLK------->(13) SCLK
 * 
 * TY901  :IIC
 *         
 *         
 * 接收机中立点1524ms  最大值2060
 * 
 * 电流传感器 模拟0
 * 电压传感器 模拟1
 */





#include <SPI.h>
#include <Wire.h>
#include <JY901.h>
#include <SoftwareSerial.h>

int currpin=0;    //电流传感器 模拟0
int volpin=1;     //电压传感器 模拟1
int ppm1 = 2;     //UNO1数字2   数字2  int0
char buf [100];
volatile byte pos;
volatile boolean process_it;
unsigned long rc1_PulseStartTicks;      
volatile int rc1_val;
int rpm;
volatile float curr=0;
volatile float vol=0;
volatile float power;
 
 
void setup (void)
{ 
  
  Serial.begin (9600,SERIAL_8E1);
  pinMode(MISO, OUTPUT);
  JY901.StartIIC();

  // 设置为接收状态
  SPCR |= _BV(SPE);

  // 准备接受中断
  pos = 0;   // 清空缓冲区
  process_it = false;

  // 开启中断
  SPI.attachInterrupt();

          //PPM inputs from RC receiver
        pinMode(ppm1, INPUT); 

        // 电平变化即触发中断
        attachInterrupt(0, rc1, CHANGE);  
}


// SPI 中断程序
ISR (SPI_STC_vect)
{
  byte c = SPDR;  // 从 SPI 数据寄存器获取数据
  if (pos < sizeof(buf))
  {
    buf [pos++] = c;
    if (c == '\n')
      process_it = true;
  }
}

void printingVelo()
{
    if (process_it)
    {
      buf [pos] = 0;  
      //Serial.println (buf);
      rpm=atoi(buf);
      //Serial.print("RPM:");
      //Serial.println(rpm);
      float temp;
      temp=rpm/60;
      float velocity;
      velocity=temp*0.041758;         //Km/h
     // Serial.print("V:");
      Serial.println(rpm);
      pos = 0;
      process_it = false;
    } 
  }

void printingAcc()
{
    JY901.GetAcc();
    JY901.GetAngle();
    //Serial.print("OA:");Serial.print(" ");Serial.print((float)JY901.stcAcc.a[1]/32768*16);Serial.println("");
   // Serial.print("Acc:");Serial.print(" ");
    Serial.print((float)JY901.stcAcc.a[1]/32768*16-sin(0.01745*(float)JY901.stcAngle.Angle[0]/32768*180));Serial.println("");
    
  }
void printingAngle()
{
  JY901.GetAngle();
  //Serial.print("Ang:");
  Serial.print((-1)*(float)JY901.stcAngle.Angle[0]/32768*180);Serial.println(" ");  
  }


void printingCur()
{
  float temp;
  temp=((analogRead(currpin)-512)*0.12207);
  curr=temp-0.73;
  //Serial.print("Current:");
  //Serial.println(curr);
  delay(5);
  }

void printingVol()
{
  float temp;
  temp=analogRead(volpin);
  vol=0.0244*temp;
  //Serial.print("Voltage:");
  //Serial.println(vol);
  }

void rc1()
{
        // did the pin change to high or low?
        if (digitalRead( ppm1 ) == HIGH)
                rc1_PulseStartTicks = micros();    // store the current micros() value
        else
                rc1_val = micros() - rc1_PulseStartTicks; 
}

void printingPPM()
{
     Serial.print("cha2: ");
     Serial.println(rc1_val); 
     Serial.print("duty-cycle: ");
     Serial.print(abs(rc1_val-1524)/536*100);
     Serial.println("% ");
      
  }

void printingPower()
{
  power=vol*curr;
  //Serial.print("P:");
  Serial.println(power);
  //Serial.println("W");
  }
  
void loop (void)
{
  printingVelo();
  printingAcc();
  printingCur();
  printingVol();
  printingPower();
  printingAngle();
  Serial.println(" ");
  delay(100); 
}




