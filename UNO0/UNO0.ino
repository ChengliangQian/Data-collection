/*
 * -*- coding: utf-8 -*- 
 * UNO0 主机发送转速
 * 连线方式：
 *   A------------------B
 * (10) SS---------->(10) SS
 * (11) MOSI------->(11) MOSI
 * (12) MISO<-------(12) MISO
 * (13) SCLK------->(13) SCLK
 */

#include <SPI.h>

volatile byte rpmcount;
unsigned long rpm;
unsigned long timeold;
float velocity;

void rpm_fun()
 {
 rpmcount++;
 }
 
void setup (void)
{
  Serial.begin(9600);        // 开始串口通讯
  digitalWrite(SS, HIGH);
  SPI.begin ();              // PI通讯开始
  attachInterrupt(0, rpm_fun, FALLING);
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  velocity = 0;
}

void loop (void)
{
  char c;
  char string[25]; 
  delay(99);
  delayMicroseconds(455);
  detachInterrupt(0);   //关中断
  //rpm =(60*4.351*1000/(millis() - timeold))*rpmcount;
 rpm = 122857*rpmcount/(millis() - timeold);    //r/min -- 电机转速
 Serial.println(rpm);
 timeold = millis();
 rpmcount = 0;
 itoa(rpm, string, 10); 
 
  // 片选为从机
  digitalWrite(SS, LOW);    // SS - pin 10
 
  // 发送字串
  for (char * p = string ; c = *p; p++) {
    SPI.transfer (c);
  }
  SPI.transfer ('\n');

  // 取消从机
  digitalWrite(SS, HIGH);
  attachInterrupt(0, rpm_fun, FALLING);
}
