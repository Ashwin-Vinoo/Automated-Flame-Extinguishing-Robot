#include <SoftwareSerial.h>
SoftwareSerial Bluetooth(2,3); // RX, TX
unsigned long time;
char voice_string[100];
byte data[3];
int a[5];
int count=0;
void setup()  
{ 
  Serial.begin(9600);
  Bluetooth.begin(9600);
  Serial.flush();
  Bluetooth.flush();
  pinMode(4,OUTPUT);  //M11
  pinMode(5,OUTPUT);  //M12
  pinMode(6,OUTPUT);  //M21
  pinMode(7,OUTPUT);  //M22
  pinMode(8,OUTPUT);  //Enable for Matlab Bluetooth
  pinMode(9,OUTPUT);  //Enable for Voice Bluetooth
  pinMode(12,OUTPUT); //Flame Extinguisher
  digitalWrite(8,HIGH); 
  digitalWrite(9,HIGH);  
  a[0]=analogRead(A0); 
  a[1]=analogRead(A1); 
  a[2]=analogRead(A2); 
  a[3]=analogRead(A3); 
  a[4]=analogRead(A4); 
  a[5]=analogRead(A5);   
}
void loop() 
{
  if(Serial.available()>3)
  {
    while(Serial.available()!=3)
      Serial.read();
  }
  else if(Serial.available()==3)
  {
    data[0]=Serial.read();
    data[1]=Serial.read();
    data[2]=Serial.read();
    if(data[0]==0)
    {
      count=0;
      if(data[1]==1)
        digitalWrite(12,HIGH);
      else
        digitalWrite(12,LOW);
      if(data[2]==1)
        forward(700);
      else if(data[2]==2)
        right(500);
      else if(data[2]==4)
        left(500);
      else if(data[2]==3)
        backward(700);
      halt();
    }  
    else if(data[0]==1)
    {
      count=0;
      if(data[1]==0&&data[2]>15)
        right(data[2]*1.5);  
      else if(data[1]==1&&data[2]>15)
        left(data[2]*1.5);  
      else
        forward(1000);    
      halt();
      delay(50);
      Serial.write(50);
    }
    else if(data[0]==2)
    {  
      count++;
      if(count>=3)
      {
        left(200);
        halt();   
        delay(50);   
      }
      Serial.write(50);
    }
    else if(data[0]==3&&Bluetooth.available())
    { 
      count=0;     
      while(Bluetooth.available()!=0)
      {
        voice_string[count]=Bluetooth.read();
        count++;  
        delay(2);
      } 
      voice_string[count]='\0';
      count=0;
      if(strstr(voice_string,"forward"))
        forward(900);
      else if(strstr(voice_string,"right"))
      {
        right(700);
        halt();
      }
      else if(strstr(voice_string,"left"))
      {
        left(700);
        halt();
      }
      else if(strstr(voice_string,"back"))
        backward(900);
      else if(strstr(voice_string,"stop"))
        halt();
    }
  } 
}
void forward(int x)
{ 
  digitalWrite(12,LOW);
  time=millis();
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  while(millis()-time<x)
  {  
    if(Serial.available())
      break;
    if((analogRead(A0)<a[0]-28 ||analogRead(A1)<a[1]-28||analogRead(A2)<a[2]-28||analogRead(A3)<a[3]-28||analogRead(A4)<a[4]-28||analogRead(A5)<a[5]-28)&&data[0]==1)
    {
      extinguish();
      break;
    } 
  }
}
void backward(int x)
{
  time=millis();
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  while(millis()-time<x)
  {
    if(Serial.available())
      break;
  }
}
void right(int x)
{
  time=millis();
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);
  while(millis()-time<x)
  {
    if(Serial.available())
      break;
  }
}
void left(int x)
{
  time=millis();
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  while(millis()-time<x)
  {
    if(Serial.available())
      break;
  }
}
void halt()
{
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
}
void extinguish()
{
  halt();
  digitalWrite(12,HIGH);
  left(250);
  delay(50);
  right(500);
  delay(50);
  left(500);
  delay(50);
  right(250);
  digitalWrite(12,LOW); 
}




















