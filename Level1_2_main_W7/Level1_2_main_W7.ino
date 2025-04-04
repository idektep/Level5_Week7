//Node MCU ESP32 Week7
#define LOW_SPEED 125
#define MAX_SPEED 255

uint32_t Data = 0;
long MeasureDistance = 0;
uint8_t LdrMapped = 0;
float Temp = 0;
float Humid = 0;

/*-----------------------------------------------*/
void setup() {
  Serial.begin(9600);
  SensorAndLedSetup();
  MotorDriveSetup();
  Serial.println("Level1-2");
  Welcome();

}
/*-----------------------------------------------*/
void loop() {
  if (Serial.available() > 0) {
    int newData = Serial.parseInt();
    if (newData != 0) {
      Data = newData;
    }
  }
    switch (Data) {

    case 1:  //Test Buzzer
      
      break;

    case 2:  
     
      break;

    case 3:  

      break;

    case 4:  

      break;

    case 5:
      
      break;
    
    case 6:

      break;
      
    case 7:

      break;

    case 8://Stop
      
      break;

    case 9://Reset

      break; 

    case 10: //forward
      Forward(MAX_SPEED);
      delay(3000);
      Stop();
      break; 

    case 11: //backward
      //Add function 
      break; 

    case 12: //rotate right
      //Add function 
      break; 

    case 13: 
      //Add function 
      break; 

    case 14: 
      //Add function 
      break; 

    case 15: 
      //Add function 
      break; 

    case 16: 
      //Add function 
      break; 

    case 17: 
      //Add function 
      break; 

    case 18: 
      //Add function 
      break; 

    case 19: 
      //Add function 
      break; 

  }
}
void LightStop()
{
  Ldr();
  if(LdrMapped < 15)
  {
    Data = 8;
  }
}
void ForwardToWall(uint32_t Speed, uint32_t Distance)
{
  int i = 1;
  do
  {
    Ultrasonic();
    if((MeasureDistance < Distance) && (MeasureDistance > 0))
    {
      Stop();
      delay(10);
      i--;
    }
    else
    {
      DisplayLcd(4, "Auto Car", 0, "");
      Forward(Speed);
      LightStop();
    }
  }while(i);
}