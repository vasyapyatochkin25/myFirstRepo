#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Arduino_FreeRTOS.h>

#define BUTTON A5
#define POWER 2
#define A 3
#define B A4
#define LED 13
#define PWM_Pin 10
#define PERIOD 1000UL

const int rs = 9, en = 8, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

byte bukva_B[8]   = {B11110,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // Буква "Б"
byte bukva_G[8]   = {B11111,B10001,B10000,B10000,B10000,B10000,B10000,B00000,}; // Буква "Г"
byte bukva_D[8]   = {B01111,B00101,B00101,B01001,B10001,B11111,B10001,B00000,}; // Буква "Д"
byte bukva_ZH[8]  = {B10101,B10101,B10101,B11111,B10101,B10101,B10101,B00000,}; // Буква "Ж"
byte bukva_Z[8]   = {B01110,B10001,B00001,B00010,B00001,B10001,B01110,B00000,}; // Буква "З"
byte bukva_I[8]   = {B10001,B10011,B10011,B10101,B11001,B11001,B10001,B00000,}; // Буква "И"
byte bukva_IY[8]  = {B01110,B00000,B10001,B10011,B10101,B11001,B10001,B00000,}; // Буква "Й"
byte bukva_L[8]   = {B00011,B00111,B00101,B00101,B01101,B01001,B11001,B00000,}; // Буква "Л"
byte bukva_P[8]   = {B11111,B10001,B10001,B10001,B10001,B10001,B10001,B00000,}; // Буква "П"
byte bukva_Y[8]   = {B10001,B10001,B10001,B01010,B00100,B01000,B10000,B00000,}; // Буква "У"
byte bukva_F[8]   = {B00100,B11111,B10101,B10101,B11111,B00100,B00100,B00000,}; // Буква "Ф"
byte bukva_TS[8]  = {B10010,B10010,B10010,B10010,B10010,B10010,B11111,B00001,}; // Буква "Ц"
byte bukva_CH[8]  = {B10001,B10001,B10001,B01111,B00001,B00001,B00001,B00000,}; // Буква "Ч"
byte bukva_Sh[8]  = {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00000,}; // Буква "Ш"
byte bukva_Shch[8]= {B10101,B10101,B10101,B10101,B10101,B10101,B11111,B00001,}; // Буква "Щ"
byte bukva_Mz[8]  = {B10000,B10000,B10000,B11110,B10001,B10001,B11110,B00000,}; // Буква "Ь"
byte bukva_IYI[8] = {B10001,B10001,B10001,B11001,B10101,B10101,B11001,B00000,}; // Буква "Ы"
byte bukva_Yu[8]  = {B10010,B10101,B10101,B11101,B10101,B10101,B10010,B00000,}; // Буква "Ю"
byte bukva_Ya[8]  = {B01111,B10001,B10001,B01111,B00101,B01001,B10001,B00000,}; // Буква "Я"
byte Kapla[8] = {0b00000,0b00100,0b01110,0b11111,0b11111,0b11111,0b01110,0b00000}; 
byte heart[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000 };
byte smile[8] = {B00000, B00000,  B01010,  B01010, B00000,B10001, B01110, B00000 };
byte smiley[8] = {0b00000,0b00000,0b01010,0b00000,0b00000,0b10001,0b01110,0b00000};
byte frownie[8] = {0b00000,0b00000,0b01010,0b00000,0b00000,0b00000,0b01110,0b10001};
byte armsDown[8] = {0b00100,0b01010,0b00100,0b00100,0b01110,0b10101,0b00100,0b01010};
byte armsUp[8] = {0b00100,0b01010,0b00100,0b10101,0b01110,0b00100,0b00100,0b01010};

String str="";

unsigned char Encoder_cnt;
unsigned char PWM_Speed;
 
volatile uint32_t timer_cur,timer,Time_Set,Time_Current,Time_Motor_On_Set;
volatile uint8_t TimSecond,TimMinutes,TimHourse,Tim_MotorOn_Second,Tim_MotorOn_Minutes,Tim_MotorOn_Hourse,TimKalibrate;
volatile uint8_t TimKalibrateHourse,TimkalibrateMinutes,TimKalibrateSecond;
volatile uint8_t Timer_Supplies;

volatile uint8_t BlokPrintMainMenu=0;
volatile uint8_t Y=0;
volatile uint8_t Cnt=0,Cnt_1=0;
volatile String Str_Comand_print="";
volatile bool Flag_PolivOn =0;
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );


 void setup() 
 {  
  pinMode(PWM_Pin,OUTPUT);
  digitalWrite(PWM_Pin,LOW);

  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
 
  pinMode(BUTTON,INPUT_PULLUP);
  digitalWrite(BUTTON, HIGH);
  

  pinMode(POWER,INPUT_PULLUP);
  pinMode(A,INPUT_PULLUP);
  pinMode(B,INPUT_PULLUP);
  
  
  Serial.begin(115200);
  analogReference(INTERNAL);

  //EEPROM_clear();
  Timer_Supplies = EEPROM.read(0);

  PWM_Speed = EEPROM.read(1);
  TimSecond = EEPROM.read(2);
  TimMinutes = EEPROM.read(3);
  TimHourse = EEPROM.read(4);
  
  Tim_MotorOn_Hourse = EEPROM.read(5);
  Tim_MotorOn_Minutes = EEPROM.read(6);
  Tim_MotorOn_Second = EEPROM.read(7);

  TimKalibrateHourse = EEPROM.read(8);
  TimkalibrateMinutes = EEPROM.read(9);
  TimKalibrateSecond = EEPROM.read(10);
  
  
  Time_Set = (TimHourse*3600)+(TimMinutes*60)+TimSecond;
  Time_Motor_On_Set = (Tim_MotorOn_Hourse*3600)+(Tim_MotorOn_Minutes*60)+Tim_MotorOn_Second;
  TimKalibrate = (TimKalibrateHourse*3600)+(TimkalibrateMinutes*60)+TimKalibrateSecond;
  
  Serial.println(TimSecond);
  Serial.println(TimMinutes);
  Serial.println(TimHourse);
  Serial.println(Time_Set);
  
  Serial.println(Tim_MotorOn_Hourse);
  Serial.println(Tim_MotorOn_Minutes);
  Serial.println(Tim_MotorOn_Second);
  Serial.println(Time_Motor_On_Set);
  
  Serial.println(TimKalibrateHourse);
  Serial.println(TimkalibrateMinutes);
  Serial.println(TimKalibrateSecond);
  Serial.println(TimKalibrate);
  
  attachInterrupt(1,Encoder,RISING);
  

// Пины D9 и D10 - 976 Гц
TCCR1A = 0b00000001;  // 8bit
TCCR1B = 0b00001011;  // x64 fast pwm


  Blink_Led(LED,300);
  Init_LCD();


    // Now set up two tasks to run independently.


  xTaskCreate(
    TaskTimer
    , "Timer"
    , 256
    , NULL
    , 1
    , NULL);

  xTaskCreate(
    TaskButton
    , "Button"
    , 256
    , NULL
    , 1
    , NULL); 
}



void TaskButton(void *pvParameters)
{
  (void) pvParameters;

   for(;;)
   {
    //Serial.println("Taskbutton");
   if (!digitalRead(BUTTON))
   
    switch (Button_clik(BUTTON))
    {
      case 1:
        BlokPrintMainMenu = 1;
        delay(100);
            lcd.clear();  
        Print_Menu(6,0);
        delay(2000);
        Settings_Settings();
        Print_Main_Menu(1,0);
        
        BlokPrintMainMenu = 0;
      break;

      default:
      break;
      
    }
   }
}


void TaskTimer(void *pvParameters)
{
  (void) pvParameters;

   for(;;)
   {
   // vTaskDelay( 10 / portTICK_PERIOD_MS ); // ждать одну секунду
    if ((millis() - timer) >= 1000) 
    {
    // ваше действие
      timer += 1000;
      timer_cur+=1000;
      Time_Current = Conver_Time_MS_to_S(timer_cur);
      
      if (!BlokPrintMainMenu)
        Print_Current_Time(8,1,timer_cur);
    }
    
    if (Timer_Supplies)
      Cycle_Time(Time_Motor_On_Set);
    else
      Cycle_Time(TimKalibrate);
   }
}



void loop() 
{
   // Serial.println(analogRead(A1));
    /*
    if ((millis() - timer) >= 1000) 
    {
    // ваше действие
      timer += 1000;
      timer_cur+=1000;
      Print_Current_Time(8,1,timer_cur);
    }

    if (Timer_Supplies)
      Cycle_Time(Time_Motor_On_Set);
    else
      Cycle_Time(TimKalibrate);
      */
    /*  
  if (!digitalRead(BUTTON))
    if(Button_bounce(BUTTON))
    {
      Button_realising(BUTTON);
      Menu();
      Print_Main_Menu(1,0);
    }
*/

}

void Init_LCD()
{
  lcd.begin(16,2);
  Print_Hi();
  delay(1500);
  lcd.clear();
  Print_Main_Menu(1,0);
}

void Encoder()
{  
  if (!digitalRead(B))
  {
    if (Encoder_cnt!=0xFF)
    Encoder_cnt++;
  }
    
  else
  {
    if (Encoder_cnt!=0)
    Encoder_cnt--;
  }   
  Cnt_1=Cnt;
  Cnt = Encoder_cnt;
        
  if (Cnt>Cnt_1)
    Y=0;
  else
    Y=1;
}

void EEPROM_clear()
{
  for (int i = 0;i<256;i++)
  {
    EEPROM.write(i, 0);
  }
}


void Blink_Led(byte pin,uint16_t del)
{
    digitalWrite(pin,HIGH);
    delay(del);
    digitalWrite(pin,LOW);
    delay(200);
    digitalWrite(pin,HIGH);
    delay(200);
    digitalWrite(pin,LOW);
    delay(200);
    digitalWrite(pin,HIGH);
    delay(200);
    digitalWrite(pin,LOW);
    delay(200);
    digitalWrite(pin,HIGH);  
}


void Rx_Serial(String *data)
{
  while (Serial.available())
  {
    *data += char(Serial.read());
   delay(2);
  }
}
/***********************************************************************************************************/
/***********************************************************************************************************/
void print_(byte *data, byte pos_X,byte pos_Y,byte addr)
{
    lcd.createChar(addr, data);
    lcd.setCursor(pos_X, pos_Y);
    lcd.write(byte(addr));  
}
void Print_Sleep(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);
  lcd.print("C");
  print_(bukva_P,pos_X+1,pos_Y,0);
  print_(bukva_Ya,pos_X+2,pos_Y,1);
  print_(bukva_Shch,pos_X+3,pos_Y,2);
  print_(bukva_I,pos_X+4,pos_Y,3);
  print_(bukva_IY,pos_X+5,pos_Y,4);
  lcd.print(" PE");
  print_(bukva_ZH,pos_X+9,pos_Y,5);
  lcd.write(byte(3));
  lcd.print("M");
  
}

void Print_Menu(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);
  lcd.print("MEH");
  print_(bukva_Yu,pos_X+3,pos_Y,0);

  lcd.setCursor(pos_X-2, pos_Y);
  lcd.print("*");
  lcd.setCursor(pos_X+5, pos_Y);
  lcd.print("*");
  delay(100);
  lcd.setCursor(pos_X-3, pos_Y);
  lcd.print("*");
  lcd.setCursor(pos_X+6, pos_Y);
  lcd.print("*");
  delay(100);
  lcd.setCursor(pos_X-4, pos_Y);
  lcd.print("*");
  lcd.setCursor(pos_X+7, pos_Y);
  lcd.print("*");
  delay(100);
    lcd.setCursor(pos_X-5, pos_Y);
  lcd.print("*");
  lcd.setCursor(pos_X+8, pos_Y);
  lcd.print("*");
  delay(100);
    lcd.setCursor(pos_X-6, pos_Y);
  lcd.print("*");
  lcd.setCursor(pos_X+9, pos_Y);
  lcd.print("*");
  delay(100);


  
} 

void Print_Realise_Button(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);
  lcd.print("OT");
  print_(bukva_P,pos_X+2,pos_Y,0);
  print_(bukva_Y,pos_X+3,pos_Y,1);
  lcd.print("CT");
  print_(bukva_I,pos_X+6,pos_Y,2);
  lcd.print("TE KHO");
  lcd.write(byte(0));
  lcd.print("K");
  lcd.write(byte(1));
}

void Print_kalibrovka(byte pos_X,byte pos_Y)
{
  ////"KA***POBKA"
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("KA");
  print_(bukva_L,pos_X+2,pos_Y,1);
  print_(bukva_I,pos_X+3,pos_Y,2);
  print_(bukva_B,pos_X+4,pos_Y,3);
  lcd.print("POBKA");
}

void Print_settings(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("HACTPO");
  print_(bukva_IY,pos_X+6,pos_Y,0);
  lcd.print("K");
  print_(bukva_I,pos_X+8,pos_Y,1);
    
}

void Print_Speed_Motor(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("CKOPOCT");
  print_(bukva_Mz,pos_X+7,pos_Y,0);
  lcd.print(" ");
  print_(bukva_P,pos_X+9,pos_Y,1);
  lcd.print("O");
  print_(bukva_D,pos_X+11,pos_Y,2);
  lcd.print("A");
  print_(bukva_CH,pos_X+13,pos_Y,3);
  print_(bukva_I,pos_X+14,pos_Y,4);

  
}

void Print_Hand_Control(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("P");
  print_(bukva_Y,pos_X+1,pos_Y,0);
  print_(bukva_CH,pos_X+2,pos_Y,1);
  lcd.print("HOE");

  lcd.setCursor(pos_X, pos_Y+1);
  
  lcd.write(byte(0));
  print_(bukva_P,pos_X+1,pos_Y+1,2);
  lcd.print("PAB");
  print_(bukva_L,pos_X+5,pos_Y+1,3);
  lcd.print("EH");
  print_(bukva_I,pos_X+8,pos_Y+1,4);
  lcd.print("E");
  
}

void Print_Timer(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("TA");
  print_(bukva_IY,pos_X+2,pos_Y,0);
  lcd.print("MEP");

}

void Print_Hi()
{
  print_(bukva_P,5,0,0);
  lcd.print("P");
  print_(bukva_I,7 ,0,1);
  lcd.print("BET");
}

void Print_Main_Menu(byte pos_X,byte pos_Y)
{
  lcd.clear();
  lcd.setCursor(pos_X, pos_Y);
  
  lcd.print("CTAT");
  print_(bukva_Y,pos_X+4,pos_Y,0);
  lcd.print("C");
  lcd.print(": ");
  //print_(Kapla,pos_X+8,pos_Y,1);
  
  lcd.setCursor(pos_X, pos_Y+1);
  lcd.print("BPEM");
  print_(bukva_Ya,pos_X+4,pos_Y+1,2);
  lcd.print(": ");
}

void Print_On_poliv(byte pos_X,byte pos_Y)
{
  print_(Kapla,pos_X,pos_Y,7); 
}

void Print_Off_poliv(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X,pos_Y);
  lcd.print(" ");
}

void Print_Current_Time(byte pos_X,byte pos_Y,uint32_t Time)
{  
  static uint8_t Second,Minutes,Hourse;
  Time=Time/1000;
  
  Hourse = Time/3600;
  Minutes = (Time%3600)/60;
  Second = (Time%3600)%60;
  
  Print_Current_Timer(pos_X,pos_Y,Hourse,Minutes,Second);

}

void Print_Timer_Motor_On(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("BPEM");
  print_(bukva_Ya,pos_X+4,pos_Y,0);
  lcd.print(" ");
  print_(bukva_P,pos_X+6,pos_Y,1);
  lcd.print("O");
  print_(bukva_D,pos_X+8,pos_Y,2);
  lcd.print("A");
  print_(bukva_CH,pos_X+10,pos_Y,3);
  print_(bukva_I,pos_X+11,pos_Y,4);

}

void Print_Current_Timer(byte pos_X,byte pos_Y,byte HH,byte MM,byte SS)
{
  lcd.setCursor(pos_X, pos_Y);

  if (HH<10)
    lcd.print("0");
  lcd.print(HH);
  lcd.print(":");
  if (MM<10)
    lcd.print("0");  
  lcd.print(MM);
  lcd.print(":");
  if (SS<10)
    lcd.print("0");
  lcd.print(SS);
}

void Print_Hand(byte pos_X,byte pos_Y)
{
    lcd.setCursor(pos_X, pos_Y);
  lcd.print("CTAT");
  print_(bukva_Y,pos_X+4,pos_Y,0);
  lcd.print("C");
  lcd.print(": ");
  
  lcd.setCursor(pos_X, pos_Y+1);
  lcd.print("CKOPOCT");
  print_(bukva_Mz,pos_X+7,pos_Y+1,1);
  lcd.print(": ");
}

void Print_Timer_Supplies(byte pos_X,byte pos_Y)
{
  lcd.setCursor(pos_X, pos_Y);

  lcd.print("B");
  print_(bukva_IYI,pos_X+1,pos_Y,5);
  print_(bukva_B,pos_X+2,pos_Y,6);
  lcd.print("OP ");
  
  lcd.print("TA");
  print_(bukva_IY,pos_X+8,pos_Y,0);
  lcd.print("MEPA");
  
  print_(bukva_P,pos_X,pos_Y+1,1);
  lcd.print("O");
  print_(bukva_D,pos_X+2,pos_Y+1,2);
  lcd.print("A");
  print_(bukva_CH,pos_X+4,pos_Y+1,3);
  print_(bukva_I,pos_X+5,pos_Y+1,4);
}

inline void Timer_Supples_()
{
  lcd.clear();
  Print_Timer_Supplies(1,0);
  delay(2000);
  lcd.clear();
  Print_kalibrovka(1,0);
  Print_Timer(1,1);

  uint16_t TimeOut = 500000; 

  Encoder_cnt = 0;
   Y = Timer_Supplies;
  while(TimeOut--)
    {    
        if (Y)
        {
           print_(armsUp,15,Y,7);
           lcd.setCursor(15,0);
           lcd.print(" ");
        }
        else
        {
           print_(armsUp,15,Y,7);
           lcd.setCursor(15,1);
           lcd.print(" ");
        }

        
       if (!digitalRead(BUTTON))
        if(Button_clik(BUTTON))
        {
            Timer_Supplies= Y;
            EEPROM.write(0,Timer_Supplies);
            return;
        }
      delay(10); 
    }
}

/***********************************************************************************************************/
/***********************************************************************************************************/
uint32_t Conver_Time_MS_to_S(uint32_t Time)
{
  uint8_t Second = 0,Minutes=0,Hourse=0;
  
  Time=Time/1000;
  
  Hourse = Time/3600;
  Minutes = (Time%3600)/60;
  Second = (Time%3600)%60;

  return (Hourse*3600)+(Minutes*60)+Second;
}
byte Button_clik(byte pin)
{
    uint8_t CountClik = 0;
    uint8_t TimeOutClik = 350;

    
    if(Button_bounce(pin))
      if (Button_realising(pin))
       {
        CountClik++;

        while(TimeOutClik--)
        {
          if (!digitalRead(pin))
             if(Button_bounce(pin))
                if (Button_realising(pin))
                {
                  CountClik++;
                  return CountClik;
                }
          
          delay(1);
        }

        return CountClik;
       }
      else
        return 0;
}

bool Button_bounce(byte pin)
{ 
  byte cnt = 100;
  byte count_low = 0;
  
  while(cnt)
  {
    if (!digitalRead(pin))
    {
      if (count_low++>30)
        return 1;
    }
    else
    count_low = 0;
    delay(1);
  }
  return 0;
}

bool Button_realising(byte pin)
{
  int cnt = 2000;
    while(cnt--)
    {
      if (digitalRead(pin))
        return 1;
      delay(1); 
    }
    lcd.clear();
    BlokPrintMainMenu = 1;
    Print_Realise_Button(0,0);
    while(!digitalRead(BUTTON));
    lcd.clear();
    BlokPrintMainMenu = 0;
    
    return 1;
}


/***********************************************************************************************************/
/***********************************************************************************************************/



/***********************************************************************************************************/
/***********************************************************************************************************/
void Settings_Settings()
{ 
    lcd.clear();
    Print_settings(3,0);
    delay(2000);
    Settings_Timer();
    Timer_Supples_();
    if (!Y)
      Kalibrovka();
    else
      Settings_Timer_Motor_On();
    Setting_Speed();
    Hand_Control(); 
}

void Hand_Control()
{ 
    lcd.clear();
    Print_Hand_Control(3,0);
    delay(2000);
    lcd.clear();
    Encoder_cnt = 0xff;
    
    uint8_t Flag_On_Motor = 0;
    uint16_t TimeOut = 500000; 
    
    Print_Hand(1,0);
    
    while(TimeOut--)
    {      
        lcd.setCursor(11,1);
        lcd.print(map(Encoder_cnt,0,255,0,100));
        lcd.print("%");
        lcd.print("   ");

        Blok_Control_Motor(Print_Hand,1,0);
    
       if (!digitalRead(POWER))
        if(Button_clik(POWER))
        {
           
            Flag_On_Motor=~Flag_On_Motor;
        }

        if (Flag_On_Motor)
        {
            Print_On_poliv(10,0);
          
          analogWrite(PWM_Pin,Encoder_cnt);
        }
          
        else
        {
          Print_Off_poliv(10,0);
          analogWrite(PWM_Pin,0);
        }
        
      
       if (!digitalRead(BUTTON))
        if(Button_clik(BUTTON))
        {
            lcd.clear();
            analogWrite(PWM_Pin,0);
            return;
        }
      delay(10); 
    }
}

void Blok_Control_Motor(void (*function)(byte x , byte y),byte x, byte y)
{
      if (Flag_PolivOn)
      {
        lcd.clear();
        lcd.print("IDET POLIV");
          while (Flag_PolivOn);
        lcd.clear();
        function(x,y);
       // Print_kalibrovka(2,0);
      }
}

void Kalibrovka()
{
    uint32_t Time = millis();
    uint32_t Current_Time = 0;
    uint8_t Flag_Count = 0;
    uint16_t TimeOut = 500000; 
        
    lcd.clear();
    Print_kalibrovka(2,0);
    

    while(TimeOut--)
    { 

      if ((millis() - Time) >= 1000) 
      {
        Time+=1000;
        
        if (Flag_Count)
          Current_Time++;
      }
           
      Print_Current_Time(4,1,Current_Time*1000);
      
        Blok_Control_Motor(Print_kalibrovka,2,0);
      
        if (Flag_Count)
        {
          analogWrite(PWM_Pin,PWM_Speed);
          Print_On_poliv(13,0);
        }
        else
        {
            analogWrite(PWM_Pin,0);
            Print_Off_poliv(13,0);
        } 


       
      if (!digitalRead(POWER))
        if(Button_clik(POWER)) 
            Flag_Count=~Flag_Count;
       
      

      if (!digitalRead(BUTTON))
        if(Button_clik(BUTTON))
        {
            TimKalibrate = Current_Time;
            
            TimKalibrateHourse = TimKalibrate/3600;
            TimkalibrateMinutes = (TimKalibrate%3600)/60;
            TimKalibrateSecond = (TimKalibrate%3600)%60;
            
            EEPROM.write(8, TimKalibrateHourse);
            EEPROM.write(9, TimkalibrateMinutes);
            EEPROM.write(10, TimKalibrateSecond);

            return;
        }
      delay(10); 
    }
}

/***********************************************************************************************************/
/***********************************************************************************************************/
void Settings_Timer()
{ 

    lcd.clear();
    Print_Timer(5,0);
    
    TimSecond = EEPROM.read(2);
    TimMinutes = EEPROM.read(3);
    TimHourse = EEPROM.read(4);
      
    Encoder_cnt = TimHourse;

    uint16_t TimeOut = 5000; 
    
    while(TimeOut--)
    {      
      Print_Timer(5,0);
      Print_Current_Timer(5,1,TimHourse,TimMinutes,TimSecond);

      if (Encoder_cnt>99)
        Encoder_cnt = 0;
        
      TimHourse=Encoder_cnt;
      
      if (!digitalRead(BUTTON))
        if(Button_clik(BUTTON))
        {

            EEPROM.write(4, TimHourse);
            Encoder_cnt = TimMinutes;
            TimeOut = 5000;
            
            while(TimeOut--)
            {
                Print_Timer(5,0);
                Print_Current_Timer(5,1,TimHourse,TimMinutes,TimSecond);
                
                if (Encoder_cnt>59)
                  Encoder_cnt = 0;
        
                TimMinutes = Encoder_cnt;

                if (!digitalRead(BUTTON))
                  if(Button_clik(BUTTON))
                  {
           
                      EEPROM.write(3, TimMinutes);
                      Encoder_cnt = TimSecond;
                      TimeOut = 5000;
                    
                      while(TimeOut--)
                      {
                          Print_Timer(5,0);
                          Print_Current_Timer(5,1,TimHourse,TimMinutes,TimSecond);
                
                          if (Encoder_cnt>59)
                            Encoder_cnt = 0;
        
                          TimSecond = Encoder_cnt;

                          if (!digitalRead(BUTTON))
                             if(Button_clik(BUTTON))
                             {
                        
                                EEPROM.write(2, TimSecond);

                                Time_Set = (TimHourse*3600)+(TimMinutes*60)+TimSecond;
                                return;
                             }
                            delay(10); 
                      }
                      return;
                  }
                  delay(10);
            }
            return;
        }
      delay(10); 
    }
    
}
void Settings_Timer_Motor_On()
{ 
    
    lcd.clear();
    Print_Timer_Motor_On(1,0);///Timer_motor
    
    Tim_MotorOn_Hourse = EEPROM.read(5);
    Tim_MotorOn_Minutes = EEPROM.read(6);
    Tim_MotorOn_Second = EEPROM.read(7);
      
    Encoder_cnt = Tim_MotorOn_Hourse;

    uint16_t TimeOut = 5000; 
    
    while(TimeOut--)
    {      
     
      Print_Timer_Motor_On(1,0);
      Print_Current_Timer(5,1,Tim_MotorOn_Hourse,Tim_MotorOn_Minutes,Tim_MotorOn_Second);

      if (Encoder_cnt>99)
        Encoder_cnt = 0;
        
      Tim_MotorOn_Hourse=Encoder_cnt;
      
      if (!digitalRead(BUTTON))
        if(Button_clik(BUTTON))
        {
          
            EEPROM.write(5, Tim_MotorOn_Hourse);
            Encoder_cnt = Tim_MotorOn_Minutes;

            TimeOut = 5000;
            while(TimeOut--)
            {
                
                Print_Timer_Motor_On(1,0);
                Print_Current_Timer(5,1,Tim_MotorOn_Hourse,Tim_MotorOn_Minutes,Tim_MotorOn_Second);
                
                if (Encoder_cnt>59)
                  Encoder_cnt = 0;
        
                Tim_MotorOn_Minutes = Encoder_cnt;

                if (!digitalRead(BUTTON))
                  if(Button_clik(BUTTON))
                  {
                     
                      EEPROM.write(6, Tim_MotorOn_Minutes);
                      Encoder_cnt = Tim_MotorOn_Second;
                      
                      TimeOut = 5000;
                      while(TimeOut--)
                      {       
                          Print_Timer_Motor_On(1,0);
                          Print_Current_Timer(5,1,Tim_MotorOn_Hourse,Tim_MotorOn_Minutes,Tim_MotorOn_Second);
                
                          if (Encoder_cnt>59)
                            Encoder_cnt = 0;
        
                          Tim_MotorOn_Second = Encoder_cnt;

                          if (!digitalRead(BUTTON))
                             if(Button_clik(BUTTON))
                             {
                                
                                EEPROM.write(7, Tim_MotorOn_Second);

                                Time_Motor_On_Set = (Tim_MotorOn_Hourse*3600)+(Tim_MotorOn_Minutes*60)+Tim_MotorOn_Second;
                                return;
                             }
                            delay(10); 
                      }
                      return;
                  }
                  delay(10);
            }
            return;
        }
      delay(10); 
    }
    
}
void Setting_Speed()
{
    lcd.clear();  
    Print_Speed_Motor(0,0);
    Encoder_cnt = EEPROM.read(1);
          

    uint16_t TimeOut = 5000; 
    
    while(TimeOut--)
    {   
        lcd.setCursor(6,1);
        lcd.print(map(PWM_Speed,0,255,0,100));
        lcd.print("%");
        lcd.print("   ");
        PWM_Speed=Encoder_cnt;

        
        //analogWrite(PWM_Pin,PWM_Speed);
                  
        if (!digitalRead(BUTTON))
          if(Button_clik(BUTTON))
          {
           
              EEPROM.write(1,PWM_Speed); 
              return;   
          }
        delay(10);      
    }
}
/***********************************************************************************************************/
/***********************************************************************************************************/
void Cycle_Time(uint32_t Time_Ref)
{
  
  if (Time_Current>=Time_Set)
  {
    
    Motor_On(Time_Ref);
    timer = millis();
    timer_cur=0;
    Time_Current=0;
  
  }
}

void Motor_On(uint32_t TimSet)
{
    uint32_t Time_Motor_On=0;
    uint32_t Time=0;
    uint8_t Hourse = 0,Minutes = 0,Second = 0;


    Flag_PolivOn = 1;
    Time = millis();
    
    while(Time_Motor_On<TimSet)
    {
      analogWrite(PWM_Pin,PWM_Speed);
      if (!BlokPrintMainMenu)
          Print_On_poliv(10,0);
      if ((millis() - Time) >= 1000) 
      {
        Time+=1000;
        
        Time_Motor_On++;

        Hourse = Time_Motor_On/3600;
        Minutes = (Time_Motor_On%3600)/60;
        Second = (Time_Motor_On%3600)%60;

        if (!BlokPrintMainMenu)
          Print_Current_Timer(8,1,Hourse,Minutes,Second);
      }
    }
    analogWrite(PWM_Pin,0);
    if (!BlokPrintMainMenu)
      Print_Off_poliv(10,0); 
    Flag_PolivOn = 0;
}
