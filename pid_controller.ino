#include <Keypad.h>
#include "max6675.h"
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PID_v1.h>

//----------------------------------------------------------------------------------
//keypad related:
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {2, 3, 4, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {9, 10, 11}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//thermocouple related:
int thermoDO = 7;
int thermoCS = 5;
int thermoCLK = 8;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);

//----------------------------------------------------------------------------------
//PIC controller related:
//----------------------------------------------------------------------------------

//Relay control pin:
#define RELAY_PIN 12

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, DIRECT);

//PID controller configuration addreses in EEPROM:
#define EEPROM_ADDR_PID_KP_0        0
#define EEPROM_ADDR_PID_KP_1        1
#define EEPROM_ADDR_PID_KP_2        2
#define EEPROM_ADDR_PID_KP_3        3
#define EEPROM_ADDR_PID_KI_0        4
#define EEPROM_ADDR_PID_KI_1        5
#define EEPROM_ADDR_PID_KI_2        6
#define EEPROM_ADDR_PID_KI_3        7
#define EEPROM_ADDR_PID_KD_0        8
#define EEPROM_ADDR_PID_KD_1        9
#define EEPROM_ADDR_PID_KD_2        10
#define EEPROM_ADDR_PID_KD_3        11
#define EEPROM_ADDR_PID_WINDOW_0    12
#define EEPROM_ADDR_PID_WINDOW_1    13

//PID configuration structure:
typedef struct
{
  double Kp;
  double Ki;
  double Kd;
  int window_size_s;
} PID_coef_struct;
PID_coef_struct PID_coef;

//PID initialize configuration:
void PID_init()
{
  byte byte_arr[4];
  //read last P coefficient from EEPROM:
  byte_arr[0] = EEPROM.read(EEPROM_ADDR_PID_KP_0);
  byte_arr[1] = EEPROM.read(EEPROM_ADDR_PID_KP_1);
  byte_arr[2] = EEPROM.read(EEPROM_ADDR_PID_KP_2);
  byte_arr[3] = EEPROM.read(EEPROM_ADDR_PID_KP_3);
  memcpy(&PID_coef.Kp, byte_arr, sizeof(double));

  //read last I coefficient from EEPROM:
  byte_arr[0] = EEPROM.read(EEPROM_ADDR_PID_KI_0);
  byte_arr[1] = EEPROM.read(EEPROM_ADDR_PID_KI_1);
  byte_arr[2] = EEPROM.read(EEPROM_ADDR_PID_KI_2);
  byte_arr[3] = EEPROM.read(EEPROM_ADDR_PID_KI_3);
  memcpy(&PID_coef.Ki, byte_arr, sizeof(double));

  //read last D coefficient from EEPROM:
  byte_arr[0] = EEPROM.read(EEPROM_ADDR_PID_KD_0);
  byte_arr[1] = EEPROM.read(EEPROM_ADDR_PID_KD_1);
  byte_arr[2] = EEPROM.read(EEPROM_ADDR_PID_KD_2);
  byte_arr[3] = EEPROM.read(EEPROM_ADDR_PID_KD_3);
  memcpy(&PID_coef.Kd, byte_arr, sizeof(double));

  //read control window duration:
  PID_coef.window_size_s = (EEPROM.read(EEPROM_ADDR_PID_WINDOW_1) << 8) + EEPROM.read(EEPROM_ADDR_PID_WINDOW_0);

  Serial.println("PID initialization:");
  Serial.println(int(PID_coef.Kp));
  Serial.println(int(PID_coef.Ki));
  Serial.println(int(PID_coef.Kd));
  Serial.println(int(PID_coef.window_size_s));
}

//Save new P coef:
void PID_wr_Kp()
{
  byte byte_arr[4];
  memcpy(byte_arr, &PID_coef.Kp, sizeof(double));
  EEPROM.write(EEPROM_ADDR_PID_KP_0, byte_arr[0]);
  EEPROM.write(EEPROM_ADDR_PID_KP_1, byte_arr[1]);
  EEPROM.write(EEPROM_ADDR_PID_KP_2, byte_arr[2]);
  EEPROM.write(EEPROM_ADDR_PID_KP_3, byte_arr[3]);
}

//Save new I coef:
void PID_wr_Ki()
{
  byte byte_arr[4];
  memcpy(byte_arr, &PID_coef.Ki, sizeof(double));
  EEPROM.write(EEPROM_ADDR_PID_KI_0, byte_arr[0]);
  EEPROM.write(EEPROM_ADDR_PID_KI_1, byte_arr[1]);
  EEPROM.write(EEPROM_ADDR_PID_KI_2, byte_arr[2]);
  EEPROM.write(EEPROM_ADDR_PID_KI_3, byte_arr[3]);
}

//Save new D coef:
void PID_wr_Kd()
{
  byte byte_arr[4];
  memcpy(byte_arr, &PID_coef.Kd, sizeof(double));
  EEPROM.write(EEPROM_ADDR_PID_KD_0, byte_arr[0]);
  EEPROM.write(EEPROM_ADDR_PID_KD_1, byte_arr[1]);
  EEPROM.write(EEPROM_ADDR_PID_KD_2, byte_arr[2]);
  EEPROM.write(EEPROM_ADDR_PID_KD_3, byte_arr[3]);
}

//Save new window size:
void PID_wr_window_size()
{
  EEPROM.write(EEPROM_ADDR_PID_WINDOW_0, (PID_coef.window_size_s & 0xFF));
  EEPROM.write(EEPROM_ADDR_PID_WINDOW_1, ((PID_coef.window_size_s & 0xFF00) >> 8));
}

//Temperature profile:
#define TEMP_PROFILE_MAX_STAGE    5 //max number of stages in temperature profile

typedef struct 
{
  int temp_oC;
  unsigned long duration_min;
} temp_stage;

//temperature profile structure:
typedef struct
{
  temp_stage temp_stage_arr[TEMP_PROFILE_MAX_STAGE];
  int stage_num;
} temp_profile_struct;

temp_profile_struct temp_profile = {{0}}; 

//write stage parameters to temperature profile:
void Temp_profile_write(int temp, unsigned long duration)
{
  temp_profile.temp_stage_arr[temp_profile.stage_num].temp_oC = temp;
  temp_profile.temp_stage_arr[temp_profile.stage_num].duration_min = duration;
  temp_profile.stage_num++;
}

//----------------------------------------------------------------------------------
//LCD display related:
//----------------------------------------------------------------------------------

//Idle state display:
void LCD_idle()
{
  //IDLE STATE
  lcd.home();
  lcd.clear();
  lcd.print("IDLE STA");
  lcd.setCursor(0,1);
  lcd.print("TE");
}

//Set coef display:
void LCD_set_coef(char coef, int value)
{
  lcd.home();
  lcd.clear();
  lcd.print("SET ");
  lcd.print(coef);
  lcd.print(":");
  lcd.setCursor(0,1);
  lcd.print(value, DEC);
}

//Set window size display:
void LCD_set_window(int window_size)
{
  lcd.home();
  lcd.clear();
  lcd.print("WINDOW:");
  lcd.setCursor(0,1);
  lcd.print(window_size, DEC);
}

//Set number of stages in profile:
void LCD_set_number_of_stages(int num)
{
  lcd.home();
  lcd.clear();
  lcd.print("STAGE NU");
  lcd.setCursor(0,1);
  lcd.print("M: ");
  lcd.print(num, DEC);
}

//Set stage temperature value display:
void LCD_set_stage_temp(int stage_num, int temp)
{
  lcd.home();
  lcd.clear();
  lcd.print("TEMP");
  lcd.print(stage_num, DEC);
  lcd.print(":");
  lcd.setCursor(0,1);
  lcd.print(temp, DEC);
}

//Set stage duration value display:
void LCD_set_stage_duration(int stage_num, unsigned long duration)
{
  lcd.home();
  lcd.clear();
  lcd.print("TIME");
  lcd.print(stage_num, DEC);
  lcd.print(":");
  lcd.setCursor(0,1);
  lcd.print(duration, DEC);
}

//Active status display:
void LCD_active_status()
{
  //ACTIVE STATE
  lcd.home();
  lcd.clear();
  lcd.print("ACTIVE S");
  lcd.setCursor(0,1);
  lcd.print("TATE");
}

//Current temperature display:
void LCD_active_cur_temp(int temp) //temperature 3 digits max
{
  lcd.home();
  lcd.clear();
  lcd.print("CURRENT ");
  lcd.setCursor(0,1);
  lcd.print("TEMP:");
  lcd.print(temp, DEC);
}

//Setpoint temperature:
void LCD_active_setpoint_temp(int temp) //temperature 3 digits max
{
  lcd.home();
  lcd.clear();
  lcd.print("SET TEMP");
  lcd.setCursor(0,1);
  lcd.print(":");
  lcd.print(temp, DEC);
}

//Temperature profile stage status:
void LCD_active_profile_stage_status(int cur_stage, int total_stage)
{
  lcd.home();
  lcd.clear();
  lcd.print("STAGES: ");
  lcd.setCursor(0,1);
  lcd.print(cur_stage, DEC);
  lcd.print("/");
  lcd.print(total_stage, DEC);
}

//Time left:
void LCD_active_time_left(int time_left)
{
  lcd.home();
  lcd.clear();
  lcd.print("TIME LEF");
  lcd.setCursor(0,1);
  lcd.print("T: ");
  lcd.print(time_left, DEC);
  lcd.print(" M");
}

//----------------------------------------------------------------------------------
//PID controller state machine related:
//----------------------------------------------------------------------------------

//PID controller state machine's states:
enum
{
  PID_IDLE=0,
  PID_ACTIVE,
  PID_FAULT
};

//PID controller sample period:
#define PID_SAMPLE_PERIOD_MS  1000

//PID state machine's current state:
int pid_current_state = PID_IDLE;

//PID state machine's set state from UI:
int pid_set_state = PID_IDLE;

//PID controller's state machine:
void PID_state_machine()
{
  static unsigned char i = 0;
  static unsigned long windowStartTime = 0;
  static unsigned long stage_end_time = 0;
  static unsigned long pid_sample_dly = 0;
  
  switch (pid_current_state)
  {
    case PID_IDLE: //PID controller is IDLE
      switch (pid_set_state)
      {
      case PID_ACTIVE:
        pid_current_state = PID_ACTIVE;
        //set PID coefficients:
        myPID.SetTunings(PID_coef.Kp,PID_coef.Ki,PID_coef.Kd);
        //setup set point temperature for first profile:
        i = 0;
        Setpoint = temp_profile.temp_stage_arr[i].temp_oC;
        //initilize windows start time
        windowStartTime = millis();
        //set stage end time:
        stage_end_time = millis() + temp_profile.temp_stage_arr[i].duration_min*60*1000;
        //set output limits:
        myPID.SetOutputLimits(0, PID_coef.window_size_s);//also convert from seconds to miliseconds
        //turn the PID on
        myPID.SetMode(AUTOMATIC);
        break;
      default:
        pid_current_state = PID_IDLE; //stay the same state
        break;
      }
      break;
    case PID_ACTIVE:  //PID controller is executing temperature profile
      switch (pid_set_state)
      {
      case PID_IDLE:
        i=0;
        pid_current_state = PID_IDLE;
        pid_set_state = PID_IDLE;
        digitalWrite(RELAY_PIN, HIGH);
        myPID.SetMode(MANUAL);
        break;
      default:
        if ((millis() - pid_sample_dly) > PID_SAMPLE_PERIOD_MS)
        {
          pid_current_state = PID_ACTIVE; //stay the same state
          //execute PID computation and control:
          Input = thermocouple.readCelsius();
          Serial.print("C = ");
          Serial.println(Input);
          myPID.Compute();
          if (millis() - windowStartTime > PID_coef.window_size_s * 1000)
          { //time to shift the Relay Window
            windowStartTime += PID_coef.window_size_s * 1000;
          }
          if (1000*Output > millis() - windowStartTime) digitalWrite(RELAY_PIN, LOW);
          else digitalWrite(RELAY_PIN, HIGH);
  
          Serial.print("Output: ");
          Serial.println(Output);
          
          //check stage duration:
          if (millis() >= stage_end_time)
          {
            //check if this is last stage:
            if (i >= (temp_profile.stage_num - 1))
            {
              Serial.print("Finished!");
              i=0;
              pid_current_state = PID_IDLE;
              pid_set_state = PID_IDLE;
              digitalWrite(RELAY_PIN, HIGH);
              myPID.SetMode(MANUAL);
            }
            else
            {
              //change to new stage:
              Serial.print("Changing state");
              i++;
              Setpoint = temp_profile.temp_stage_arr[i].temp_oC;
              //initilize windows start time
              windowStartTime = millis();
              //set stage end time:
              stage_end_time = millis() + temp_profile.temp_stage_arr[i].duration_min*60*1000;
            }
          }
          pid_sample_dly = millis(); //start sample period again
        }
        break;
      }
      break;
    case PID_FAULT: //PID controller is in fault state
      break;
  }
}

//----------------------------------------------------------------------------------
//User interface related:
//----------------------------------------------------------------------------------

#define UI_START_BUTTON_PRESS_DLY_MS    3000

//User interface state machine's states:
enum
{
    IDLE_STATUS=0,
    IDLE_START_DLY,
    IDLE_SET_P,
    IDLE_SET_I,
    IDLE_SET_D,
    IDLE_SET_WINDOW,
    IDLE_SET_NUMBER_OF_STAGES,
    IDLE_SET_STAGE_TEMP,
    IDLE_SET_STAGE_DURATION,
    ACTIVE_STATUS,
    ACTIVE_CUR_TEMP,
    ACTIVE_SETPOINT_TEMP,
    ACTIVE_STAGE_STATUS,
    ACTIVE_TIME_LEFT
};

int ui_current_state = IDLE_STATUS;

//UI state machine:
void UI_state_machine()
{  
  char key = 0;
  static unsigned long time_dly = 0;

  static int cur_stage_num = 0;

  //Serial.println("Executing UI");
  
  switch (pid_current_state)
  {
    case PID_IDLE: //PID controller is in IDLE state
      switch (ui_current_state)
      {
      case IDLE_STATUS:
        LCD_idle();
        key = keypad.getKey();
        switch (key)
        {
        case '*': //go to press button delay to start temperature profile
          ui_current_state = IDLE_START_DLY;
          time_dly = millis();
          break;
        case '#': //go to setup menu
          ui_current_state = IDLE_SET_P;
          break;
        }
        break;
      case IDLE_START_DLY:
        LCD_idle();
        keypad.getKeys();
        if (keypad.findInList('*') != -1)
        {
          if ((millis() - time_dly) > UI_START_BUTTON_PRESS_DLY_MS)
          {
            ui_current_state = ACTIVE_STATUS;
            pid_set_state = PID_ACTIVE;
          }
        }
        else
        {
          ui_current_state = IDLE_STATUS;
        }
        break;
      case IDLE_SET_P:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          ui_current_state = IDLE_SET_I;
          PID_wr_Kp();
          break;
        case '*':
          PID_coef.Kp = 0;
          break;
        case 0:
          break;
        default:
          if ((PID_coef.Kp > 0) && (PID_coef.Kp < 10))
          {
            PID_coef.Kp = 10 * PID_coef.Kp + (key - 48);
          }
          else if (PID_coef.Kp == 0)
          {
            PID_coef.Kp = (key - 48);
          }
          else
          {
            //need to indicate, that coefficient is already two digits
          }
          break;
        }
        LCD_set_coef('P',PID_coef.Kp);
        break;
      case IDLE_SET_I:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          ui_current_state = IDLE_SET_D;
          PID_wr_Ki();
          break;
        case '*':
          PID_coef.Ki = 0;
          break;
        case 0:
          break;
        default:
          if ((PID_coef.Ki > 0) && (PID_coef.Ki < 10))
          {
            PID_coef.Ki = 10 * PID_coef.Ki + (key - 48);
          }
          else if (PID_coef.Ki == 0)
          {
            PID_coef.Ki = (key - 48);
          }
          else
          {
            //need to indicate, that coefficient is already two digits
          }
          break;
        }
        LCD_set_coef('I',PID_coef.Ki);
        break;
      case IDLE_SET_D:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          ui_current_state = IDLE_SET_WINDOW;
          PID_wr_Kd();
          break;
        case '*':
          PID_coef.Kd = 0;
          break;
        case 0:
          break;
        default:
          if ((PID_coef.Kd > 0) && (PID_coef.Kd < 10))
          {
            PID_coef.Kd = 10 * PID_coef.Kd + (key - 48);
          }
          else if (PID_coef.Kd == 0)
          {
            PID_coef.Kd = (key - 48);
          }
          else
          {
            //need to indicate, that coefficient is already two digits
          }
          break;
        }
        LCD_set_coef('D',PID_coef.Kd);
        break;
      case IDLE_SET_WINDOW:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          ui_current_state = IDLE_SET_NUMBER_OF_STAGES;
          PID_wr_window_size();
          break;
        case '*':
          PID_coef.window_size_s = 0;
          break;
        case 0:
          break;
        default:
          if ((PID_coef.window_size_s > 0) && (PID_coef.window_size_s < 10))
          {
              PID_coef.window_size_s = 10 * PID_coef.window_size_s + (key - 48);
          }
          else if (PID_coef.window_size_s == 0)
          {
            PID_coef.window_size_s = (key - 48);
          }
          else
          {
            //need to indicate, that coefficient is already two digits
          }
          break;
        }
        LCD_set_window(PID_coef.window_size_s);
        break;
      case IDLE_SET_NUMBER_OF_STAGES:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          if (temp_profile.stage_num)
          {
          ui_current_state = IDLE_SET_STAGE_TEMP;
          cur_stage_num = 0; // set current number of stages to zero
          }
          else
          {
            ui_current_state = IDLE_STATUS;
          }
          break;
        case '*':
          temp_profile.stage_num = 0;
          break;
        case 0:
          break;
        default:
          if (temp_profile.stage_num == 0)
          {
            if ((key - 48) <=  TEMP_PROFILE_MAX_STAGE)
            {
              temp_profile.stage_num = (key - 48);
            }
            else
            {
              //need to indicate that to much stages 
            }
          }
          else
          {
            //first clean number
          }
          break;
        }
        LCD_set_number_of_stages(temp_profile.stage_num);
        break;
      case IDLE_SET_STAGE_TEMP:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          ui_current_state = IDLE_SET_STAGE_DURATION;
          break;
        case '*':
          temp_profile.temp_stage_arr[cur_stage_num].temp_oC = 0;
          break;
        case 0:
          break;
        default:
          if ((temp_profile.temp_stage_arr[cur_stage_num].temp_oC > 0) && (temp_profile.temp_stage_arr[cur_stage_num].temp_oC < 10))
          {
              temp_profile.temp_stage_arr[cur_stage_num].temp_oC = 10 * temp_profile.temp_stage_arr[cur_stage_num].temp_oC + (key - 48);
          }
          else if (temp_profile.temp_stage_arr[cur_stage_num].temp_oC == 0)
          {
            temp_profile.temp_stage_arr[cur_stage_num].temp_oC = (key - 48);
          }
          else
          {
            //need to indicate, that coefficient is already two digits
          }
          break;
        }
        LCD_set_stage_temp(cur_stage_num + 1, temp_profile.temp_stage_arr[cur_stage_num].temp_oC);
        break;
      case IDLE_SET_STAGE_DURATION:
        key = keypad.getKey();
        switch(key)
        {
        case '#':
          if (cur_stage_num < (temp_profile.stage_num - 1))
          {
            ui_current_state = IDLE_SET_STAGE_TEMP;
            cur_stage_num++;
          }
          else
          {
            ui_current_state = IDLE_STATUS;
          }
          break;
        case '*':
          temp_profile.temp_stage_arr[cur_stage_num].duration_min = 0;
          break;
        case 0:
          break;
        default:
          if ((temp_profile.temp_stage_arr[cur_stage_num].duration_min > 9) && (temp_profile.temp_stage_arr[cur_stage_num].duration_min < 100))
          {
            temp_profile.temp_stage_arr[cur_stage_num].duration_min = 10 * temp_profile.temp_stage_arr[cur_stage_num].duration_min + (key - 48);
          }
          else if ((temp_profile.temp_stage_arr[cur_stage_num].duration_min > 0) && (temp_profile.temp_stage_arr[cur_stage_num].duration_min < 9))
          {
            temp_profile.temp_stage_arr[cur_stage_num].duration_min = 10 * temp_profile.temp_stage_arr[cur_stage_num].duration_min + (key - 48);
          }
          else if (temp_profile.temp_stage_arr[cur_stage_num].duration_min == 0)
          {
            temp_profile.temp_stage_arr[cur_stage_num].duration_min = (key - 48);
          }
          else
          {
            //need to indicate, that value is already three digits
          }
          break;
        }
        LCD_set_stage_duration(cur_stage_num + 1, temp_profile.temp_stage_arr[cur_stage_num].duration_min);
        break;
      default:
        ui_current_state = IDLE_STATUS;
        break;
      }
      break;
    case PID_ACTIVE: //PID controller is in active state
      switch (ui_current_state)
      {
      case ACTIVE_STATUS:
        LCD_active_status();
        if (keypad.getKey() == '*')
        {
          ui_current_state = IDLE_STATUS;
          pid_set_state = PID_IDLE;
        }
        break;
      case ACTIVE_CUR_TEMP:
      break;
      case ACTIVE_SETPOINT_TEMP:
        break;
      case ACTIVE_STAGE_STATUS:
        break;
      case ACTIVE_TIME_LEFT:
        break;
      default:
        ui_current_state = ACTIVE_STATUS;
        break;
      }
      break;
    case PID_FAULT:
      break;
  }
}

//----------------------------------------------------------------------------------
//Setup function:
//----------------------------------------------------------------------------------

void setup() {
  //serial port setup:
  Serial.begin(9600);

  //Configure relay pin:
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); //relay is active low
  
  //LCD setup:
  lcd.begin(8, 2);

  //PID controller setup:
  PID_init();
}

//----------------------------------------------------------------------------------
//Main loop:
void loop() {
  UI_state_machine();
  PID_state_machine();
}
//----------------------------------------------------------------------------------
