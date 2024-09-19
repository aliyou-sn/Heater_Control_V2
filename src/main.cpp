#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"
#include <time.h>
#include "MAX6675.h"



// Actual Temperature for Grill and warmer
//In celcius
int16_t GrillTemperatureC;
int16_t WarmerTemperatureC;
//In farenheit
int16_t GrillTemperatureF;
int16_t WarmerTemperatureF;

// Create an instance of the HSPI bus
SPIClass hspi(HSPI);

int thermoDO = 12;
int thermoDI = 13;
int thermoCLK = 14;

// int thermoDO = 19;
// int thermoCLK = 18;

int thermoCS_T1 = 26;
int thermoCS_T2 = 25;
uint32_t start, stop;

int triac1Pin = 17;
int triac2Pin = 22;

int zeroCrossPin = 5; // Zero-crossing detection pin
volatile bool zeroCrossDetected = false;
volatile unsigned long lastZeroCrossTime = 0;



// PID variables for Grill
float kp1 = 203.0, ki1 = 7.2, kd1 = 1.04;
float PID_p1, PID_i1, PID_d1, PID_value1;
float previous_error1 = 0;
float integral1 = 0;
unsigned long lastTime1 = 0;

// PID variables for Warmer
float kp2 = 203.0, ki2 = 7.2, kd2 = 1.04;
float PID_p2, PID_i2, PID_d2, PID_value2;
float previous_error2 = 0;
float integral2 = 0;
unsigned long lastTime2 = 0;

// MAX6675 thermocouple_1(thermoCS_T1, thermoDO, thermoCLK);
// MAX6675 thermocouple_2(thermoCS_T2, thermoDO, thermoCLK);
MAX6675 thermocouple_1(thermoCS_T1,  &hspi);
MAX6675 thermocouple_2(thermoCS_T2,  &hspi);

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

// TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
TFT_eSPI tft = TFT_eSPI();

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX, touchY;

    bool touched = tft.getTouch( &touchX, &touchY, 600 );

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
    }
}


// Task to update system time on the display

void displayTimeTask(void *Parameters) {    
    for (;;) 
    {

        // Get current system time
        time_t now = time(NULL);
        struct tm *local = localtime(&now);
        
        // Format time as "MM:SS"
        char minutesStr[3];
        char secondsStr[3];
        lv_snprintf(minutesStr, sizeof(minutesStr), "%02d", local->tm_hour);
        lv_snprintf(secondsStr, sizeof(secondsStr), "%02d", local->tm_min);


        // Update label text
        lv_label_set_text(ui_MinutesLabel, minutesStr);
        lv_label_set_text(ui_SecondsLabel, secondsStr);        
    
        // Delay for 30 second
        vTaskDelay(30000 / portTICK_PERIOD_MS);

    }
}



void adjustTriac(int triacPin, float PID_value) {
    int firingDelay = map(PID_value, 0, 10, 0, 7400); // Map PID to firing delay
    delayMicroseconds(firingDelay); 
    digitalWrite(triacPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triacPin, LOW);
}


//Function to get current temperature of grill and warmer every 1 sec
void GetTemp(void *parameter){
  for(;;)
  {

  int status1 = thermocouple_1.read();
  int status2 = thermocouple_2.read();


  GrillTemperatureC = thermocouple_1.getTemperature();
  WarmerTemperatureC = thermocouple_2.getTemperature();
  //for Fahrenheit conversion
  GrillTemperatureF = (0.5556*GrillTemperatureC) + 32;
  WarmerTemperatureF = (0.5556*WarmerTemperatureC) + 32;


  unsigned long currentTime = millis();
  float elapsedTime1 = (currentTime - lastTime1) / 1000.0; // For Grill
  float elapsedTime2 = (currentTime - lastTime2) / 1000.0; // For Warmer


  char buf1[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  char buf2[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  if(Fahrenheit)
  {

    if(digitalRead(Grill) == HIGH && GrillTemperatureF < GrillSetTemperatureF - 5)
    {

      float error1 = GrillSetTemperatureF - GrillTemperatureF; // Error for Grill
      integral1 += error1 * elapsedTime1; // Integral term for Grill
      float derivative1 = (error1 - previous_error1) / elapsedTime1; // Derivative term for Grill

      // PID output for Grill
      PID_p1 = kp1 * error1;
      PID_i1 = ki1 * integral1;
      PID_d1 = kd1 * derivative1;
      PID_value1 = PID_p1 + PID_i1 + PID_d1;

      previous_error1 = error1;
      lastTime1 = currentTime;


      if (zeroCrossDetected) 
      {
          adjustTriac(triac1Pin, PID_value1);
      }
    }
 


    if(digitalRead(Warmer) == HIGH && WarmerTemperatureF < WarmerSetTemperatureF - 5)
    {


      float error2 = WarmerSetTemperatureF - WarmerTemperatureF; // Error for Warmer
      integral2 += error2 * elapsedTime2; // Integral term for Warmer
      float derivative2 = (error2 - previous_error2) / elapsedTime2; // Derivative term for Warmer

      // PID output for Warmer
      PID_p2 = kp2 * error2;
      PID_i2 = ki2 * integral2;
      PID_d2 = kd2 * derivative2;
      PID_value2 = PID_p2 + PID_i2 + PID_d2;

      previous_error2 = error2;
      lastTime2 = currentTime;

      if (zeroCrossDetected) 
      {
        adjustTriac(triac2Pin, PID_value2);
      }
    }

    lv_snprintf(buf1, sizeof(buf1), "%d", GrillTemperatureF);
    lv_label_set_text(ui_GCurrentTempLabel, buf1);
    lv_snprintf(buf2, sizeof(buf2), "%d", WarmerTemperatureF);
    lv_label_set_text(ui_WCurrentTempLabel, buf2);

     Serial.println("In Fahrenheit");
  }
  else
  {


    if(digitalRead(Grill) == HIGH && GrillTemperatureC < GrillSetTemperatureC - 5)
    {

      float error1 = GrillSetTemperatureC - GrillTemperatureC; // Error for Grill
      integral1 += error1 * elapsedTime1; // Integral term for Grill
      float derivative1 = (error1 - previous_error1) / elapsedTime1; // Derivative term for Grill

      // PID output for Grill
      PID_p1 = kp1 * error1;
      PID_i1 = ki1 * integral1;
      PID_d1 = kd1 * derivative1;
      PID_value1 = PID_p1 + PID_i1 + PID_d1;

      previous_error1 = error1;
      lastTime1 = currentTime;
      if (zeroCrossDetected) 
      {
        adjustTriac(triac1Pin, PID_value1);
      }
    }
   

    if(digitalRead(Warmer) == HIGH && WarmerTemperatureC < WarmerSetTemperatureC - 5)
    {


      float error2 = WarmerSetTemperatureC - WarmerTemperatureC; // Error for Warmer
      integral2 += error2 * elapsedTime2; // Integral term for Warmer
      float derivative2 = (error2 - previous_error2) / elapsedTime2; // Derivative term for Warmer

      // PID output for Warmer
      PID_p2 = kp2 * error2;
      PID_i2 = ki2 * integral2;
      PID_d2 = kd2 * derivative2;
      PID_value2 = PID_p2 + PID_i2 + PID_d2;

      previous_error2 = error2;
      lastTime2 = currentTime;


      if (zeroCrossDetected) 
      {
        adjustTriac(triac2Pin, PID_value2);
      }
    }


    lv_snprintf(buf1, sizeof(buf1), "%d", GrillTemperatureC);
    lv_label_set_text(ui_GCurrentTempLabel, buf1);
    lv_snprintf(buf2, sizeof(buf2), "%d", WarmerTemperatureC);
    lv_label_set_text(ui_WCurrentTempLabel, buf2);

    Serial.println("In Celcius");
   
    }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void IRAM_ATTR zeroCrossISR() {
  zeroCrossDetected = true;
  lastZeroCrossTime = micros();
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */


    lv_init();


    // Set up PWM for backlight on pin 32
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(TFT_BL_PIN, PWM_CHANNEL);

    // Set initial brightness (80%)
    ledcWrite(PWM_CHANNEL, 205);

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( 1 ); /* Landscape orientation, flipped */

    // Calibration Values for my Setup
    uint16_t calData[5] = { 247, 3669, 293, 3407, 7 };
    tft.setTouch(calData);

    // lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    pinMode(Grill, OUTPUT);
    pinMode(Warmer, OUTPUT);
    pinMode(zeroCrossPin, INPUT_PULLUP);
    pinMode(triac1Pin, OUTPUT); 
    pinMode(triac2Pin, OUTPUT); 

    // Attach zero-crossing interrupt
    attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCrossISR, FALLING);


    //Current temperature task
    xTaskCreate(GetTemp,"Update temperature", 5000, NULL, 2, NULL);

    //Current temperature task
    xTaskCreate(displayTimeTask,"display Time", 5000, NULL, 2, NULL);

    Serial.println( "Setup done" );

    hspi.begin(thermoCLK, thermoDO, thermoDI, thermoCS_T1);

    thermocouple_1.begin();
    delay(200);
    thermocouple_2.begin();
    delay(200);


    digitalWrite(Grill, LOW);
    digitalWrite(Warmer, LOW);
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    // delay(5);
}
