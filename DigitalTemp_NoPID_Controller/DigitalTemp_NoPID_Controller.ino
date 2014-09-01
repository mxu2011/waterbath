#include <SPI.h>
#include <util.h>
#include <Timer.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#define ONE_WIRE_BUS 10
#define RELAY_PIN 5
#define RELAY_PIN_GROUND 4
#define COOLING_PIN A4

//in hours
float segment_times[] = {0.3, 0.3, 0.3};
float segment_temps[] = {30, 42, 37};
float setpoint = segment_temps[0];
int segment_idx = 0;
int num_segments = 3;

enum operating_state {OFF = 0, HEAT, COOL};
enum power_level {P1 = 0, P2, P3, P4, P5};

operating_state op_state = OFF;
power_level p_level = P1;

float last_temp, current_temp;

unsigned long windowStartTime;

volatile long onTime;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor;

bool last_reading_error = false;

Timer t;

int window_size = 5000;
unsigned long window_start_time;


void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY_PIN_GROUND, OUTPUT);
  pinMode(COOLING_PIN, OUTPUT);
  
  digitalWrite(COOLING_PIN, LOW);
  digitalWrite(RELAY_PIN_GROUND, LOW);
  
  t.every(3000, write_output, (void*)0);

  Serial.begin(9600);
  
  //wait for serial
  while(!Serial)
  {
    ;
  }
  
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0)) 
  {
    Serial.println("Sensor Error");
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  
  while(!sensors.isConversionAvailable(0))
  {
    delay(10);
    
  }
  current_temp = sensors.getTempC(tempSensor);
  last_temp = current_temp;
  Serial.println(current_temp);
  
  //start the process of aquiring a temperature reading
  sensors.requestTemperatures();
  
  update_op_state((void*)0);
  
  t.every(5000, update_op_state, (void*)0);
  t.after(segment_times[segment_idx] * 60 * 60 * 1000, update_segment, (void *)0);

  window_start_time = millis();
}

void loop()
{  
  t.update();
  unsigned long now = millis();
  int window_time_elapsed =  (now - window_start_time);

  if(now - window_start_time > window_size)
    window_start_time += window_size;
 
  
  if(sensors.isConversionAvailable(0))
  {
    last_temp = current_temp;
    current_temp = sensors.getTempC(tempSensor);
    
    //if we got an erroneous reading, use last good reading 
    if(current_temp <= 0)
    {
      last_reading_error = true;
      current_temp = last_temp;
    } 
    else
      last_reading_error = false;
      
    sensors.requestTemperatures();    
  }    
  
  if(op_state == COOL)
  {
    digitalWrite(RELAY_PIN, LOW);
    switch(p_level)
    {
      case P1:
        analogWrite(COOLING_PIN, 0.4 * 255);
        break;
      case P2:
        analogWrite(COOLING_PIN, 0.55 * 255);
        break;
      case P3:
        analogWrite(COOLING_PIN, 0.65 * 255);
        break;
      case P4:
        analogWrite(COOLING_PIN, 0.9 * 255);
        break;
      case P5:
        analogWrite(COOLING_PIN, 1.0 * 255);
        break;
    }
  }
  else if(op_state == HEAT)
  {
    int output = 0;
    //This is run 1 second after P1, 1.5 seconds after P2...
    switch(p_level)
    {
      case P1:  
        output = 1000;
        break;
      case P2:
        output = 2000;
        break;
      case P3:
        output = 3000;
        break;
      case P4:
        output = 4000;
        break;
      case P5:
        output = 5000;
        break;
    }
    
    if(output > window_time_elapsed || p_level == P5)
    {
      digitalWrite(RELAY_PIN, HIGH);
      analogWrite(COOLING_PIN, LOW);
    }
    else
    {
      digitalWrite(RELAY_PIN, LOW);
      analogWrite(COOLING_PIN, LOW);
    }
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
    analogWrite(COOLING_PIN, LOW);
  }
}

void update_op_state(void *context)
{
  if (setpoint == -1) 
  {
    op_state = OFF;
    return;
  }
  float temp_difference = setpoint - current_temp;
  float abs_temp_difference = abs(temp_difference);

  if (abs_temp_difference > 0.20)
  {
    if(temp_difference > 0)
      op_state = HEAT;
    else
      op_state = COOL;
    
  }
  else
  {
    op_state = OFF;
  }
  
  if(abs_temp_difference > 2.5)
    p_level = P5;
  else if (abs_temp_difference > 2.0)
    p_level = P4;
  else if (abs_temp_difference > 1.5)
    p_level = P3;
  else if (abs_temp_difference > 1.0)
    p_level = P2;
  else if (abs_temp_difference > 0.10)
    p_level = P1;  
}

void update_segment(void *context)
{
  segment_idx += 1;
  if (segment_idx >= num_segments)
  {
    setpoint = -1;
    op_state = OFF;
  }
  else
  {
    setpoint = segment_temps[segment_idx];
    t.after(segment_times[segment_idx] * 60 * 60 * 1000, update_segment, (void *)0);
  }
}

void write_output(void *context)
{ 
    Serial.print(current_temp);
    Serial.print(",");
    Serial.print(op_state);
    Serial.print(",");
    Serial.print(p_level);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(millis());
}
