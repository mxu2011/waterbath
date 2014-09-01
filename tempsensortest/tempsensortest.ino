#include <OneWire.h>

#include <DallasTemperature.h>

//DQ connected to D2 on Arduino
#define ONE_WIRE_BUS 2

OneWire oneWire (ONE_WIRE_BUS);

DallasTemperature sensors (&oneWire);

void setup(void)
{
sensors.begin();
}

void loop(void)
{
sensors.requestTemperatures();
delay(1000);

Serial.print("The temperature is: ");
Serial.println(sensors.getTempCByIndex(0));
}
