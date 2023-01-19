#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/transforms/threshold.h"

using namespace sensesp;

// SensESP builds upon the ReactESP framework. Every ReactESP application
// defines an "app" object.
ReactESP app;

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("engine_monitor")
                    ->set_wifi("PangoPi", "mikeharris")
                    ->set_sk_server("192.168.5.1", 3000)
                    ->enable_ota("pangolin")
                    ->get_app();
  
  // Define the pin connection
  uint8_t alarm_pin = 6;  // The alarm pin
  uint8_t rpm_pin = 5;    // For the RPM counter
  uint8_t dts_pin = 4;    // The Dallas Temperature Sensors

  // connect a RPM meter. A DigitalInputCounter implements an interrupt
  // to count pulses and reports the readings every read_delay ms. A Frequency
  // transform takes a number of pulses and converts that into a frequency. 
  const float rpm_multiplier = 1.0 / 1.0;
  const unsigned int rpm_read_delay = 503;  // prime number

  auto* rpm_sensor =
    new DigitalInputCounter(rpm_pin, INPUT_PULLUP, RISING, rpm_read_delay);

  rpm_sensor
    ->connect_to(new Frequency(rpm_multiplier, 
                               "/sensors/engine_rpm/calibrate"))  
    // connect the output of sensor to the input of Frequency()
    ->connect_to(new SKOutputFloat("propulsion.main.revolutions", 
                                   "/sensors/engine_rpm/sk"));

  // OneWire Temperature Sensors
  //////////////////////////////

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(dts_pin);

  // Define how often SensESP should read the sensor(s) in milliseconds
  uint dts_read_delay = 2003; // prime number close to 2 seconds.

  // Measure heat exchanger temperature
  auto* heat_exchanger_temp =
    new OneWireTemperature(dts, dts_read_delay, "/heatExchangerTemperature/oneWire");

  //  Create metadata to add to the SKOutput
  SKMetadata* heat_exchanger_metadata = new SKMetadata();
  heat_exchanger_metadata->units_ = "K";
  heat_exchanger_metadata->display_name_ = "Heat Exchanger Temperature";
  heat_exchanger_metadata->short_name_ = "Exchanger Temp";
  heat_exchanger_metadata->description_ = "Main engine heat exchanger temperature";
  heat_exchanger_metadata->timeout_ = 900;  // 15 minutes
  
  heat_exchanger_temp->connect_to(new Linear(1.0, 0.0, "/heatExhangerTemperature/linear"))
    ->connect_to(new SKOutputFloat("propulsion.main.heatExchangerTemperature",
                                   "/heatExchangerTemperature/skPath",
                                   heat_exchanger_metadata));

  // Measure engine room temperature
  auto* room_temp =
    new OneWireTemperature(dts, dts_read_delay, "/engineRoomTemperature/oneWire");

  //  Create metadata to add to the SKOutput
  SKMetadata* room_temp_metadata = new SKMetadata();
  room_temp_metadata->units_ = "K";
  room_temp_metadata->display_name_ = "Engine Room Temperature";
  room_temp_metadata->short_name_ = "Engine Room Temp";
  room_temp_metadata->description_ = "Engine Room Temperature";
  room_temp_metadata->timeout_ = 900;  // 15 minutes

  room_temp->connect_to(new Linear(1.0, 0.0, "/engineRoomTemperature/linear"))
    ->connect_to(new SKOutputFloat("environment.inside.engineRoom.temperature",
                                   "/engineRoomTemperature/skPath", 
                                   room_temp_metadata));

  // Alarm listeners
  const char* config_path = "/threshold/lights";

  // Wire up the output of the heat exchanger, and then output
  // the transformed float to boolean to DigitalOutput
  // Threshold = 273.15 + 95 Kelvin => 95 degrees Celcius
  heat_exchanger_temp
      ->connect_to(new FloatThreshold(0.0f, 273.15f + 95.0f, false,
                                      "/threshold/heatExchangerTemp"))
      ->connect_to(new DigitalOutput(alarm_pin));

  // Connect the output of the rpm sensor to a threshold transform and connect
  // to a digital output.
  // 3300 rpm / 60 = 55 Hz
  rpm_sensor
      ->connect_to(
          new FloatThreshold(0.0f, 3300.0f / 60, false, "/threshold/revolutions"))
      ->connect_to(new DigitalOutput(alarm_pin));
  
  // Start the SensESP application running. Because of everything that's been
  // set up above, it constantly monitors the interrupt pin, and every
  // read_delay ms, it sends the calculated frequency to Signal K.
  sensesp_app->start();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { app.tick(); }