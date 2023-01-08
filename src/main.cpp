#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

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
                    // Set a custom hostname for the app.
                    ->set_hostname("engine_monitor")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi("PangoPi", "mikeharris")
                    ->set_sk_server("192.168.5.1", 3000)
                    ->get_app();

  // The "Signal K path" identifies the output of the sensor to the Signal K
  // network. If you have multiple sensors connected to your microcontoller
  // (ESP), each one of them will (probably) have its own Signal K path
  // variable. For example, if you have two propulsion engines, and you want the
  // RPM of each of them to go to Signal K, you might have sk_path_portEngine =
  // "propulsion.port.revolutions" and sk_path_starboardEngine =
  // "propulsion.starboard.revolutions" In this example, there is only one
  // propulsion engine, and its RPM is the only thing being reported to Signal
  // K. To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.4.0/doc/vesselsBranch.html
  const char* rpm_sk_path = "propulsion.main.revolutions";
  const char* oil_pressure_sk_path = "propulsion.main.oilPressure";
  const char* oil_temp_sk_path = "propulsion.main.oilTemperature";
  const char* coolant_temp_sk_path = "propulsion.main.coolantTemperature";
  const char* exhaust_temp_sk_path = "propulsion.main.exhaustTemperature";
  const char* runtime_sk_path = "propulsion.main.runTime";

  // propulsion.main.revolutions - engine rpm in Hz
  // propulsion.main.state - 
  // propulsion.main.temperature - Engine temperature (K)
  // propulsion.main.oilTemperature - in K
  // propulsion.main.oilPressure - in Pa
  // propulsion.main.alternatorVoltage - in V
  // propulsion.main.coolantTemperature - in K
  // propulsion.main.coolantPressure - in Pa
  // propulsion.main.exhaustTemperature - in K
  // propulsion.main.runTime - Engine hours in s

  // The "Configuration path" is combined with "/config" to formulate a URL
  // used by the RESTful API for retrieving or setting configuration data.
  // It is ALSO used to specify a path to the SPIFFS file system
  // where configuration data is saved on the microcontroller.  It should
  // ALWAYS start with a forward slash if specified. If left blank,
  // that indicates this sensor or transform does not have any
  // configuration to save.
  // Note that if you want to be able to change the sk_path at runtime,
  // you will need to specify a configuration path.
  // As with the Signal K path, if you have multiple configurable sensors
  // connected to the microcontroller, you will have a configuration path
  // for each of them, such as config_path_portEngine =
  // "/sensors/portEngine/rpm" and config_path_starboardEngine =
  // "/sensor/starboardEngine/rpm".
  const char* rpm_config_path = "/sensors/engine_rpm";
  const char* oil_pressure_config_path = "sensors/oilPressure";
  const char* oil_temp_config_path = "sensors/oilTemperature";
  const char* coolant_temp_config_path = "sensors/coolantTemperature";
  const char* exhaust_temp_config_path = "sensors/exhaustTemperature";
  const char* runtime_config_path = "sensors/runTime";
  // These two are necessary until a method is created to synthesize them.
  // Everything after "/sensors" in each of these ("/engine_rpm/calibrate" and
  // "/engine_rpm/sk") is simply a label to display what you're configuring in
  // the Configuration UI.
  const char* rpm_config_path_calibrate = "/sensors/engine_rpm/calibrate";
  const char* rpm_config_path_skpath = "/sensors/engine_rpm/sk";

  //////////
  // connect a RPM meter. A DigitalInputCounter implements an interrupt
  // to count pulses and reports the readings every read_delay ms
  // (500 in the example). A Frequency
  // transform takes a number of pulses and converts that into
  // a frequency. The sample multiplier converts the 97 tooth
  // tach output into Hz, SK native units.
  const float rpm_multiplier = 1.0 / 1.0;
  const unsigned int rpm_read_delay = 503; // prime number

  // Wire it all up by connecting the producer directly to the consumer
  // ESP32 pins are specified as just the X in GPIOX
  uint8_t rpm_pin = 5;

  auto* rpm_sensor = new DigitalInputCounter(rpm_pin, INPUT_PULLUP, RISING, rpm_read_delay);

  rpm_sensor
      ->connect_to(new Frequency(
          rpm_multiplier, rpm_config_path_calibrate))  // connect the output of sensor
                                               // to the input of Frequency()
      ->connect_to(new SKOutputFloat(
          rpm_sk_path, rpm_config_path_skpath));  // connect the output of Frequency()
                                          // to a Signal K Output as a number

  /*
     Find all the sensors and their unique addresses. Then, each new instance
     of OneWireTemperature will use one of those addresses. You can't specify
     which address will initially be assigned to a particular sensor, so if you
     have more than one sensor, you may have to swap the addresses around on
     the configuration page for the device. (You get to the configuration page
     by entering the IP address of the device into a browser.)
  */

  /*
     Tell SensESP where the sensor is connected to the board
     ESP32 pins are specified as just the X in GPIOX
  */
  uint8_t dts_pin = 4;

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(dts_pin);

  // Define how often SensESP should read the sensor(s) in milliseconds
  uint dts_read_delay = 2003; // prime number

  // Measure coolant temperature
  auto* coolant_temp =
      new OneWireTemperature(dts, dts_read_delay, "/coolantTemperature/oneWire");

  coolant_temp->connect_to(new Linear(1.0, 0.0, "/coolantTemperature/linear"))
      ->connect_to(new SKOutputFloat("propulsion.main.coolantTemperature",
                                     "/coolantTemperature/skPath"));

  // Measure exhaust temperature
  auto* exhaust_temp =
      new OneWireTemperature(dts, dts_read_delay, "/exhaustTemperature/oneWire");

  exhaust_temp->connect_to(new Linear(1.0, 0.0, "/exhaustTemperature/linear"))
      ->connect_to(new SKOutputFloat("propulsion.main.exhaustTemperature",
                                     "/exhaustTemperature/skPath"));


  // Start the SensESP application running. Because of everything that's been
  // set up above, it constantly monitors the interrupt pin, and every
  // read_delay ms, it sends the calculated frequency to Signal K.
  sensesp_app->start();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { app.tick(); }