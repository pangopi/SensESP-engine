#include <Arduino.h>

#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/time_counter.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/transforms/threshold.h"

using namespace sensesp;

//#define SERIAL_DEBUG_DISABLED

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
  const uint8_t alarm_pin = 7;  // The alarm pin
  const uint8_t rpm_pin = 5;    // For the RPM counter
  const uint8_t dts_pin = 3;    // The Dallas Temperature Sensors
  const uint8_t lpg_pin = 18;   // Digital in for LPG alarm
  const uint8_t coolant_pin = 11; // Analog in for coolant sender
  const uint8_t oil_pressure_pin = 12; // Analog in for the oil pressure sender
  const uint8_t oil_switch = 16; // Digital in for the oil pressure switch

  // connect a RPM meter. A DigitalInputCounter implements an interrupt
  // to count pulses and reports the readings every read_delay ms. A Frequency
  // transform takes a number of pulses and converts that into a frequency. 
  const float rpm_multiplier = 1.0; //1.0 / 1.0;
  const unsigned int rpm_read_delay = 503;  // prime number
  const unsigned int rpm_debounce_ignore_interval = 15;


  auto* rpm_sensor =
    //new DigitalInputCounter(rpm_pin, INPUT_PULLDOWN, RISING, rpm_read_delay);
    new DigitalInputDebounceCounter(rpm_pin, INPUT_PULLDOWN, RISING, rpm_read_delay, rpm_debounce_ignore_interval);

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
  /////////////////////////////////////
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
  //////////////////////////////////
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

// class TemperatureInterpreter : public CurveInterpolator {
//  public:
//   TemperatureInterpreter(String config_path = "")
//       : CurveInterpolator(NULL, config_path) {
//     // Populate a lookup table tp translate the ohm values returned by
//     // our temperature sender to degrees Kelvin
//     clear_samples();
//     // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
//     add_sample(CurveInterpolator::Sample(0, 418.9));
//     add_sample(CurveInterpolator::Sample(5, 414.71));
//     add_sample(CurveInterpolator::Sample(36, 388.71));
//     add_sample(CurveInterpolator::Sample(56, 371.93));
//     add_sample(CurveInterpolator::Sample(59, 366.48));
//     add_sample(CurveInterpolator::Sample(81, 355.37));
//     add_sample(CurveInterpolator::Sample(112, 344.26));
//     add_sample(CurveInterpolator::Sample(240, 322.04));
//     add_sample(CurveInterpolator::Sample(550, 255.37));
//     add_sample(CurveInterpolator::Sample(10000, 237.6));
//   }
// };

//   // Coolant temperature sender
//   /////////////////////////////
//   // Sensor---R1--|--R2---GND
//   //              |-ADC
//   const float coolant_Vin = 3.3; // Volatge deiveder nominal voltage
//   const float coolant_R2 = 46.0; // Voltage divider R2 for coolant temp sender
//   auto* coolant_temp = new AnalogInput(coolant_pin, 500, "/propulsion/coolant/ADC", 3.3F);
//   coolant_temp->connect_to(new AnalogVoltage())
//       ->connect_to(new VoltageDividerR2(coolant_R2, coolant_Vin,
//                                         "/propulsion/coolant/sender"))
//       ->connect_to(new TemperatureInterpreter("/propulsion/coolant/curve"))
//       ->connect_to(new Linear(1.0, 0.0, "/propulsion/coolant/calibrate"))
//       ->connect_to(new SKOutputFloat(
//           "propulsion.main.coolant.temperature", "/propulsion/coolant/temp",
//           new SKMetadata("K", "Coolant Temperature",
//                          "Engine coolant temperature", "Coolant Temp", 2000)));
  
  // Oil pressure sender
  //////////////////////
  // const float oilPressure_Vin = 3.3;
  // const float oilPressure_R2 = 46.0; 
  // auto* oilPresssure = new AnalogInput(oil_pressure_pin, 500, "/propulsion/oilPressure/sender");
  // oilPresssure
  //     ->connect_to(new Linear(1.2, 3.3, "/propulsion/oilPressure/linear"))
  //     ->connect_to(new SKOutputNumeric("propulsion.main.oilPressure"));

  // Engine oil pressure switch (on when the pressure is too low)
  ///////////////////////////////////////////////////////////////
  // auto* oil_pressure_switch = new DigitalInputState(oil_pressure_pin, INPUT, 500, "");
  // oil_pressure_switch->connect_to(new SKOutputBool(
  //     "notifications.oilPressureAlarm",
  //     new SKMetadata("boolean", "Oil Pressure Alarm",
  //                    "Oil pressure switch registering low oil pressure",
  //                    "Oil Switch", 1000)));

  // // LPG gas sensor alarm input
  // auto* lpg_in = new DigitalInputState(lpg_pin, INPUT, 500, "");
  // lpg_in->connect_to(new SKOutputBool(
  //     "notifications.LPGAlarm",
  //     new SKMetadata("boolean", "LPG Gas Alarm", "State of the LPG gas sensor",
  //                    "Gas Alarm", 5000)));

  // ALARMS
  /////////
  // Wire up the output of the heat exchanger, and then output
  // the transformed float to boolean to DigitalOutput
  // Threshold = 273.15 + 95 Kelvin => 95 degrees Celcius
  // heat_exchanger_temp
  //     ->connect_to(new FloatThreshold(0.0f, 273.15f + 95.0f, false,
  //                                     "/threshold/heatExchangerTemp"))
  //     ->connect_to(new DigitalOutput(alarm_pin));

  // Connect the output of the rpm sensor to a threshold transform and connect
  // to a digital output.
  // 3300 rpm / 60 = 55 Hz
  // rpm_sensor
  //     ->connect_to(
  //         new FloatThreshold(0.0f, 3300.0f / 60, false, "/threshold/revolutions"))
  //     ->connect_to(new DigitalOutput(alarm_pin));
  
  // Start the SensESP application running. Because of everything that's been
  // set up above, it constantly monitors the interrupt pin, and every
  // read_delay ms, it sends the calculated frequency to Signal K.
  sensesp_app->start();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { app.tick(); }