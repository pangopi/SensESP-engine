# SensESP Engine Monitor

Engine monitor for S/V Pangolin II. This project will monitor sensors on the main engine and send the data to SignalK.

Currently implemented sensors are:
- RPM sensor using an infrared slot sensor mounted to the main pulley
- Temperature monitor for the engine room
- Temperature monitor mounted to the heat exchanger

Planned sensors to be added:
- Oil pressure sensor
- Oil pressure switch
- Alternator output switch
- Coolant temperature sensor

Planned features:
- Sound alarm when value out of bounds
- Record engine run time

Built using [SensESP](https://github.com/SignalK/SensESP/).

