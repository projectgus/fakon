## Connections

Breakout board connections for development:

| Signal                        | To                                              | Type                                | Note                                                                                                                                                                         |
|-------------------------------|-------------------------------------------------|-------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PCAN L/H                      | Various (incl EE21 pins 12,13)                  | CAN pair                            |                                                                                                                                                                              |
| COMP CAN L/H                  | AC compressor (via EE21 pins 77,76), PTC heater | CAN pair                            |                                                                                                                                                                              |
| Brake light switch            | Switch, EPCU (EE21 pin 4)                       | 12V Input                           |                                                                                                                                                                              |
| Brake light test switch       | Switch, EPCU (EE21 pin 5)                       | 12V Input                           | Use SPDT switch for development, "Test" may need to become output for final board.                                                                                           |
| IMMO                          | EPCU (EE21 pin 6)                               | K-Line (can use LIN transceiver)    |                                                                                                                                                                              |
| SCU Park TX                   | EPCU (EE21 pin 30)                              | 5V PP Output                        | 10Hz PWM signal, maybe can be OD pulled up at EPCU                                                                                                                           |
| SCU Park RX                   | EPCU (EE21 pin 31)                              | 5V Input                            | 10Hz PWM signal, may need pullup to 5V                                                                                                                                       |
| SBW Button Output             | EPCU (EE21 pin 36)                             | 12V PP? Output                      | 100Hz(?) PWM. Maybe OD, Only needed if replacing OEM SBW (shift by wire).                                                                                                    |
| SBW Input                     | EPCU (EE21 pin 10)                              | 12V Input                           | 100Hz PWM. Probably OD with pullup. Only needed if replacing OEM SBW.                                                                                                        |
| EV Ready Input                | EPCU (EE21 pin 71)                              | 12V Input (w/ pullup)               | Redundant signal, maybe not needed in final hardware.                                                                                                                        |
| P_STS Input                   | EPCU (EE21 pin 72)                              | 12V Input (w/ pullup)               | 50Hz PWM. Redundant gear selector signal, maybe not needed in final hardware.                                                                                                |
| A/C Compressor Interlock +    | Compressor (EE21 pin 3)                         | 5V Output                           |                                                                                                                                                                              |
| A/C Compressor Interlock -    | Compressor (EE21 pin 50)                        | 5V Input                            |                                                                                                                                                                              |
| Heater Interlock +            | PTC Heater LV connector                         | 5V Output                           |                                                                                                                                                                              |
| Heater Interlock -            | PTC Heater LV connector                         | 5V Input                            |                                                                                                                                                                              |
| Brake trigger output          | "Vehicle loom"                                  | 12V Output                          | Details may depend on vehicle...                                                                                                                                             |
| Charge port door signal 5V    | Charging connector pin 2                        | 5V Power                            |                                                                                                                                                                              |
| Charge port door signal       | Charging connector pin 2                        | 5V Power                            |                                                                                                                                                                              |
| Charge port lock coil drive   | Relay to charging connector                     | Low side 12V relay coil driver      | Check where these relays are, do I still have them?                                                                                                                          |
| Charge port unlock coil drive | Relay to charging connector                     | Low side 12V relay coil driver      |                                                                                                                                                                              |
| "IG1" power sense input       | Power                                           | 12V Input                           |                                                                                                                                                                              |
| "IG3" power sense input       | Power                                           | 12V Input                           |                                                                                                                                                                              |
| Starter signal monitor        | "Vehicle loom", EPCU (EE21 pin 7)               | 12V Input                           |                                                                                                                                                                              |
| IG3 Relay coil output         | Relay box, common with EE21 pin 69 I think      | High side 12V relay coil driver     | Can be driven either by Fakon or by the OBC.                                                                                                                                 |
| Airbag crash signal           | BMS connector pin NNN                           | 12V OD Output                       | 50Hz PWM, see https://openinverter.org/forum/viewtopic.php?p=62451#p62451 . On final design maybe make this a dedicated module with inertia switch fitted (to meet AU regs). |
| Parking brake signal          | "Vehicle loom"                                  | 12V Input                           |                                                                                                                                                                              |
| Steering button inputs        | "Vehicle loom"                                  | 12V Input                           |                                                                                                                                                                              |
| EPCU Wake Up signal           | EPCU (EE21 pin 41)                              | Output (probably?), voltage unknown | The gateway uses this signal for something, currently unknown                                                                                                                |

## Additional test wiring

* EE21 pin 74 Battery charging switch (active low?)
* EE21 pin 75 Battery charging illumination output







