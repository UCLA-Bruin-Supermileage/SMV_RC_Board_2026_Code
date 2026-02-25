# SMV Rear Control Board Testbench

Currently in development. There are pin mapping conflicts for the tail lights, brake lights, and turn signal light 1 (uses pins that are auto-claimed by STM32 for certain built in functions)

# Pin Mapping
| Name | Pin |
| -------- | -------- |
| CS_ADC | PB4 |
| Turn_Signal_2 | PB7 |
| CAN1_RX | PB8 |
| CAN1_TX | PB9 |
| CS_Accelerometer | PB12 |
| SPI2_SCK | PB13 |
| SPI2_MISO | PB14 |
| SPI2_MOSI | PB15 |
| Tail_Light | PA13 (conflict) |
| Turn_Signal_1 | PA14 (conflict) |
| Brake_Light | PH1 (conflict) |
