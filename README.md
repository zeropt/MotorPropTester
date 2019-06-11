# MotorPropTester
Code to test the power and efficiency of a brushless motor and propeller using a specialized test rig

An arduino, controlling the motor, steps up the power in certain intervals and records the following data to an SD card:
- total time in milliseconds since the rig was powered on
- value the arduino sends to the ESC
- return value from the load sensor
- supplied voltage
- supplied current
- total revolutions since the motor was activated
- current RPM
- current Motor Temperature
- current ESC Temperature
- Whether or not the system is active or finished/killed
