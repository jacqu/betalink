# betalink
Betalink is a modified Betaflight firmware that adds one serial command to the
MSP protocol that controls the motor velocity and retrieve the state of the
FC in only one transaction. It adds a motor velocity PID on the FC that uses 
bidirectional DSHOT telemetry as feedback. A command line program called "betalink"
is provided to test the firmware. A Simulink block is also provided within
the project "RPIt" to interface the FC within Simulink thanks to a dedicated betalink
block.
