The KK-0530B requires a 24V power supply for activation. 
Unfortunately, the 24V output from the RoboMaster Development Board Type C cannot be controlled directly. 
To resolve this issue, we've integrated an additional circuit. 
By incorporating a Mosfet, we can control the activation of the 24V power using a 3.3V PWM signal from the board.

The circuit can be placed just nearby the control board.

The circuit is very simple:
![circuit_diagram](https://github.com/ntu-hci-lab/SpinShot/blob/main/Hardware/Circuit/solenoid%20control%20circuit/Circuit_Diagram.png)

And can be implement in a very small space:
![circuit_diagram](https://github.com/ntu-hci-lab/SpinShot/blob/main/Hardware/Circuit/solenoid%20control%20circuit/Implementation.png)
