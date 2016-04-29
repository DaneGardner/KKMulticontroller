This was originally available on kkmulticopter.com, dated Jan 2 2011.

-------------------------- Quote Follows --------------------------

The v.5.5 "Blackboard" KKmulticontroller is a SMD constructed PCB with in built Murata Piezo Gyros.

The KKmulticontroller is a flight control board for remote control multicopters with 2,3,4 and 6 rotors. Its purpose is to stablise the aircraft during flight. To do this it takes the signal from the three gyros on the board (roll, pitch and yaw) and feeds the information into the Integrated Circuit (Atmega IC). This then processes the information according the the KK software and sends out a control signal to the Electronic Speed Controllers (ESCs) which are plugged onto the board and also connected to the motors. Depending upon the signal from the IC the ESCs will either speed up or slow down the motors (and tilt the rear rotor with a servo in a Tricopter) in order to establish level flight.
 
The board also takes a control signal from the Remote Control Receiver (RX) and feeds this into the IC via the aileron, elevator, throttle and rudder pins on the board. After processing this information, the IC will then send out a signal to the motors (Via the M1 to M6 pins on the board) to speed up or slow down to achieve controlled flight (up, down, backwards, forwards, left, right, yaw)  on the command from the RC Pilot sent via his Transmitter (TX). In the case of a Tricopter, one of the pin connectors (M4) will control a servo to achieve yaw authority.
 
The v.5.5 has an Atmega168 chip on board and an ISP header which gives users the option to tweak and upload their own controller code.

* Onboard Gyros ENC-03-RC (small version Gyrostar)
* Changed all caps, except C5 C6, C7 and C8 for ceramic 0.1 uF
* Slot mounted Yaw gyro
* Atmega168 instead of 48 (More memory so you can play with the XXcontroller soft by Mike Barton, or write your own)
* Funky black soldermask.

Designs are here for those who wants to try and make their own board and as a reference. Private, non-commercial reproduction only!
