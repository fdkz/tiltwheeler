
motor:
    markings: maxon DC motor / 138062 / swiss made / U 12

    10 mm shell outer diameter.
    25 mm length without gearbox and without encoder.

    Didn't find anything with the code 138062 on www.maxonmotor.com, but code 138061 is an encoder (pinout at least works for us, although ours has 6 pins instead of 10). 

    random info: http://hades.mech.northwestern.edu/index.php/Actuators_Available_in_the_Mechatronics_Lab

    TODO: motor voltage and speed and gearbox info?



https://www.ebay.com/itm/303062402243
sellest on pilt "1 - s-l1600.jpg"
U 12 asemel on X 04 ja "MAXON MOTOR 138062 SWISS MADE 1461165".

https://www.ebay.com/itm/292612931067
samasugune, aga U 12 asemel on 1461165

https://www.ebay.com/itm/Maxon-138062-Micro-Motor-With-Gearbox-And-Encoder-Perfect-for-Arduinos-Robots-/191711805889
siin on udune pilt, aga infot:

  Motor: 9v (12-15V recommended if using PID with PWM control)
  Gearbox: 64:1

  Encoder: 12 counts / rev (so 768 counts / revolution of the output shaft)

  Pinout of the connector:
  1: Motor +
  2: Encoder + 
  3: Encoder output A
  4: Encoder output B
  5: Encoder -
  6: Motor -

https://picclick.fr/Maxon-138062-Micro-Motor-Servo-With-Gearbox-And-113599123214.html
ja siin on W02



------




Derivative Kick

This is a problem that occurs due to the derivative section of the PID control. When a reference change occurs, the output of the control becomes very large, so that the actuator can be saturated.
 

The solution is based on that when the Setpoint value is constant, its derivative is zero, so that the derivative control action becomes:


-------

underactuated robotics


-------

Many forms of
linear systems such as LQR, lead-lag, PID, state feedback and pole placement are
commonly used in robotic stability designs.