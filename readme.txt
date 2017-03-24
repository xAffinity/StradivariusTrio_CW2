Controls:
1.RXXX.XX -> Spins XXX.XX revolutions in the clockwise direction ( at a set speed of 5 revolutions per second ) 
2.VXXX.XX -> Spins at an angular velocity of XXX.XX revolutions per second in the clockwise direction
3.R-XXX.XX -> Spins XXX.XX revolutions in the anti-clockwise direction
4.V-XXX.XX -> Spins at an angular velocity of XXX.XX revolutions per second in the anti-clockwise direction
5.RXXX.XXVYYY.YY -> Spins XXX.XX revolutions at YYY.YY revolutions per second in the clockwise direction
6.R-XXX.XXVYYY.YY -> Spins XXX.XX revolutions at YYY.YY revolutions per second in the anti-clockwise direction
7.TNDNDND..... -> Plays a repeatitive tune of up to 16 notes where N is a note from a single octave ( including flats and sharps ) and D is the duration of each note ( in beats ) in the range from 1 to 8 beats.

Note:
1.For R and V commands XXX.XX/YYY.YY means you can input any number in the precision as specified in the specification (eg.R100 or V20.1 or R100.33V2.0)

Instructions:
1.From the slre_master file, import slre.c and slre.h into the compiler. 
2.When connected to the PC Serial Monitor, press reset and wait for the command terminal to show the Rotor Origin. 
3.When Rotor Origin is seen, you can now input control commands from above into the motor.
4.If your command has been received, the command terminal should print your settings.
5.Commands should be able to be sent one after another without pushing the reset button and the motor will perform the latest command. In the event the motor is not responding ( normally occurs when motor is in the rest state ) , you could try giving a slight push to the rotating disc, or press the reset button and try commaning it again. 
6.If you give any command after a tune command (eg. TA4C5F#8...) , the motor will take around 0.5-1 seconds to react to the command. You do not have to press the reset button if this occurs.
7.For the RV commands, if impossible settings are set (eg.R10V10), the motor will not work properly. 
8.If you want to add a - in the command, it always comes is as the second character (eg. R-100V20 or V-20 ).  It will not accept commands that does not respects this rule ( eg. R-100V-20 or R100V-20 ).

