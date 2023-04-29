# BTLathe
CNC Lathe machine, controlled with Android via ESP32 by BT, Servo57c by two UART

OneByte commands:
a  takeAlarm();
p  takePause();
s  shaveShaft();
c  sliceShaft();
t  cutTail();
f  finaliseTale();
A  flagAbort = true;

MultyByte commands:
SDNNNN, wherein:
S = {l1234567890}  -> {lockCurrent, motorSpeed}
D = {xylrio}       -> {goToX, goToY, left, rigth, in, out}
NNNN = [0 to 9999] -> target coordinate

Example 1: 5r100  -> on 5th speed move 100 tenths to the rigth
Example 2: 7x2500 -> on 7th speed move to x = 2500
Example 3: li5    -> on locked current move 5 tenths in