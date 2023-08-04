# AS5600

Experiment to see if the cheap AMS AS5600 magnetic encoders will work over I2C.

These common on Amazon, for example:

https://www.amazon.com/Hyhpsjiee-Magnetic-Induction-Measurement-Precision/dp/B0BS1LCSR8

they have seven pins:

* VCC 3.3v
* GND
* ratiometric output
* direction (clockwise positive or reversed), connect this to ground
* GPO for programming, has a pull up, leave it alone
* SCL
* SDA

The I2C address is fixed at 0x36.  For multiples (e.g. swerve steering), we should
use an I2C multiplexer (e.g. PCA9546), but for this experiment i'll just use 0x36.


