Hexapod project code to make it run. 
In the future, servos might have to be controlled via a microcontroller to allow faster update and more smooth operation. 
Startingpoint of this project was this: https://github.com/CoretechR/Zerobug. (Tibia is used from this project, the other parts had to be adapted for the servos used here)


Two PCA9685 PWM controllers are responsible for servo control via an I2C interface, controlled by a Rpi Zero.
Asselbly should be quite straightforeward, control at this stage is easiest connecting to the Rpi via ssh.

Have fun!
