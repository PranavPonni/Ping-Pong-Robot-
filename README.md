# Ping-Pong-Robot-
Versions of Ping Pong Robot Arduino code, for shooting and movement. (Need hardware to replicate and run)

Understanding how code works:

The code attached is the part of the main code that I worked on that mainly executes the robot's movement
as well as the encoding of the hall sensor e?ect. If you see the 2nd and 3rd images, the user inputs di?erent
types of commands: (open, test, push, o?) and the robot taking the user input changes the motor output
accordingly, for example, test, leads the motor to forward for 5 seconds to “test” the movement of the robot.
Similarly, with repeated on-site testing, we were able to find the function of the angle at which the robot
shoots the ping pong ball. For the codes, I referred to various websites, and pre-existing codes, and debugged
them using Chat-GPT to find errors and improvements. For the code formatting, I used the Arduino IDE and
various library files to improve the code. With repeated testing and on-site ball shooting, we wrote down the
di?erent angles from the gyro sensor (gyroscope) to shoot into bins 1,2 & 3. We made an Excel spreadsheet
for 5-6 throws, averaged the values, and used the average value in the Arduino code to store the actual
shooting angle for the robot obtained from the test experiment. The moveServo functions in the main code
helped control the servo motor's movement to a specified angle. The function takes in the o?set value and
adjusts the servo's position to reach that angle. This is done by calculating the angle di?erence and updating
in the loop, the servo angle until the target is reached.
I also worked on PID control and the encoder’s feedback from the hall sensors.
The values for Kp, Ki & Kd were done by multiple tests and done with an estimation as it was hard to figure
out the right values using integral and derivation equations/calculations. Repeated testing proved that we
could modify the variables, a little greater than or lesser than and hence were able to process a more
accurate movement. The rotation_robot function is responsible for controlling the rotation of the robot using
PID control with DC motors equipped with encoders. The while loop keeps running until there is a request to
stop and using the PID values, the motor speed and directions are adjusted accordingly to achieve the
required direction to shoot.
Although I am not proficient in MATLAB, I managed to help Taichi oversee the robot controller programming.
Since he had remote controlling software programmed for a drone from a previous course, we decided to use
that existing software and change it accordingly to our robot. We changed the buttons of the drone to the
movement of the motors of the robot and added extra code to relay the Arduino shooting manual onto the
MATLAB module so that when we click the shoot button on the remote-control software, using the Bluetooth
XBEE the signal relays from the Arduino code and hence the ball is shot at the predetermined angle into the
bin. I also made minor adjustments to the movement of the robot due to spatial restrictions. Since the robot
had to be placed and seated within the 30 cm * 30 cm square, instead of turning both motors to change
directions, we rotated the left motor anticlockwise and the right motor clockwise if we wanted to turn right,
thus staying in the box yet changing directions. Similarly, we kept a servo motor that adjusts the height of the
shooting arm to make sure that we don’t have to move back and forth unnecessarily.
