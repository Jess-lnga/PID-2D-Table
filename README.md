# PID-2D-Table
This is the code and other ressources for a second version of a ball balancing robot. This time, the ball will be allowed to move in two dimensions instead of one. Go check my other projects to see the 1 D version.

The system will have a ball rolling freely on a plane. The goal is to bring the ball to the center of the table as fast as possible everytime there is a disturbance. Some fun implementation can be done later, for example, make the ball follow a circle, or other curves. This kind of project has been done multiple times, with many different architectures. Here are some good implementations that I saw:
1- https://youtu.be/kAaYaZcpbLo?si=GLI-FVm3SjZnmLwM from the AED MUSA 
2- 

So, not a revolution in robotics, but still a cool project. I decided to do my own implementation, but inspired from what I saw

To be able to bring the ball back to the center, we will need a way to actuate the surface on roll and pitch. To do so, we implemented a sort of Stewart platform but with less degrees of freedom (DOF). A Steward platform has 6 actuators that are placed around a surface. Most of the implementations use linear actuators. But at the end, we can move the end effector with 6 DOF. Here is an example of a Stewart platform: https://fr.wikipedia.org/wiki/Plateforme_de_Stewart

For our system, we don't need all the 6 degrees of freedom, but only two. I decided to implement a system with only 3 actuators placed regularly around the plate. We can have 3 DOF with this disposition: Translation along z, pitch and roll rotations. Theoretically, we do not need the translation on the z axis, but the height of the platform is going to affect the amplitude of the roll and pitch movement.

<img width="1536" height="2040" alt="WhatsApp Image 2026-09-24 at 22 25 52" src="https://github.com/user-attachments/assets/67091cfa-fb0b-437f-bc8a-d6e74ca5c729" />

Here are the sketchs that I did for the first implementation. 


# **Software and sensors**

I used TIAKONGRC motors with 20 kg of torque, but my main problem is the speed of actuation. I do not know if the platform will react fast enough (I did not do enough previsions, so I am going to test and see if its enough). Here is the reference of the motors: https://bayhobbies.co.nz/tiankongac-20kg-digital-servo-high-torque-full-metal-gear-waterproof-servo-270-degree-with-25t-servo-horn/?srsltid=AU7gw4U7lks168itvwmimeYvdkK77_uPc8vz9mkhQFHDOvutGmBhPWa1

The way I am going to test it, is by manually control the motors to balance the ball instead of using a controller. To do so, I will link the system to an accelerometer. 
The accelerometer will provide data for roll and pitch. The esp32 will receive it, and compute the inverse kinematics so that the platform matches the inclination of the sensor. I decided to use my smartphone to generate the IMU values. 


