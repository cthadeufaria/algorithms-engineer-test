# Coding Question

### Respository containing my personal solution to a coding question described on the text below.

When performing lower limb exercises, patients have motion sensors placed on the chest, thighs and shanks. Each of these sensors provides two types of information:

1. a 3D unit vector (reference vector) describing the orientation of the underlying segment - the spine for the chest sensor, the femur for the thigh sensors and the tibia for the shank sensors - in a coordinate system where the x component corresponds to Earth’s North, the y component corresponds to Earth’s West direction and the z component corresponds to the “up” direction;
2. the estimated norm of the linear acceleration in m/s2 (0 corresponds to the sensor being at rest) 


Assume you have a patient with the sensors placed and facing the North 
direction while performing a single repetition of the following movement: first lifting the right leg with the knee bent and then lifting the left leg in the exact same fashion. You will receive five continuous real-time streams of sensor data, i.e. for each instant in time (take 50 Hz as a possible rate) you receive one pair of reference vector and acceleration from each sensor, but you do not know on which of the five possible positions - chest, right thigh, left thigh, right shank and left shank - each sensor is placed although you do know that there is one sensor on each of these positions. The goal of this challenge is to propose a solution capable of determining the position of each of the 5 sensors. Attached to this test you will have a helper project in Python (./supportlib), which reads data from a file and  passes  it  into  an  abstract  class  for  processing,  replicating  the  use-case  of  a  real-time implementation and enabling you to implement and test your solution effectively.
Implement your solution within the project while complying with the following requirements:
1. Except for the TODO in the main file, all your code must be within a separate package (all existing  code  files  are  there  to  support  your  implementation  and  do  not  need  to  be changed)
2. Your  top  class  must  extend  SensorPositionFinder  and  must  be  instantiated  in  the mentioned TODO
3. Each  time  an  unknown  sensor  position  is  identified,  your  top  class  must  call  the on_sensor_position_found method of SensorPositionRequester
4. Once all sensor positions are identified, your implementation should call the   on_finish method of SensorPositionRequester