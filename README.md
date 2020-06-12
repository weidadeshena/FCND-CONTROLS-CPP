#### Writeup
## Implemented body rate control in C++.
A P controller to control the rotating moment using the error of body rate. Since moment equals to moment of inertia I times \omega dot, the output of the controller can be calculated as
'''
  V3F pqrErr = pqrCmd - pqr;
  V3F I(Ixx, Iyy, Izz);
  momentCmd = I * (kpPQR * pqrErr);
'''
## Implement roll pitch control in C++.
A P controller is used to control the roll and pitch. When the collective thrust is bigger than 0, the acceleration of the drone is (Newton's second law)
'''
float c_d = -collThrustCmd / mass;
'''
With the constraint of the tilt angle, we can find the command angle as
'''
      float b_x_c = accelCmd.x / c_d;
      b_x_c = CONSTRAIN(b_x_c, -maxTiltAngle, maxTiltAngle);
      float b_y_c = accelCmd.y / c_d;
      b_y_c = CONSTRAIN(b_y_c, -maxTiltAngle, maxTiltAngle);
'''
Then, we can find the command output as
'''
      float b_x_c_dot = kpBank * (b_x_c - R(0,2));
      float b_y_c_dot = kpBank * (b_y_c - R(1,2));
      float p = (R(1,0) * b_x_c_dot - R(0,0) * b_y_c_dot) / R(2,2);
      float q = (R(1,1) * b_x_c_dot - R(0,1) * b_y_c_dot) / R(2,2);
      pqrCmd.x = p;
      pqrCmd.y = q;
      pqrCmd.z = 0.f;
'''
If the collective thrust is 0, the command is set to 0.
## Implement altitude controller in C++.
A PID controller is used to controll the attitude. With the constraint of maximum vertical speed, we have
'''
velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
'''
Then, we can implement the PID controller as
'''
  float p = kpPosZ * (posZCmd - posZ);
  integratedAltitudeError += (posZCmd - posZ) * dt;
  float i = KiPosZ * integratedAltitudeError;
  float d = kpVelZ * (velZCmd - velZ);
  float u_1_bar = p + i + d + accelZCmd;
'''
The thrust is calculated as
'''
  float c = (u_1_bar - (float)CONST_GRAVITY) / R(2,2);
  thrust = - mass * c ;
'''
since on z axis the positive side is downwards.
## Implement lateral position control in C++.
A PD controller is used to control the lateral position.
With the speed contraint we have
'''
if (velCmd.mag() > maxSpeedXY) {
          velCmd = velCmd.norm() * maxSpeedXY;
      }
'''
Then, we can implement the PD control as
'''
  V3F p = kpPosXY * (posCmd - pos);
  V3F d = kpVelXY * (velCmd - vel);
  accelCmd = accelCmdFF + p + d;
'''
Finally, limit the acceleration:
'''
  if (accelCmd.mag() > maxAccelXY) {
          accelCmd = accelCmd.norm() * maxAccelXY;
      }
'''
## Implement yaw control in C++.
A P controller is used to calculate the yaw command. The yaw angle error is restricted between 0 and 2\pi.
'''
  float yawErr = yawCmd - yaw;
  yawErr = fmodf(yawErr, 2 * F_PI);
  yawRateCmd = kpYaw * yawErr;
'''
## Implement calculating the motor commands given commanded thrust and moments in C++.
From the matrix equation given in the video we can easily find the frequency squared, which is proportional to the force and moment. Hence we have
'''
  cmd.desiredThrustsN[0] = (collThrustCmd + momentCmd.x/L + momentCmd.y/L - momentCmd.z/kappa) / 4.f; // front left
  cmd.desiredThrustsN[1] = (collThrustCmd - momentCmd.x/L + momentCmd.y/L + momentCmd.z/kappa) / 4.f; // front right
  cmd.desiredThrustsN[2] = (collThrustCmd + momentCmd.x/L - momentCmd.y/L + momentCmd.z/kappa) / 4.f; // rear left
  cmd.desiredThrustsN[3] = (collThrustCmd - momentCmd.x/L - momentCmd.y/L - momentCmd.z/kappa) / 4.f; // rear right
'''
Due to z axis pointing downwards, the sign is flipped.





