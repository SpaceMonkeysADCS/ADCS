void Attitude_PD(float q_BW[4], float q_e[4], float omega[3], float Kp[3][3], float Kd[3][3], float I[3][3], float wheel_tau[3], float FgW[3], float r_COMB[3]){
  //Max Howerter 3/26/2026
  //
  //This is the main attitude controller for the sat, following the controll law specified in the controller documentation
  //
  //INPUTS:
  // q_BW:      IMU quaternion                             1x4
  // q_e:       Error quaternion (q_DB)                    1x4
  // omega:     IMU angular velocity                       1x3
  // Kp:        Proportional gain matrix                   3x3
  // Kd:        Derivative gain matrix                     3x3
  // I:         Inertia tensor matrix about balence corner 3x3
  // FgW:       Gravity force vector (world frame)         1x3
  // r_COMB:    Vector relating corner to COM (body frame) 1x3
  //
  //OUTPUTS:
  // wheel_tau: Wheel torques to be sent to ESC            1x3
  
  //Breaking up error quaternion into q_e = [q0; qv];
  float q0 = q_e[0];
  float qv[3] = {q_e[1], q_e[2], q_e[3]};


  //proportional term
  float e_p[3];
    for (int i = 0; i < 3; i++) {
      e_p[i] = 0.0;               // initialize
      for (int j = 0; j < 3; j++) {
        e_p[i] += sign(q0) * Kp[i][j] * qv[j];
    }
  }

  //Deriv term
  float e_d[3];
    for (int i = 0; i < 3; i++) {
      e_d[i] = 0.0;               // initialize
      for (int j = 0; j < 3; j++) {
        e_d[i] += Kd[i][j] * omega[j];
    }
  }

  //Inertia comp, Iw term in the overall comp term w x Iw
  float w_I[3];
    for (int i = 0; i < 3; i++) {
      w_I[i] = 0.0;               // initialize
      for (int j = 0; j < 3; j++) {
        w_I[i] +=  I[i][j] * omega[j];
    }
  }
  //Final inertia comp term
  float inertia_comp[3];
  crossProduct(omega, w_I, inertia_comp);


  //Finding grav vector in body frame
  float FgB[3];
  quatRotate(q_BW, FgW, FgB);


  //Finding grav torque in body frame
  float tauGB[3];
  crossProduct(r_COMB, FgB, tauGB);

  //running 3 axis control law
  for (int i = 0; i < 3; i++){
    wheel_tau[i] = -1.0*e_p[i] - e_d[i] + w_I[i] - tauGB[i];
  }
  
}
