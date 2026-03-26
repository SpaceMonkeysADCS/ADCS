void quatINV(float q[4], float q_inv[4]){
  //Max Howerter 3/26/2026
  //
  //This function calculates the inverse quaternion
  //
  //INPUTS:
  // q: quaternion 4x1
  //
  //OUTPUTS:
  // q_inv: q^-1   4x1
  
  q_inv[0] = q[0];
  q_inv[1] = -q[1];
  q_inv[2] = -q[2];
  q_inv[3] = -q[3];
}
