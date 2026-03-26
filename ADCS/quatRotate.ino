void quatRotate(float q[4],float v[3], float v_rot[3]){
  //Max Howerter 3/26/2026
  //This function inputs a quaternion, and a vector and will calculate the new vector that has been rotated by the given quaternion
  //INPUTS:
  //
  // q: quaternion to rotate by 4x1
  // v: vector to be rotated    3x1
  // 
  //OUTPUTS:
  // v_rot: rotated vector      3x1

  //turning vector into correct format for the qvq^-1 formula
  float v_q[4] = {0.0, v[0], v[1], v[2]};

  //Initializing
  float q_inv[4];
  float temp[4];
  float result[4];

  //Calculating q^-1
  quatINV(q, q_inv);

  //Calculating temp = q*v
  quatMult(q, v_q, temp);

  //Calculating qvq^-1
  quatMult(temp, q_inv, result);

  //Exporting result into desired format
  v_rot[0] = result[1];
  v_rot[1] = result[2];
  v_rot[2] = result[3];
}
