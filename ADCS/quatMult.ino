void quatMult(float q1[4], float q2[4], float result[4]){
  //Max Howerter 3/26/2026
  //
  //This function performs quaternion multiplication
  //
  //INPUTS:
  // q1: quaternion 1 4x1
  // q2: quaternion 2 4x1
  // 
  //OUTPUTS:
  // result: q1⊗q2    4x1

  //Indexing terms for visualization
  float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

  //Calculating
  result[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
  result[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
  result[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
  result[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}
