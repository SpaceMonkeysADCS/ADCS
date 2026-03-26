void errorQuaternion(float q_BW[4], float q_DW[4], float q_e[4]){
  //Max Howerter 3/24/2026
  //
  //This function computes the error quaternion from two quaternions
  //
  //INPUTS:
  // q_BW: quaternion (world -> body)        1x4
  // q_DW: quaternion (world -> desired)     1x4
  //
  //OUTPUTS:
  // q_e: error quaternion (body -> desired) 1x4

  
  //Normalize both Quaternions
  float mag_BW = sqrt(q_BW[0] * q_BW[0] + q_BW[1] * q_BW[1] + q_BW[2] * q_BW[2] + q_BW[3] * q_BW[3]);
  float mag_DW = sqrt(q_DW[0] * q_DW[0] + q_DW[1] * q_DW[1] + q_DW[2] * q_DW[2] + q_DW[3] * q_DW[3]);

  //Normalizing both quaternions
  q_BW[0] = q_BW[0]/mag_BW;
  q_BW[1] = q_BW[1]/mag_BW;
  q_BW[2] = q_BW[2]/mag_BW;
  q_BW[3] = q_BW[3]/mag_BW;

  q_DW[0] = q_DW[0]/mag_DW;
  q_DW[1] = q_DW[1]/mag_DW;
  q_DW[2] = q_DW[2]/mag_DW;
  q_DW[3] = q_DW[3]/mag_DW;

  //initializing inverse quaternion
  float q_WB[4];
  q_WB[0] = q_BW[0];
  q_WB[1] = -q_BW[1];
  q_WB[2] = -q_BW[2];
  q_WB[3] = -q_BW[3];

  quatMult(q_DW, q_WB, q_e);
  
}
