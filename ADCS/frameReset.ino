void frameReset(float qBW[4], float q0[4], float qTilt[4]){
  //Max Howerter
  //This function resets start frame to a frame that is always alligned with gravity
 
  float temp1[4];
  float temp2[4];
  float qTilt_inv[4];
  float q0_inv[4];
  quatINV(qTilt, qTilt_inv);
  quatINV(q0, q0_inv);


  quatMult(q0_inv, qTilt_inv, temp1);
  quatMult(qBW, temp1, temp2);

  qBW[0] = temp2[0];
  qBW[1] = temp2[1];
  qBW[2] = temp2[2];
  qBW[3] = temp2[3];
}