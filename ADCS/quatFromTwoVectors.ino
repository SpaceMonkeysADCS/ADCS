void quatFromVec(float v1[3], float v2[3], float q[4]){
  //Max Howerter
  //This function computes the quaternion describing rotation from one vector to another vector
  float c[3] = {};
  crossProduct(v1, v2, c);
  float dot = dotProduct(v1, v2);

  q[0] = 1 + dot;
  q[1] = c[0];
  q[2] = c[1];
  q[3] = c[2];

  normalizeQuat(q);

}