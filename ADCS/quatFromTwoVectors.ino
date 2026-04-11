void quatFromVec(float v1[3], float v2[3], float q[4]){
  //Max Howerter
  //This function computes the quaternion describing rotation from one vector to another vector
  float c[3] = {};
  float dot = 0.0f;
  crossProduct(v1, v2, c);
  dotProduct(v1, v2, &dot);

  q[0] = 1 + dot;
  q[1] = c[0];
  q[2] = c[1];
  q[3] = c[2];

  normalizeQuat(q);

}