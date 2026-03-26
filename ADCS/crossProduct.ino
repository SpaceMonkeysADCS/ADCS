void crossProduct(const float a[3], const float b[3], float result[3]) {
  //Max Howerter
  //
  //This fucntion calculates the cross product of two 1x3 vectors
  //
  //INPUTS:
  // a: vector1 1x3
  // b: vector2 1x3
  //
  //OUTPUTS:
  // result: a cross b 1x3
  
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}
