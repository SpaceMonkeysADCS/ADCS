void quatFromTwoVectors(float v1[3], float v2[3], float q[4]) {
  //Max Howerter 3/26/2026
  //
  //This function finds the quaternion that represents the rotation from vector 1 to vector 2
  //
  //INPUTS:
  // v1: vector 1            1x3
  // v2: vector 2            1x3
  //
  //OUTPUTS:
  // q:  quaternion (1 -> 2) 1x4
  
  // Normalize v1
  float norm1 = sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
  float a[3] = {v1[0]/norm1, v1[1]/norm1, v1[2]/norm1};

  // Normalize v2
  float norm2 = sqrt(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]);
  float b[3] = {v2[0]/norm2, v2[1]/norm2, v2[2]/norm2};

  // Cross product
  float cross[3] = {
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0]
  };

  // Dot product
  float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];

  // Handle opposite vectors (180° rotation)
  if (dot < -0.999999f) {
    float axis[3];

    // Find orthogonal axis
    if (fabs(a[0]) < fabs(a[1])) {
      axis[0] = 0;
      axis[1] = -a[2];
      axis[2] = a[1];
    } else {
      axis[0] = -a[2];
      axis[1] = 0;
      axis[2] = a[0];
    }

    // Normalize axis
    float normAxis = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    axis[0] /= normAxis;
    axis[1] /= normAxis;
    axis[2] /= normAxis;

    // Quaternion for 180° rotation → scalar = 0
    q[0] = axis[0];
    q[1] = axis[1];
    q[2] = axis[2];
    q[3] = 0.0f;
    return;
  }

  // General case
  q[0] = cross[0];
  q[1] = cross[1];
  q[2] = cross[2];
  q[3] = 1.0f + dot;

  // Normalize quaternion
  float normQ = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] /= normQ;
  q[1] /= normQ;
  q[2] /= normQ;
  q[3] /= normQ;
}
