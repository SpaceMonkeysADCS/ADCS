void norm(float grav[3]){
  float norm = sqrt(grav[0]*grav[0] + grav[1]*grav[1] + grav[2]*grav[2]);

  grav[0] = grav[0]/norm;
  grav[1] = grav[1]/norm;
  grav[2] = grav[2]/norm;

 
}