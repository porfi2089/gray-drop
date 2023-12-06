#include <functions.h>

float Calculate_Transformation_Matrix(float roll, float yaw, float pitch, int g, int p){

  // Define the rotation matrices
  float R_roll[3][3] = {{1, 0, 0},
                        {0, cos(roll), -sin(roll)},
                        {0, sin(roll), cos(roll)}};

  float R_pitch[3][3] = {{cos(pitch), 0, sin(pitch)},
                         {0, 1, 0},
                         {-sin(pitch), 0, cos(pitch)}};

  float R_yaw[3][3] = {{cos(yaw), -sin(yaw), 0},
                       {sin(yaw), cos(yaw), 0},
                       {0, 0, 1}};

  // Multiply the rotation matrices to get the overall transformation matrix
  float R[3][3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        R[i][j] += R_roll[i][k] * R_pitch[k][j];
      }
    }
  }

  float R_total[3][3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        R_total[i][j] += R[i][k] * R_yaw[k][j];
      }
    }
  }

  // Return the matrix by parts
    return R_total[g][p];
  
}