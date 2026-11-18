#include "matrix_utils.h"
#include <string.h>
#include <math.h>

// Matrix multiplication: C = A * B
void matmul(const float *A, const float *B, float *C, int m, int n, int p) {
  // For each element in the output matrix
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < p; j++) {
      C[i * p + j] = 0.0f;  // Initialize element to zero
      // Compute dot product of row i of A and column j of B
      for (int k = 0; k < n; k++) {
        C[i * p + j] += A[i * n + k] * B[k * p + j];
      }
    }
  }
}

// Matrix-vector multiplication: y = A * x
void matvecmul(const float *A, const float *x, float *y, int m, int n) {
  // For each element in the output vector
  for (int i = 0; i < m; i++) {
    y[i] = 0.0f;  // Initialize element to zero
    // Compute dot product of row i of A and vector x
    for (int j = 0; j < n; j++) {
      y[i] += A[i * n + j] * x[j];
    }
  }
}

// Matrix-vector multiplication with addition: y = A*x + b
void matvecmul_add(const float *A, const float *x, const float *b, float *y, int m, int n) {
  // For each element in the output vector
  for (int i = 0; i < m; i++) {
    y[i] = b[i];  // Start with the offset b
    // Add dot product of row i of A and vector x
    for (int j = 0; j < n; j++) {
      y[i] += A[i * n + j] * x[j];
    }
  }
}

// Matrix transpose: B = A^T
void transpose(const float *A, float *B, int m, int n) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      B[j * m + i] = A[i * n + j];
    }
  }
}

// Multiply a matrix by its transpose: C = A * A^T
void matmul_transpose(const float *A, float *C, int m, int n) {
  // For each element in the output matrix
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < m; j++) {
      C[i * m + j] = 0.0f;  // Initialize element to zero
      // Compute dot product of row i and row j of A
      for (int k = 0; k < n; k++) {
        C[i * m + j] += A[i * n + k] * A[j * n + k];
      }
    }
  }
}

// Add two matrices: C = A + B
void matadd(const float *A, const float *B, float *C, int m, int n) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      C[i * n + j] = A[i * n + j] + B[i * n + j];
    }
  }
}

// Subtract two matrices: C = A - B
void matsub(const float *A, const float *B, float *C, int m, int n) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      C[i * n + j] = A[i * n + j] - B[i * n + j];
    }
  }
}

// Create identity matrix
void eye(float *A, int n) {
  memset(A, 0, n * n * sizeof(float));  // Zero out the matrix
  for (int i = 0; i < n; i++) {
    A[i * n + i] = 1.0f;  // Set diagonal elements to 1
  }
}

// Scale a matrix: B = s * A
void matscale(const float *A, float *B, float s, int m, int n) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      B[i * n + j] = s * A[i * n + j];
    }
  }
}

// Enforce symmetry: A = (A + A^T)/2
void symmetrize(float *A, int n) {
  for (int i = 0; i < n; i++) {
    for (int j = i + 1; j < n; j++) {
      float avg = (A[i * n + j] + A[j * n + i]) * 0.5f;
      A[i * n + j] = A[j * n + i] = avg;
    }
  }
}

// Enforce positive definiteness using diagonal dominance
void enforce_pd(float *A, int n, float min_val) {
  // Find matrix scale (max diagonal element)
  float max_diag = 0.0f;
  for (int i = 0; i < n; i++) {
    if (!isnan(A[i * n + i]) && !isinf(A[i * n + i])) {
      max_diag = fmaxf(max_diag, fabsf(A[i * n + i]));
    }
  }

  // Use scale-dependent minimum value
  float scaled_min = fmaxf(min_val, max_diag * 1e-9f);

  // Enforce diagonal dominance with a safety margin
  for (int i = 0; i < n; i++) {
    // ensure positive diagonal
    if (isnan(A[i * n + i]) || isinf(A[i * n + i]) || A[i * n + i] < scaled_min) {
      A[i * n + i] = scaled_min;
    }
  }
}

int symmetric_eigen_3x3(const float *A, float *eigenvalues, float *eigenvectors, int max_iterations) {
  // Initialize eigenvectors to identity matrix
  eye(eigenvectors, 3);
  
  // Copy input matrix
  float matrix[9];
  memcpy(matrix, A, 9 * sizeof(float));
  
  int iter;
  float convergence_threshold = 1e-10f;
  
  for (iter = 0; iter < max_iterations; iter++) {
    // Find largest off-diagonal element
    float max_val = 0.0f;
    int p = 0, q = 1;
    for (int i = 0; i < 3; i++) {
      for (int j = i + 1; j < 3; j++) {
        float val = fabsf(matrix[i * 3 + j]);
        if (val > max_val) {
          max_val = val;
          p = i;
          q = j;
        }
      }
    }
    
    // Check for convergence
    if (max_val < convergence_threshold) {
      break;
    }
    
    // Compute rotation angle
    float a_pp = matrix[p * 3 + p];
    float a_qq = matrix[q * 3 + q];
    float a_pq = matrix[p * 3 + q];
    
    // Use a more robust formula for the rotation angle
    float tau = (a_qq - a_pp) / (2.0f * a_pq);
    float t = 1.0f / (fabsf(tau) + sqrtf(1.0f + tau * tau));
    if (tau < 0.0f) t = -t;
    
    float c = 1.0f / sqrtf(1.0f + t * t);
    float s = t * c;
    
    // Apply rotation to matrix
    for (int i = 0; i < 3; i++) {
      if (i != p && i != q) {
        float a_ip = matrix[i * 3 + p];
        float a_iq = matrix[i * 3 + q];
        matrix[i * 3 + p] = c * a_ip - s * a_iq;
        matrix[p * 3 + i] = matrix[i * 3 + p];  // Maintain symmetry
        matrix[i * 3 + q] = s * a_ip + c * a_iq;
        matrix[q * 3 + i] = matrix[i * 3 + q];  // Maintain symmetry
      }
    }
    
    // Update the diagonal elements correctly
    matrix[p * 3 + p] = c * c * a_pp + s * s * a_qq - 2.0f * c * s * a_pq;
    matrix[q * 3 + q] = s * s * a_pp + c * c * a_qq + 2.0f * c * s * a_pq;
    
    // Zero out the p,q elements
    matrix[p * 3 + q] = 0.0f;
    matrix[q * 3 + p] = 0.0f;
    
    // Update eigenvectors
    for (int i = 0; i < 3; i++) {
      float e_ip = eigenvectors[i * 3 + p];
      float e_iq = eigenvectors[i * 3 + q];
      eigenvectors[i * 3 + p] = c * e_ip - s * e_iq;
      eigenvectors[i * 3 + q] = s * e_ip + c * e_iq;
    }
  }
  
  // Extract eigenvalues from the diagonal
  eigenvalues[0] = matrix[0 * 3 + 0];
  eigenvalues[1] = matrix[1 * 3 + 1];
  eigenvalues[2] = matrix[2 * 3 + 2];
  
  // Make sure eigenvectors are normalized
  for (int i = 0; i < 3; i++) {
    float norm = 0.0f;
    for (int j = 0; j < 3; j++) {
      norm += eigenvectors[j * 3 + i] * eigenvectors[j * 3 + i];
    }
    norm = sqrtf(norm);
    // Avoid division by zero
    if (norm > 1e-10f) {
      for (int j = 0; j < 3; j++) {
        eigenvectors[j * 3 + i] /= norm;
      }
    }
  }
  
  return iter;
}