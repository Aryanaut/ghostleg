#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

/**
 * @brief Multiply two matrices: C = A * B
 * 
 * @param A First matrix (m x n)
 * @param B Second matrix (n x p)
 * @param C Result matrix (m x p)
 * @param m Number of rows in A and C
 * @param n Number of columns in A, number of rows in B
 * @param p Number of columns in B and C
 */
void matmul(const float* A, const float* B, float* C, int m, int n, int p);

/**
 * @brief Multiply a matrix and a vector: y = A * x
 * 
 * @param A Matrix (m x n)
 * @param x Vector (n x 1)
 * @param y Result vector (m x 1)
 * @param m Number of rows in A, length of y
 * @param n Number of columns in A, length of x
 */
void matvecmul(const float* A, const float* x, float* y, int m, int n);

/**
 * @brief Performs y = A*x + b
 * 
 * @param A Matrix (m x n)
 * @param x Vector (n x 1)
 * @param b Vector (m x 1)
 * @param y Result vector (m x 1)
 * @param m Number of rows in A, length of y and b
 * @param n Number of columns in A, length of x
 */
void matvecmul_add(const float* A, const float* x, const float* b, float* y, int m, int n);

/**
 * @brief Transpose a matrix: B = A^T
 * 
 * @param A Input matrix (m x n)
 * @param B Result matrix (n x m)
 * @param m Number of rows in A, number of columns in B
 * @param n Number of columns in A, number of rows in B
 */
void transpose(const float* A, float* B, int m, int n);

/**
 * @brief Multiply a matrix by its transpose: C = A * A^T
 * 
 * @param A Input matrix (m x n)
 * @param C Result matrix (m x m)
 * @param m Number of rows in A, dimensions of C
 * @param n Number of columns in A
 */
void matmul_transpose(const float* A, float* C, int m, int n);

/**
 * @brief Add two matrices: C = A + B
 * 
 * @param A First matrix (m x n)
 * @param B Second matrix (m x n)
 * @param C Result matrix (m x n)
 * @param m Number of rows
 * @param n Number of columns
 */
void matadd(const float* A, const float* B, float* C, int m, int n);

/**
 * @brief Subtract two matrices: C = A - B
 * 
 * @param A First matrix (m x n)
 * @param B Second matrix (m x n)
 * @param C Result matrix (m x n)
 * @param m Number of rows
 * @param n Number of columns
 */
void matsub(const float* A, const float* B, float* C, int m, int n);

/**
 * @brief Create identity matrix
 * 
 * @param A Output identity matrix (n x n)
 * @param n Dimensions of matrix
 */
void eye(float* A, int n);

/**
 * @brief Scale a matrix by a scalar: B = s * A
 * 
 * @param A Input matrix (m x n)
 * @param B Result matrix (m x n)
 * @param s Scalar value
 * @param m Number of rows
 * @param n Number of columns
 */
void matscale(const float* A, float* B, float s, int m, int n);

/**
 * @brief Enforce symmetry of a square matrix: A = (A + A^T)/2
 * 
 * @param A Matrix to symmetrize (n x n)
 * @param n Dimensions of matrix
 */
void symmetrize(float* A, int n);

/**
 * @brief Enforce positive definiteness by ensuring diagonal elements are positive
 * 
 * @param A Matrix to enforce (n x n)
 * @param n Dimensions of matrix
 * @param min_val Minimum value for diagonal elements (default 1e-9)
 */
void enforce_pd(float* A, int n, float min_val);

/**
 * @brief Compute eigenvectors and eigenvalues of a symmetric 3x3 matrix
 *
 * Uses the Jacobi eigenvalue algorithm for symmetric matrices.
 *
 * @param A Input 3x3 symmetric matrix
 * @param eigenvalues Output array for 3 eigenvalues
 * @param eigenvectors Output 3x3 matrix where each column is an eigenvector
 * @param max_iterations Maximum number of iterations (typically 50 is sufficient)
 * @return Number of iterations performed
 */
int symmetric_eigen_3x3(const float *A, float *eigenvalues, float *eigenvectors, int max_iterations);

#endif // MATRIX_UTILS_H