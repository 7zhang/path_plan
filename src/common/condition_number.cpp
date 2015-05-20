#include <stdlib.h>
#include <stdio.h>

/* DGESVD prototype */
extern "C" void dgesvd_( char* jobu, char* jobvt, int* m, int* n, double* a,
			 int* lda, double* s, double* u, int* ldu, double* vt, int* ldvt,
			 double* work, int* lwork, int* info );
/* Auxiliary routines prototypes */
extern void print_matrix( char* desc, int m, int n, double* a, int lda );

/* Parameters */
#define M 6
#define N 6
#define LDA M
#define LDU M
#define LDVT N

/* Main program */
double condition_number(double *a) {

	/* Locals */
	int m = M, n = N, lda = LDA, ldu = LDU, ldvt = LDVT, info, lwork;

//	print_matrix( "matrix a (stored columnwise)", m, n, a, lda);

//	double wkopt;
//	double* work;
	/* Local arrays */
	double s[N], u[LDU*M], vt[LDVT*N];

        // double a[LDA*M] = {
	// 	8.79,  9.93,  9.83, 5.45,  3.16,
	// 	6.11,  6.91,  5.04, -0.27,  7.98,
	// 	-9.15, -7.93,  4.86, 4.85,  3.01,
	// 	9.57,  1.64,  8.83, 0.74,  5.80,
	// 	-3.49,  4.02,  9.80, 10.00,  4.27,
	// 	9.84,  0.15, -8.99, -6.02, -5.31
	// };

	/* Executable statements */
//	printf( " DGESVD Example Program Results\n" );
	/* Query and allocate the optimal workspace */
	lwork = 500;
	double work[500];
//	dgesvd_( "All", "All", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, &wkopt, &lwork,
//		&info );
//	lwork = (int)wkopt;
//	printf("lwork = %d\n", lwork);
//	work = (double*)malloc( lwork*sizeof(double) );
	/* Compute SVD */
	dgesvd_( "N", "N", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, work, &lwork, &info);

	/* Check for convergence */
	if( info > 0 ) {
		printf( "The algorithm computing SVD failed to converge.\n" );
		return -1;
	}

	/* Print singular values */
//	print_matrix( "Singular values", 1, n, s, 1 );
	/* Print left singular vectors */
//	print_matrix( "Left singular vectors (stored columnwise)", m, n, u, ldu );
	/* Print right singular vectors */
//	print_matrix( "Right singular vectors (stored rowwise)", n, n, vt, ldvt );
	/* Free workspace */
//	free( (void*)work );
	if (s[0] != 0) {
		return s[N - 1] / s[0];
	} else {
		return 0;
	}
} /* End of DGESVD Example */

// int main() {
// 	double a[LDA*N] = {
// 			8.79,  6.11, -9.15,  9.57, -3.49,  9.84,
// 			9.93,  6.91, -7.93,  1.64,  4.02,  0.15,
// 			9.83,  5.04,  4.86,  8.83,  9.80, -8.99,
// 			5.45, -0.27,  4.85,  0.74, 10.00, -6.02,
// 			3.16,  7.98,  3.01,  5.80,  4.27, -5.31,
// 			2.16,  3.98,  10.01,  2.90,  7.7, 0.31
// 		};
// 	condition_number(a);
// 	return 0;
// }

/* Auxiliary routine: printing a matrix */
void print_matrix( char* desc, int m, int n, double* a, int lda ) {
	int i, j;
	printf( "\n %s\n", desc );
	for( i = 0; i < m; i++ ) {
//		for( j = 0; j < n; j++ ) printf( " %6.2f", a[i+j*lda] );
		for( j = 0; j < n; j++ ) printf( " %6.10f", a[i+j*lda] );
		printf( "\n" );
	}
}
