/*
 * AHRS_math.c
 *
 *  Created on: Mar 8, 2012
 *      Author: yonghan
 */

/* This file is part of the Razor AHRS Firmware */

#include <math.h>
#include "AHRS_math.h"

// Computes the dot product of two vectors
double vector_DotProduct(vector_t *v1, vector_t *v2)
{
	return v1->e0 * v2->e0 +
		   v1->e1 * v2->e1 +
		   v1->e2 * v2->e2;
}

// Computes the cross product of two vectors
vector_t vector_CrossProduct(vector_t *v1, vector_t *v2)
{
	vector_t vOut;

	vOut.e0 = (v1->e1 * v2->e2) - (v1->e2 * v2->e1);
	vOut.e1 = (v1->e2 * v2->e0) - (v1->e0 * v2->e2);
	vOut.e2 = (v1->e0 * v2->e1) - (v1->e1 * v2->e0);

	return vOut;
}

// Multiply the vector by a scalar.
vector_t vector_Scale(vector_t *vIn, double scale)
{
	vector_t vOut;

	vOut.e0 = scale * vIn->e0;
	vOut.e1 = scale * vIn->e1;
	vOut.e2 = scale * vIn->e2;

	return vOut;
}

// Adds two vectors
vector_t vector_Add(vector_t *vIn1, vector_t *vIn2)
{
	vector_t vOut;

	vOut.e0 = vIn1->e0 + vIn2->e0;
	vOut.e1 = vIn1->e1 + vIn2->e1;
	vOut.e2 = vIn1->e2 + vIn2->e2;

	return vOut;
}

//Multiply two 3x3 matrixes
matrix33_t matrix_Multiply(matrix33_t *a, matrix33_t *b)
{
	matrix33_t mOut;

	mOut.m0.e0 = a->m0.e0 * b->m0.e0 + a->m0.e1 * b->m1.e0 + a->m0.e2 * b->m2.e0;
	mOut.m0.e1 = a->m0.e0 * b->m0.e1 + a->m0.e1 * b->m1.e1 + a->m0.e2 * b->m2.e1;
	mOut.m0.e2 = a->m0.e0 * b->m0.e2 + a->m0.e1 * b->m1.e2 + a->m0.e2 * b->m2.e2;
	mOut.m1.e0 = a->m1.e0 * b->m0.e0 + a->m1.e1 * b->m1.e0 + a->m1.e2 * b->m2.e0;
	mOut.m1.e1 = a->m1.e0 * b->m0.e1 + a->m1.e1 * b->m1.e1 + a->m1.e2 * b->m2.e1;
	mOut.m1.e2 = a->m1.e0 * b->m0.e2 + a->m1.e1 * b->m1.e2 + a->m1.e2 * b->m2.e2;
	mOut.m1.e0 = a->m2.e0 * b->m0.e0 + a->m2.e1 * b->m1.e0 + a->m2.e2 * b->m2.e0;
	mOut.m1.e1 = a->m2.e0 * b->m0.e1 + a->m2.e1 * b->m1.e1 + a->m2.e2 * b->m2.e1;
	mOut.m1.e2 = a->m2.e0 * b->m0.e2 + a->m2.e1 * b->m1.e2 + a->m2.e2 * b->m2.e2;

	return mOut;
}

// Init rotation matrix using euler angles
matrix33_t init_RotationMatrix(eulerAngle_t *e)
{
	matrix33_t mOut;
	double c1 = cos(e->Roll);
	double s1 = sin(e->Roll);
	double c2 = cos(e->Pitch);
	double s2 = sin(e->Pitch);
	double c3 = cos(e->Yaw);
	double s3 = sin(e->Yaw);


	// Euler angles, right-handed, intrinsic, XYZ convention
	// (which means: rotate around body axes Z, Y', X'')
	mOut.m0.e0 = c2 * c3;
	mOut.m0.e1 = c3 * s1 * s2 - c1 * s3;
	mOut.m0.e2 = s1 * s3 + c1 * c3 * s2;

	mOut.m1.e0 = c2 * s3;
	mOut.m1.e1 = c1 * c3 + s1 * s2 * s3;
	mOut.m1.e2 = c1 * s2 * s3 - c3 * s1;

	mOut.m2.e0 = -s2;
	mOut.m2.e1 = c2 * s1;
	mOut.m2.e2 = c1 * c2;

	return mOut;
}

double absd(double a)
{
	if (a > 0.0) return a;
	else if (a < 0.0) return -a;
	else return 0;
}

double constrain(double a, double min, double max)
{
	if (a < min) return min;
	else if (a > max) return max;
	else return a;
}
