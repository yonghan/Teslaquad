/*
 * AHRS_math.h
 *
 *  Created on: Mar 8, 2012
 *      Author: yonghan
 */

#ifndef AHRS_MATH_H_
#define AHRS_MATH_H_

#include "structs.h"

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

double vector_DotProduct(vector_t *v1, vector_t *v2);
vector_t vector_CrossProduct(vector_t *v1, vector_t *v2);
vector_t vector_Scale(vector_t *vIn, double scale);
vector_t vector_Add(vector_t *vIn1, vector_t *vIn2);
matrix33_t matrix_Multiply(matrix33_t *a, matrix33_t *b);
matrix33_t init_RotationMatrix(eulerAngle_t *e);
double absd(double a);
double constrain(double a, double min, double max);

#endif /* AHRS_MATH_H_ */
