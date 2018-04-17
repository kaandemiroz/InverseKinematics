#ifndef _IK_H_
#define _IK_H_

#ifdef _WIN32
	#include <windows.h>
#endif

#include <stdio.h>
#include <ctime>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "openGL-headers.h"
#include "pic.h"

#define pi 3.141592653589793238462643383279 

#define NUM_PARTICLES 10000
#define NUM_MAX_ATTRACTORS 50
#define NUM_MAX_REPELLERS 50

// camera angles
extern double Theta;
extern double Phi;
extern double R;

extern double boxSize;

// number of images saved to disk so far
extern int sprite;

// mouse control
extern int g_vMousePos[2];
extern int g_iLeftMouseButton, g_iMiddleMouseButton, g_iRightMouseButton;

void deleteLastAttractor(), deleteLastRepeller();

struct point
{
	float x;
	float y;
	float z;
};

// these variables control what is displayed on the screen
extern int pause, box, grid, dots, saveScreenToFile;

struct particle
{
	struct point position;
	struct point velocity;
	struct point color;
	float timeAlive;
	float lifeSpan;
};

// computes crossproduct of three vectors, which are given as points
// struct point vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1,vector2,dest)\
  CROSSPRODUCT( (vector1).x, (vector1).y, (vector1).z,\
                (vector2).x, (vector2).y, (vector2).z,\
                (dest).x, (dest).y, (dest).z )

// computes crossproduct of three vectors, which are specified by floating-point coordinates
// double x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)

// normalizes vector dest
// struct point dest
// result returned in dest
// must declare a double variable called 'length' somewhere inside the scope of the NORMALIZE macro
// macro will change that variable
#define pNORMALIZE(dest)\
\
  length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);\
  (dest).x /= length;\
  (dest).y /= length;\
  (dest).z /= length;

#define pDOT(v1, v2, dest)\
\
  dest = ((v1).x * (v2).x + (v1).y * (v2).y + (v1).z * (v2).z)

// copies vector source to vector dest
// struct point source,dest
#define pCPY(source,dest)\
\
  (dest).x = (source).x;\
  (dest).y = (source).y;\
  (dest).z = (source).z;

// assigns values x,y,z to point vector dest
// struct point dest
// double x,y,z
#define pMAKE(x,y,z,dest)\
\
  (dest).(x) = (x);\
  (dest).(y) = (y);\
  (dest).(z) = (z);

// sums points src1 and src2 to dest
// struct point src1,src2,dest
#define pSUM(src1,src2,dest)\
\
  (dest).x = (src1).x + (src2).x;\
  (dest).y = (src1).y + (src2).y;\
  (dest).z = (src1).z + (src2).z;

// dest = src1 - src2
// struct point src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
  (dest).x = (src1).x - (src2).x;\
  (dest).y = (src1).y - (src2).y;\
  (dest).z = (src1).z - (src2).z;

// mulitplies components of point src by scalar and returns the result in dest
// struct point src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
  (dest).x = (src).x * (scalar);\
  (dest).y = (src).y * (scalar);\
  (dest).z = (src).z * (scalar);

#endif