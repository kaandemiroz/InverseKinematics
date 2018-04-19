/*

USC/Viterbi/Computer Science
"Inverse Kinematics" Assignment 3

Your name:
Osman Kaan Demiroz

*/

#include "ik.h"
#include "showGrid.h"
#include "input.h"

// camera parameters
double Theta = -pi / 24;
double Phi = -pi / 2 - pi / 24;
double R = 12;

double boxSize = 12;

// mouse control
int g_vMousePos[2];
int g_iLeftMouseButton, g_iMiddleMouseButton, g_iRightMouseButton;

// number of images saved to disk so far
int sprite = 0;

bool mouseButtonHeld = false;

// these variables control what is displayed on screen
int pause = 0, box = 1, grid = 1, dots = 1, saveScreenToFile = 0;

int windowWidth, windowHeight;

float randomFloat()
{
	return (float)rand() / ((float)RAND_MAX + 1);
}

bone skeleton[NUM_MAX_BONES];

point origin = { 0.0f, 0.0f, 0.0f }, target = origin;
int numBones = NUM_MIN_BONES;

void myinit()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, 1.0, 0.01, 1000.0);

	// set background color to black
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	//glEnable(GL_POINT_SMOOTH);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	return;
}

float HueToRGB(float v1, float v2, float vH)
{
	if (vH < 0) vH += 1;
	if (vH > 1) vH -= 1;
	if ((6 * vH) < 1) return (v1 + (v2 - v1) * 6 * vH);
	if ((2 * vH) < 1) return v2;
	if ((3 * vH) < 2) return (v1 + (v2 - v1) * ((2.0f / 3) - vH) * 6);

	return v1;
}

point HSLToRGB(point hsl) {
	point rgb;

	if (hsl.y == 0)
	{
		rgb.x = rgb.y = rgb.z = (hsl.z * 255);
	}
	else
	{
		float v1, v2;

		v2 = (hsl.z < 0.5) ? (hsl.z * (1 + hsl.y)) : ((hsl.z + hsl.y) - (hsl.z * hsl.y));
		v1 = 2 * hsl.z - v2;

		rgb.x = (HueToRGB(v1, v2, hsl.x + (1.0f / 3)));
		rgb.y = (HueToRGB(v1, v2, hsl.x));
		rgb.z = (HueToRGB(v1, v2, hsl.x - (1.0f / 3)));
	}

	return rgb;
}

float length(point p)
{
	return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

void initBone(bone* b, point base, point effector, bone* parent = NULL)
{
	b->base = base;
	b->effector = effector;

	if (parent)
	{
		b->parent = parent;
		parent->child = b;
	}
}

// Converts 2D mouse position to a 3D point
//
// The depth of the point is set to be on the plane that is 
// perpendicular to camera eye vector and intersects with origin (0, 0, 0)
point mousePosTo3D(int mouseX, int mouseY)
{
	float x, y, length, size = R * 0.58f;
	point up, right, xPrime, yPrime, result;
	float aspectRatio = (float)windowWidth / windowHeight;

	y = -((float)mouseY / windowHeight * 2 * size - size);
	size *= aspectRatio;
	x = ((float)mouseX / windowWidth * 2 * size - size);

	point plane = { R * cos(Phi) * cos(Theta),
					R * sin(Phi) * cos(Theta),
					R * sin(Theta) };

	point v = { R * cos(Phi) * cos(Theta),
				R * sin(Phi) * cos(Theta),
				0 };

	if (Theta > 0)
	{
		CROSSPRODUCTp(plane, v, right);
	}
	else
	{
		CROSSPRODUCTp(v, plane, right);
	}

	CROSSPRODUCTp(plane, right, up);
	pNORMALIZE(right);
	pNORMALIZE(up);

	pMULTIPLY(right, x, xPrime);
	pMULTIPLY(up, y, yPrime);

	pSUM(xPrime, yPrime, result);

	return result;
}

// Initialize the skeleton with NUM_BONES bones
// Skeleton starts at origin, and every bone extends it by vector
void initSkeleton(point origin, point vector)
{
	int i;
	point base, effector;

	base = origin;
	pSUM(base, vector, effector);

	initBone(&skeleton[0], base, effector);

	for (i = 1; i < numBones; i++)
	{
		base = effector;
		pSUM(base, vector, effector);
		initBone(&skeleton[i], base, effector, &skeleton[i - 1]);
	}

	target = skeleton[numBones - 1].effector;
}

// Solve IK using the FABRIK method
// (Forwards And Backwards Reaching Inverse Kinematics)
// based on the target position to move the arm to
void solveIK(point origin, point target)
{
	int i, n;
	float len;
	point *base, *effector, vector;

	printf("Target: { %f, %f, %f }\n", target.x, target.y, target.z);

	for (n = 0; n < NUM_ITERATIONS; n++)
	{
		// STAGE 1: Move arm to target
		skeleton[numBones - 1].effector = target;
		for (i = numBones - 1; i > 0; i--)
		{
			base = &skeleton[i].base;
			effector = &skeleton[i].effector;

			pDIFFERENCE(*base, *effector, vector);
			len = length(vector);
			if (len != 0)
			{
				pMULTIPLY(vector, 1 / len, vector);
			}
			pSUM(*effector, vector, *base);

			if (i > 0) skeleton[i - 1].effector = *base;
		}

		// STAGE 2: Snap arm back to origin
		skeleton[0].base = origin;
		for (i = 0; i < numBones; i++)
		{
			base = &skeleton[i].base;
			effector = &skeleton[i].effector;

			pDIFFERENCE(*effector, *base, vector);
			len = length(vector);
			if (len != 0)
			{
				pMULTIPLY(vector, 1 / len, vector);
			}
			pSUM(*base, vector, *effector);

			if (i < numBones - 1) skeleton[i + 1].base = *effector;
		}

		pDIFFERENCE(target, skeleton[numBones - 1].effector, vector);
		printf("Iteration: %d, Error: %f\n", n, length(vector));
	}
}

void incrementBones()
{
	point base, effector, vector;

	if (numBones < NUM_MAX_BONES)
	{
		base = skeleton[numBones - 1].base;
		effector = skeleton[numBones - 1].effector;
		pDIFFERENCE(effector, base, vector);

		base = skeleton[numBones - 1].effector;
		pSUM(base, vector, effector);

		target = effector;
		initBone(&skeleton[numBones], base, effector, &skeleton[numBones - 1]);
		numBones++;
		solveIK(origin, target);
	}
}

void decrementBones()
{
	if (numBones > NUM_MIN_BONES)
	{
		numBones--;
		solveIK(origin, target);
	}
}

int getZeroOneColorValue(int i, int size)
{
	return (int)roundf((float)(i % size) / size);
}

void drawSkeleton()
{
	int i, red, green, blue;
	bone *b;

	glBegin(GL_LINES);

	for (i = 0; i < numBones; i++)
	{
		red = getZeroOneColorValue(i + 2 * numBones / 3, numBones);
		green = getZeroOneColorValue(i + numBones / 3, numBones);
		blue = getZeroOneColorValue(i, numBones);

		glColor4f(red, green, blue, 1.0f);

		b = &skeleton[i];
		glVertex3f(b->base.x, b->base.y, b->base.z);
		glVertex3f(b->effector.x, b->effector.y, b->effector.z);
	}

	glEnd();
}

// Show tiny dots for origin and target positions
void showDots()
{
	glBegin(GL_POINTS);

	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glVertex3f(origin.x, origin.y, origin.z);

	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glVertex3f(target.x, target.y, target.z);

	glEnd();
}

void reshape(int w, int h)
{
	// Prevent a divide by zero, when h is zero.
	// You can't make a window of zero height.
	if (h == 0)
		h = 1;

	glViewport(0, 0, w, h);

	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the perspective
	double aspectRatio = 1.0 * w / h;
	gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	windowWidth = w;
	windowHeight = h;

	glutPostRedisplay();
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// camera parameters are Phi, Theta, R
	gluLookAt(R * cos(Phi) * cos(Theta), R * sin(Phi) * cos(Theta), R * sin(Theta),
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0);


	/* Lighting */
	/* You are encouraged to change lighting parameters or make improvements/modifications
	to the lighting model .
	This way, you will personalize your assignment and your assignment will stick out.
	*/

	// global ambient light
	GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };

	// light 's ambient, diffuse, specular
	GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

	GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
	GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

	GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
	GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

	GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
	GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

	GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
	GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

	GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
	GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

	GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

	GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
	GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

	// light positions and directions
	GLfloat lP0[] = { -1.999f, -1.999f, -1.999f, 1.0f };
	GLfloat lP1[] = { 1.999f, -1.999f, -1.999f, 1.0f };
	GLfloat lP2[] = { 1.999f, 1.999f, -1.999f, 1.0f };
	GLfloat lP3[] = { -1.999f, 1.999f, -1.999f, 1.0f };
	GLfloat lP4[] = { -1.999f, -1.999f, 1.999f, 1.0f };
	GLfloat lP5[] = { 1.999f, -1.999f, 1.999f, 1.0f };
	GLfloat lP6[] = { 1.999f, 1.999f, 1.999f, 1.0f };
	GLfloat lP7[] = { -1.999f, 1.999f, 1.999f, 1.0f };

	// jelly material color

	GLfloat mKa[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat mKd[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat mKs[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mKe[] = { 0.0f, 0.0f, 0.0f, 1.0f };

	/* set up lighting */
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	// set up cube color
	glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
	glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
	glMaterialf(GL_FRONT, GL_SHININESS, 120);

	// macro to set up light i
#define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)

	LIGHTSETUP(0);
	LIGHTSETUP(1);
	LIGHTSETUP(2);
	LIGHTSETUP(3);
	LIGHTSETUP(4);
	LIGHTSETUP(5);
	LIGHTSETUP(6);
	LIGHTSETUP(7);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	// show the cube
	//showCube(&jello);

	glDisable(GL_LIGHTING);

	// show the bounding box
	if (box) showBoundingBox();
	if (grid) showGrid();
	if (dots) showDots();
	drawSkeleton();

	glutSwapBuffers();
}

void doIdle()
{
	point p;
	char s[20] = "picxxxx.ppm";

	// save screen to file
	s[3] = 48 + (sprite / 1000);
	s[4] = 48 + (sprite % 1000) / 100;
	s[5] = 48 + (sprite % 100) / 10;
	s[6] = 48 + sprite % 10;

	if (saveScreenToFile == 1)
	{
		saveScreenshot(windowWidth, windowHeight, s);
		saveScreenToFile = 0; // save only once, change this if you want continuos image generation (i.e. animation)
		sprite++;
	}

	if (sprite >= 300) // allow only 300 snapshots
	{
		exit(0);
	}

	if (pause == 0)
	{
		if (g_iLeftMouseButton)
		{
			p = mousePosTo3D(g_vMousePos[0], g_vMousePos[1]);
			if (target.x != p.x || target.y != p.y || target.z != p.z)
			{
				target = p;
				solveIK(origin, target);
			}
		}
		else if (g_iMiddleMouseButton)
		{
			p = mousePosTo3D(g_vMousePos[0], g_vMousePos[1]);
			if (origin.x != p.x || origin.y != p.y || origin.z != p.z)
			{
				origin = p;
				solveIK(origin, target);
			}
		}
	}

	glutPostRedisplay();
}

int main(int argc, char ** argv)
{
	srand((unsigned int)time(0));

	glutInit(&argc, argv);

	/* double buffered window, use depth testing, 640x480 */
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	windowWidth = 640;
	windowHeight = 480;
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Inverse Kinematics");

	/* tells glut to use a particular display function to redraw */
	glutDisplayFunc(display);

	/* replace with any animate code */
	glutIdleFunc(doIdle);

	/* callback for mouse drags */
	glutMotionFunc(mouseMotionDrag);

	/* callback for window size changes */
	glutReshapeFunc(reshape);

	/* callback for mouse movement */
	glutPassiveMotionFunc(mouseMotion);

	/* callback for mouse button changes */
	glutMouseFunc(mouseButton);

	/* register for keyboard events */
	glutKeyboardFunc(keyboardFunc);

	///* register for special key events */
	//glutSpecialFunc(specialFunc);

	/* do initialization */
	myinit();
	initSkeleton(origin, { 0.0f, 0.0f, -1.0f });

	/* forever sink in the black hole */
	glutMainLoop();

	return(0);
}

