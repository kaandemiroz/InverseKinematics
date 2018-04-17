/*

USC/Viterbi/Computer Science
"Particle System" Assignment 2

Your name:
Osman Kaan Demiroz

*/

#include "ik.h"
#include "showGrid.h"
#include "input.h"

// particle parameters
const float step_time = 0.01f;

int num_attractors;
int num_repellers;

// camera parameters
double Theta = pi / 6;
double Phi = -pi / 2;
double R = 48;

double boxSize = 12;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton, g_iMiddleMouseButton, g_iRightMouseButton;

// number of images saved to disk so far
int sprite = 0;

bool mouseButtonHeld = false;

// these variables control what is displayed on screen
int pause = 0, box = 1, grid = 1, dots = 1, saveScreenToFile = 0;

struct point original_points[8][8][8];

int windowWidth, windowHeight;

float randomFloat()
{
	return (float)rand() / ((float)RAND_MAX + 1);
}

particle particles[NUM_PARTICLES];
point attractors[NUM_MAX_ATTRACTORS];
point repellers[NUM_MAX_REPELLERS];

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
	//glEnable(GL_POLYGON_SMOOTH);
	//glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);

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

point currentColor()
{
	point color;
	int colorGradient = 20;
	time_t timeValue = time(0);
	float hue = (timeValue % colorGradient) * (1.0f / colorGradient);
	point hsl = { hue, 0.8f, 0.6f };
	//printf("Hue: %f\n", hue);
	color = HSLToRGB(hsl);
	//printf("RGB: { %f, %f, %f }\n", color.x, color.y, color.z);
	return color;
}

float length(point p)
{
	return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

point closestElement(particle* p, point* points, int num_elements)
{
	int i;
	point closest, vector;
	float distance, shortestDistance;

	pDIFFERENCE(points[0], p->position, vector);
	closest = points[0];
	shortestDistance = length(vector);

	for (i = 1; i < num_elements; i++)
	{
		pDIFFERENCE(points[i], p->position, vector);
		distance = length(vector);
		if (distance < shortestDistance)
		{
			closest = points[i];
			shortestDistance = distance;
		}
	}

	return closest;
}

point closestAttractor(particle* p)
{
	int i;
	point closestAttractor, vector;
	float distance, shortestDistance;

	pDIFFERENCE(attractors[0], p->position, vector);
	closestAttractor = attractors[0];
	shortestDistance = length(vector);

	for (i = 1; i < num_attractors; i++)
	{
		pDIFFERENCE(attractors[i], p->position, vector);
		distance = length(vector);
		if (distance < shortestDistance)
		{
			closestAttractor = attractors[i];
			shortestDistance = distance;
		}
	}

	return closestAttractor;
}

point closestRepeller(particle *p)
{
	int i;
	point closestRepeller, vector;
	float distance, shortestDistance;

	pDIFFERENCE(repellers[0], p->position, vector);
	closestRepeller = repellers[0];
	shortestDistance = length(vector);

	for (i = 1; i < num_repellers; i++)
	{
		pDIFFERENCE(repellers[i], p->position, vector);
		distance = length(vector);
		if (distance < shortestDistance)
		{
			closestRepeller = repellers[i];
			shortestDistance = distance;
		}
	}

	return closestRepeller;
}

void initParticle(particle* p)
{
	point attractor;
	float spread = 4, size = 2 * boxSize;

	if (num_attractors > 0)
	{
		float r = randomFloat() * spread;
		float phi = randomFloat() * 2 * pi;
		float theta = randomFloat() * pi;

		attractor = closestAttractor(p);
		p->position = { attractor.x + r * sinf(phi) * cosf(theta),
						attractor.y + r * sinf(phi) * sinf(theta),
						attractor.z + r * cosf(phi) };
	}
	else
	{
		p->position = { size * randomFloat() - size / 2,
						size * randomFloat() - size / 2,
						size * randomFloat() - size / 2 };
	}

	p->velocity = { 0.0f, 0.0f, 0.0f };

	p->color = currentColor();
	p->timeAlive = 0;
	p->lifeSpan = randomFloat() + 1;
}

point mousePosTo3D(int mouseX, int mouseY)
{
	float x, y, length, size = R * 0.58f;;
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

void deleteLastAttractor()
{
	if (num_attractors > 0)
	{
		num_attractors--;
		printf("Attractors: %d\n", num_attractors);
	}
}

void deleteLastRepeller()
{
	if (num_repellers > 0)
	{
		num_repellers--;
		printf("Repellers: %d\n", num_repellers);
	}
}

void updateAttractors()
{
	if (g_iLeftMouseButton)
	{
		if (!mouseButtonHeld && num_attractors < NUM_MAX_ATTRACTORS)
		{
			attractors[num_attractors] = mousePosTo3D(g_vMousePos[0], g_vMousePos[1]);
			num_attractors++;
			printf("Attractors: %d\n", num_attractors);

			mouseButtonHeld = true;
		}
	}
	else if (g_iMiddleMouseButton)
	{
		if (!mouseButtonHeld && num_repellers < NUM_MAX_REPELLERS)
		{
			repellers[num_repellers] = mousePosTo3D(g_vMousePos[0], g_vMousePos[1]);
			num_repellers++;
			printf("Repellers: %d\n", num_repellers);

			mouseButtonHeld = true;
		}
	}
	else
	{
		mouseButtonHeld = false;
	}
}

void advanceParticles()
{
	int i;
	float r, phi, theta, speed = 3, length, spread = 6;
	point temp, dir;

	for (i = 0; i < NUM_PARTICLES; i++)
	{
		particle *p = &particles[i];
		pMULTIPLY(p->velocity, step_time, temp);
		pSUM(p->position, temp, p->position);

		if (num_attractors > 0)
		{
			temp = closestAttractor(p);
			pDIFFERENCE(temp, p->position, dir);
			//pNORMALIZE(dir);

			//if (length < spread)
			{
				r = randomFloat() * 0.5f;
				phi = randomFloat() * 2 * pi;
				theta = randomFloat() * pi;

				p->velocity = { speed * dir.x + r * sinf(phi) * cosf(theta),
								speed * dir.y + r * sinf(phi) * sinf(theta),
								speed * dir.z + r * cosf(phi) };
			}

		}

		if (num_repellers > 0)
		{
			temp = closestRepeller(p);
			pDIFFERENCE(p->position, temp, dir);
			pNORMALIZE(dir);

			if (length < spread)
			{
				float mag = speed * (spread - length);
				pMULTIPLY(dir, mag, dir);
				pSUM(p->velocity, dir, p->velocity);
			}
		}

		p->timeAlive += step_time;
		if (p->timeAlive > p->lifeSpan) {
			initParticle(p);
		}
	}
}

void initParticleSystem()
{
	int i;

	num_attractors = 0;
	num_repellers = 0;

	for (i = 0; i < NUM_PARTICLES; i++)
	{
		initParticle(&particles[i]);
	}
	for (i = 0; i < 2 / step_time; i++)
	{
		advanceParticles();
	}
}

void showParticles()
{
	int i;
	float alpha;
	particle *p;

	glBegin(GL_POINTS);

	for (i = 0; i < NUM_PARTICLES; i++)
	{
		p = &particles[i];
		alpha = 0.7f - p->timeAlive / p->lifeSpan;
		glColor4f(p->color.x, p->color.y, p->color.z, alpha < 0 ? -alpha : alpha);
		glVertex3f(p->position.x, p->position.y, p->position.z);
	}

	glEnd();
}

void showDots()
{
	glBegin(GL_POINTS);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	for (int i = 0; i < num_attractors; i++)
	{
		glVertex3f(attractors[i].x, attractors[i].y, attractors[i].z);
	}
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	for (int i = 0; i < num_repellers; i++)
	{
		glVertex3f(repellers[i].x, repellers[i].y, repellers[i].z);
	}
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
	showParticles();

	glutSwapBuffers();
}

void doIdle()
{
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
		updateAttractors();
		advanceParticles();
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
	glutCreateWindow("Particle System");

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
	initParticleSystem();

	/* forever sink in the black hole */
	glutMainLoop();

	return(0);
}

