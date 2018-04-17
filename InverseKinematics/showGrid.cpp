#include "ik.h"
#include "showGrid.h"

int pointMap(int side, int i, int j)
{
	int r;

	switch (side)
	{
	case 1: //[i][j][0] bottom face
		r = 64 * i + 8 * j;
		break;
	case 6: //[i][j][7] top face
		r = 64 * i + 8 * j + 7;
		break;
	case 2: //[i][0][j] front face
		r = 64 * i + j;
		break;
	case 5: //[i][7][j] back face
		r = 64 * i + 56 + j;
		break;
	case 3: //[0][i][j] left face
		r = 8 * i + j;
		break;
	case 4: //[7][i][j] right face
		r = 448 + 8 * i + j;
		break;
	}

	return r;
}

void showGrid()
{
	int i, j, incr = 3;
	double size = boxSize;

	glColor4f(0.9f, 0.9f, 0.9f, 0.2f);

	glBegin(GL_LINES);

	//// front face
	//for (i = -size; i <= size; i += incr)
	//{
	//	glVertex3f(i, -size, -size);
	//	glVertex3f(i, -size, size);
	//}
	//for (j = -size; j <= size; j += incr)
	//{
	//	glVertex3f(-size, -size, j);
	//	glVertex3f(size, -size, j);
	//}

	//// back face
	//for (i = -size; i <= size; i += incr)
	//{
	//	glVertex3f(i, size, -size);
	//	glVertex3f(i, size, size);
	//}
	//for (j = -size; j <= size; j += incr)
	//{
	//	glVertex3f(-size, size, j);
	//	glVertex3f(size, size, j);
	//}

	//// left face
	//for (i = -size; i <= size; i += incr)
	//{
	//	glVertex3f(-size, i, -size);
	//	glVertex3f(-size, i, size);
	//}
	//for (j = -size; j <= size; j += incr)
	//{
	//	glVertex3f(-size, -size, j);
	//	glVertex3f(-size, size, j);
	//}

	//// right face
	//for (i = -size; i <= size; i += incr)
	//{
	//	glVertex3f(size, i, -size);
	//	glVertex3f(size, i, size);
	//}
	//for (j = -size; j <= size; j += incr)
	//{
	//	glVertex3f(size, -size, j);
	//	glVertex3f(size, size, j);
	//}

	// middle
	for (i = -size; i <= size; i += incr)
	{
		glVertex3f(i, -size, 0);
		glVertex3f(i, size, 0);
	}
	for (j = -size; j <= size; j += incr)
	{
		glVertex3f(-size, j, 0);
		glVertex3f(size, j, 0);
	}

	glEnd();
}

void showBoundingBox()
{
	double size = boxSize + 0.1f;

	glColor4f(0.5f, 0.5f, 0.5f, 0.2f);

	glBegin(GL_QUADS);

	// front face
	{
		glVertex3f(-size, -size, -size);
		glVertex3f(-size, -size, size);
		glVertex3f(size, -size, size);
		glVertex3f(size, -size, -size);
	}

	// back face
	{
		glVertex3f(size, size, -size);
		glVertex3f(size, size, size);
		glVertex3f(-size, size, size);
		glVertex3f(-size, size, -size);
	}

	// left face
	{
		glVertex3f(-size, size, -size);
		glVertex3f(-size, size, size);
		glVertex3f(-size, -size, size);
		glVertex3f(-size, -size, -size);
	}

	// right face
	{
		glVertex3f(size, -size, -size);
		glVertex3f(size, -size, size);
		glVertex3f(size, size, size);
		glVertex3f(size, size, -size);
	}

	//// top face
	//{
	//	glVertex3f(size, size, size);
	//	glVertex3f(size, -size, size);
	//	glVertex3f(-size, -size, size);
	//	glVertex3f(-size, size, size);
	//}

	// bottom face
	{
		glVertex3f(size, -size, -size);
		glVertex3f(size, size, -size);
		glVertex3f(-size, size, -size);
		glVertex3f(-size, -size, -size);
	}

	glEnd();

	return;
}