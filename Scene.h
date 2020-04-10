#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Matrix4.h"

using namespace std;

class Scene
{
public:
	Color backgroundColor;
	bool cullingEnabled;
	int projectionType;

	vector< vector<Color> > image;
	vector< Camera* > cameras;
	vector< Vec3* > vertices;
	vector< Color* > colorsOfVertices;
	vector< Scaling* > scalings;
	vector< Rotation* > rotations;
	vector< Translation* > translations;
	vector< Model* > models;


	Scene(const char *xmlPath);

	void initializeImage(Camera* camera);
	void forwardRenderingPipeline(Camera* camera);
	int makeBetweenZeroAnd255(double value);
	void writeImageToPPMFile(Camera* camera);
	void convertPPMToPNG(string ppmFileName, int osType);

	//Additional functions
	Matrix4 translation(Translation translation);
	Matrix4 scaling(Scaling scaling);
	Matrix4 rotation(Rotation rotation);
	double findMin(double ux, double uy, double uz);
	Matrix4 modeling_transformations(Model*& model);
	Matrix4 viewing_transformations(Model*& model, Camera*& cam);
	Matrix4 camera_transformations(Camera*& cam);
	Matrix4 viewport_transformations(Camera*& cam);
	void mid_point(Model*& model);
	void mid_point_helper(double d1, double d2, double d3, double d4,Color* color_0,Color* color_1, double first, double second, int temp);
	void triangle_rasterization(Model*& model);
};

#endif
