#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;

double PI = 3.1415;

//Additional functions
Matrix4 Scene::translation(Translation translation){
	printf("Translation Transform in\n");
	double TransMatrix[4][4];
	TransMatrix[0][0] = 1.0;
	TransMatrix[0][1] = 0.0;
	TransMatrix[0][2] = 0.0;
	TransMatrix[0][3] = translation.tx;

	TransMatrix[1][0] = 0.0;
	TransMatrix[1][1] = 1.0;
	TransMatrix[1][2] = 0.0;
	TransMatrix[1][3] = translation.ty;

	TransMatrix[2][0] = 0.0;
	TransMatrix[2][1] = 0.0;
	TransMatrix[2][2] = 1.0;
	TransMatrix[2][3] = translation.tz;

	TransMatrix[3][0] = 0.0;
	TransMatrix[3][1] = 0.0;
	TransMatrix[3][2] = 0.0;
	TransMatrix[3][3] = 1.0;

	return Matrix4(TransMatrix);
}

Matrix4 Scene::scaling(Scaling scaling){
	printf("Scaling Transform in\n");
	double ScaleMatrix[4][4];
	ScaleMatrix[0][0] = scaling.sx;
	ScaleMatrix[0][1] = 0.0;
	ScaleMatrix[0][2] = 0.0;
	ScaleMatrix[0][3] = 0.0;

	ScaleMatrix[1][0] = 0.0;
	ScaleMatrix[1][1] = scaling.sy;
	ScaleMatrix[1][2] = 0.0;
	ScaleMatrix[1][3] = 0.0;

	ScaleMatrix[2][0] = 0.0;
	ScaleMatrix[2][1] = 0.0;
	ScaleMatrix[2][2] = scaling.sz;
	ScaleMatrix[2][3] = 0.0;

	ScaleMatrix[3][0] = 0.0;
	ScaleMatrix[3][1] = 0.0;
	ScaleMatrix[3][2] = 0.0;
	ScaleMatrix[3][3] = 1.0;

	return Matrix4(ScaleMatrix);
}

double Scene::findMin(double ux, double uy, double uz){
	double min = uz;
	if(abs(uy) < abs(min)){
		min = uy;
	}
	if(abs(ux) < abs(min)){
		min = ux;
	}
	return min;
}

Matrix4 Scene::rotation(Rotation rotation){
	printf("Rotation Transform in\n");
	Matrix4 RotateMatrix;
	Vec3 u;
	u.x = rotation.ux;
	u.y = rotation.uy;
	u.z = rotation.uz;

	u.x = normalizeVec3(u).x;
  u.y = normalizeVec3(u).y;
  u.z = normalizeVec3(u).z;

	Vec3 u_dp;
	double min_el = findMin(u.x,u.y,u.z);
	if(min_el==u.x){
		u_dp.x = 0.0;
		u_dp.y = u.z;
		u_dp.z = (-1.0) * u.y;
	}
	else if(min_el==u.y){
		u_dp.y = 0.0;
		u_dp.x = u.z;
		u_dp.z = (-1.0) * u.x;
	}
	else if(min_el==u.z){
		u_dp.z = 0.0;
		u_dp.x = u.y;
		u_dp.y = (-1.0) * u.x;
	}

	u_dp = normalizeVec3(u_dp);

	Vec3 u_tp;
	u_tp = crossProductVec3(u,u_dp);

	u_tp = normalizeVec3(u_tp);

	double matrix_M[4][4];
	matrix_M[0][0] = u.x;
	matrix_M[0][1] = u.y;
	matrix_M[0][2] = u.z;
	matrix_M[0][3] = 0.0;

	matrix_M[1][0] = u_dp.x;
	matrix_M[1][1] = u_dp.y;
	matrix_M[1][2] = u_dp.z;
	matrix_M[1][3] = 0.0;

	matrix_M[2][0] = u_tp.x;
	matrix_M[2][1] = u_tp.y;
	matrix_M[2][2] = u_tp.z;
	matrix_M[2][3] = 0.0;

	matrix_M[3][0] = 0.0;
	matrix_M[3][1] = 0.0;
	matrix_M[3][2] = 0.0;
	matrix_M[3][3] = 1.0;

	double matrix_M_inverse[4][4];
	matrix_M_inverse[0][0] = u.x;
	matrix_M_inverse[0][1] = u_dp.x;
	matrix_M_inverse[0][2] = u_tp.x;
	matrix_M_inverse[0][3] = 0.0;

	matrix_M_inverse[1][0] = u.y;
	matrix_M_inverse[1][1] = u_dp.y;
	matrix_M_inverse[1][2] = u_tp.y;
	matrix_M_inverse[1][3] = 0.0;

	matrix_M_inverse[2][0] = u.z;
	matrix_M_inverse[2][1] = u_dp.z;
	matrix_M_inverse[2][2] = u_tp.z;
	matrix_M_inverse[2][3] = 0.0;

	matrix_M_inverse[3][0] = 0.0;
	matrix_M_inverse[3][1] = 0.0;
	matrix_M_inverse[3][2] = 0.0;
	matrix_M_inverse[3][3] = 1.0;

	//rotate around x axis
	double alpha = (rotation.angle*PI)/180.0;
	double matrix_rotation[4][4];
	matrix_rotation[0][0] = 1.0;
	matrix_rotation[0][1] = 0.0;
	matrix_rotation[0][2] = 0.0;
	matrix_rotation[0][3] = 0.0;

	matrix_rotation[1][0] = 0.0;
	matrix_rotation[1][1] = cos(alpha);
	matrix_rotation[1][2] = (-1.0)*sin(alpha);
	matrix_rotation[1][3] = 0.0;

	matrix_rotation[2][0] = 0.0;
	matrix_rotation[2][1] = sin(alpha);
	matrix_rotation[2][2] = cos(alpha);
	matrix_rotation[2][3] = 0.0;

	matrix_rotation[3][0] = 0.0;
	matrix_rotation[3][1] = 0.0;
	matrix_rotation[3][2] = 0.0;
	matrix_rotation[3][3] = 1.0;

	Matrix4 temp = multiplyMatrixWithMatrix(Matrix4(matrix_rotation),Matrix4(matrix_M));
	RotateMatrix = multiplyMatrixWithMatrix(Matrix4(matrix_M_inverse),temp);

	return RotateMatrix;
}

Matrix4 Scene::modeling_transformations(Model*& model){
	printf("Modeling Transform in\n");
	int transformation = model->numberOfTransformations;
	Matrix4 temp_matrix = getIdentityMatrix();

	for(int i=0; i<transformation;i++){
		char trans_type = model->transformationTypes[i];
		int trans_id = model->transformationIds[i]-1;
		if(trans_type == 't'){
			Matrix4 trans_matrix = this->translation(*translations[trans_id]);
			temp_matrix = multiplyMatrixWithMatrix(trans_matrix,temp_matrix);
		}
		else if(trans_type == 's'){
			Matrix4 trans_matrix = this->scaling(*scalings[trans_id]);
			temp_matrix = multiplyMatrixWithMatrix(trans_matrix,temp_matrix);
		}
		else if(trans_type == 'r'){
			Matrix4 trans_matrix = this->rotation(*rotations[trans_id]);
			temp_matrix = multiplyMatrixWithMatrix(trans_matrix,temp_matrix);
		}
	}
	return temp_matrix;
}

Matrix4 Scene::viewing_transformations(Model*& model, Camera*& cam){
	printf("Viewing Transform in\n");
	if(this->projectionType == 0){
		double orth[4][4];
		orth[0][0] = 2/(cam->right - cam->left);
		orth[0][1] = 0.0;
		orth[0][2] = 0.0;
		orth[0][3] = -1*(cam->right+cam->left)/(cam->right-cam->left);

		orth[1][0] = 0.0;
		orth[1][1] = 2/(cam->top - cam->bottom);
		orth[1][2] = 0.0;
		orth[1][3] = -1*(cam->top+cam->bottom)/(cam->top-cam->bottom);

		orth[2][0] = 0.0;
		orth[2][1] = 0.0;
		orth[2][2] = -2/(cam->far - cam->near);
		orth[2][3] = -1*(cam->far+cam->near)/(cam->far-cam->near);

		orth[3][0] = 0.0;
		orth[3][1] = 0.0;
		orth[3][2] = 0.0;
		orth[3][3] = 1.0;
		return Matrix4(orth);
	}
	else{
		double pers[4][4];
		pers[0][0] = (2*cam->near)/(cam->right-cam->left);
		pers[0][1] = 0.0;
		pers[0][2] = (cam->right+cam->left)/(cam->right-cam->left);
		pers[0][3] = 0.0;

		pers[1][0] = 0.0;
		pers[1][1] = (2*cam->near)/(cam->top - cam->bottom);
		pers[1][2] = (cam->top+cam->bottom)/(cam->top-cam->bottom);
		pers[1][3] = 0.0;

		pers[2][0] = 0.0;
		pers[2][1] = 0.0;
		pers[2][2] = -1*(cam->far+cam->near)/(cam->far-cam->near);
		pers[2][3] = (-2*cam->far*cam->near)/(cam->far - cam->near);

		pers[3][0] = 0.0;
		pers[3][1] = 0.0;
		pers[3][2] = -1.0;
		pers[3][3] = 0.0;
		return Matrix4(pers);
	}
}

Matrix4 Scene::camera_transformations(Camera*& cam){
	printf("Camera Transform in\n");
	double camera[4][4];
	camera[0][0] = cam->u.x;
	camera[0][1] = cam->u.y;
	camera[0][2] = cam->u.z;
	camera[0][3] = -1*(cam->u.x*cam->pos.x+cam->u.y*cam->pos.y+cam->u.z*cam->pos.z);

	camera[1][0] = cam->v.x;
	camera[1][1] = cam->v.y;
	camera[1][2] = cam->v.z;
	camera[1][3] = -1*(cam->v.x*cam->pos.x+cam->v.y*cam->pos.y+cam->v.z*cam->pos.z);

	camera[2][0] = cam->w.x;
	camera[2][1] = cam->w.y;
	camera[2][2] = cam->w.z;
	camera[2][3] = -1*(cam->w.x*cam->pos.x+cam->w.y*cam->pos.y+cam->w.z*cam->pos.z);

	camera[3][0] = 0.0;
	camera[3][1] = 0.0;
	camera[3][2] = 0.0;
	camera[3][3] = 1.0;

	return Matrix4(camera);
}

Matrix4 Scene::viewport_transformations(Camera*& cam){
	printf("viewPort Transform in\n");
	double vp[4][4];
	vp[0][0] = cam->horRes/2;
	vp[0][1] = 0.0;
	vp[0][2] = 0.0;
	vp[0][3] = (cam->horRes-1)/2;

	vp[1][0] = 0.0;
	vp[1][1] = cam->verRes/2;
	vp[1][2] = 0.0;
	vp[1][3] = (cam->verRes-1)/2;

	vp[2][0] = 0.0;
	vp[2][1] = 0.0;
	vp[2][2] = 0.5;
	vp[2][3] = 0.5;

	vp[3][0] = 0.0;
	vp[3][1] = 0.0;
	vp[3][2] = 0.0;
	vp[3][3] = 0.0;

	return Matrix4(vp);
}

void Scene::mid_point_helper(double d1, double d2, double d3, double d4,Color* color_0,Color* color_1, double first, double second, int temp){
	printf("Midpoint helper in\n");
	double d = 2*abs(d1-d2)-abs(d3-d4);
	double maximum = max(d3,d4);
	//printf("aaaaaaaaaaaaaaaaaaaaaaaaaaa%fbbbbbbbbbbbbbbbbbbbbbbbbb%f\n", maximum,first);
	while(first<maximum){
		//cout<<d4<<",,,,,,,,,,,,,"<<(first)<<",,,,,,,,,,"<<second<<endl;
		image[first][second].r = (double)(color_0->r*abs(first-d3) + color_1->r*abs(d4-first))/(double)abs(d3-d4);
		//printf("renk renk renk%f\n", image[first][second].r);
		image[first][second].g = (double)(color_0->g*abs(first-d3) + color_1->g*abs(d4-first))/(double)abs(d3-d4);
		image[first][second].b = (double)(color_0->b*abs(first-d3) + color_1->b*abs(d4-first))/(double)abs(d3-d4);
		//printf("lolooolol in\n");
		if(d<=0)
			d+=2*abs(d1-d2);
		else{
			d+=2*abs(d1-d2)-abs(d3-d4);
			if(temp==0)
				second++;
			if(temp==1)
				second--;
		}
		first++;
	}
}


void Scene::mid_point(Model*& model){
	printf("Midpoint in\n");
	//printf("%d\n",model->numberOfTriangles );
		for(int vertex=0;vertex<vertices.size();vertex++){
			double x_0 = vertices[vertex]->x;
			//printf("%d\n",x_0 );
			double x_1 = vertices[(vertex+1)%vertices.size()]->x;

			double y_0 = vertices[vertex]->y;
			double y_1 = vertices[(vertex+1)%vertices.size()]->y;

			Color* color_0 = colorsOfVertices[vertex];
			Color* color_1 = colorsOfVertices[(vertex+1)%vertices.size()];

			if (x_1-x_0 == 0 || y_1-y_0==0){
				return;
			}
			double m = (double)(y_1-y_0)/(double)(x_1-x_0);
			if(0.0<m && m<1.0){
				double x = min(x_0,x_1);
				double y = min(y_0,y_1);
				mid_point_helper(y_1,y_0,x_1,x_0, color_0, color_1,x,y,0);
			}
			else if (1.0 <= m){
				double x = min(x_0,x_1);
				double y = min(y_0,y_1);
				mid_point_helper(x_1,x_0,y_1,y_0,color_0,color_1,y,x,0);
			}
			else if(m<=0.0 && m>=-1.0){
				double x = min(x_0,x_1);
				double y = max(y_0,y_1);
				mid_point_helper(y_1,y_0,x_1,x_0,color_0,color_1,x,y,1);
			}
			else if (-1.0 > m){
				double x = max(x_0,x_1);
				double y = min(y_0,y_1);
				mid_point_helper(x_1,x_0,y_1,y_0,color_0,color_1,y,x,1);
			}

	}
}

void Scene::triangle_rasterization(Model*& model){
	for(int i=0; i<model->numberOfTriangles;i++){

	}
}



/*
	Transformations, clipping, culling, rasterization are done here.
	You can define helper functions inside Scene class implementation.
*/
void Scene::forwardRenderingPipeline(Camera *camera)
{
	// TODO: Implement this function.
	Matrix4 matrix;
	vector< Vec3 > transformed_vertices;
	for(int j=0;j<this->models.size();j++){
		vector<int> vertexID;
		//printf("%d\n",j);
		//printf("%f\n",vertices[models[j]->triangles[0].vertexIds[0]-1]->x);
		matrix = this->camera_transformations(camera);
		matrix = multiplyMatrixWithMatrix(this->viewing_transformations(models[j],camera),matrix);
		//printf("%f\n",vertices[models[j]->triangles[0].vertexIds[0]-1]->x);
		int tri_number = models[j]->numberOfTriangles;
		for(int i=0;i<tri_number;i++){
			printf("anothe triii %d\n",i);
			for(int vertex=0;vertex<3;vertex++){
				int a = models[j]->triangles[i].vertexIds[vertex]-1;
				if(find(vertexID.begin(),vertexID.end(),a) != vertexID.end()){
					continue;
				}

				vertexID.push_back(models[j]->triangles[i].vertexIds[vertex]-1);

				Vec4 vec;
				vec.x = vertices[models[j]->triangles[i].vertexIds[vertex]-1]->x;
				//printf("%f\n",vec.x );
				vec.y = vertices[models[j]->triangles[i].vertexIds[vertex]-1]->y;
				vec.z = vertices[models[j]->triangles[i].vertexIds[vertex]-1]->z;
				vec.t = 1;


				vec = multiplyMatrixWithVec4(this->modeling_transformations(models[j]),vec);
				//cout<<vertices[models[j]->triangles[i].vertexIds[vertex]-1]->x<<",,,,,,,,,,,,,,,,,,,,"<<vertices[models[j]->triangles[i].vertexIds[vertex]-1]->y<<",,,,,,,,,,,,,,,"<<vertices[models[j]->triangles[i].vertexIds[vertex]-1]->z<<endl ;
				vertices[models[j]->triangles[i].vertexIds[vertex]-1]->x = (int)vec.x;
				vertices[models[j]->triangles[i].vertexIds[vertex]-1]->y = (int)vec.y;
				vertices[models[j]->triangles[i].vertexIds[vertex]-1]->z = (int)vec.z;
			}
		}
		for(int a=0;a<vertexID.size();a++){
			printf("%d\n", vertexID[a]);
		}
			for(int vertex=0;vertex<vertices.size();vertex++){
				Vec4 vec;
				vec.x = vertices[vertex]->x;
				//printf("%f\n",vec.x );
				vec.y = vertices[vertex]->y;
				vec.z = vertices[vertex]->z;
				vec.t = 1;

				vec = multiplyMatrixWithVec4(matrix,vec);

				//Perspective divide
				vec.x = vec.x/vec.t;
				vec.y = vec.y/vec.t;
				vec.z = vec.z/vec.t;
				vec.t = vec.t/vec.t;

				//cout<<vec.x<<"...............aaaaaaaa...."<<vec.y<<"....aaaaaaaaaaa......"<<vec.z<<endl ;

				Vec4 result;
				result = multiplyMatrixWithVec4(this->viewport_transformations(camera),vec);
				vertices[vertex]->x = (int)result.x;
				vertices[vertex]->y = (int)result.y;
				vertices[vertex]->z = (int)result.z;


				//transformed_vertices.push_back(Vec3((int)result.x,(int)result.y,(int)result.z,(int)result.colorId));

				cout<<vertices[vertex]->x<<"..................."<<vertices[vertex]->y<<".........."<<vertices[vertex]->z<<endl ;

		}
		//if(models[j]->type == 0){//if wireframe
			//printf("%f\n",vertices[models[j]->triangles[0].vertexIds[0]-1]->x );
			//printf("%f\n",vertices[models[j]->triangles[0].vertexIds[1]-1]->x );
			//printf("%f\n",vertices[models[j]->triangles[0].vertexIds[2]-1]->x );
			mid_point(models[j]);
		//}
		if(models[j]->type == 1){
			triangle_rasterization(models[j]);
		}
	}
}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL)
		pElement->QueryBoolText(&cullingEnabled);

	// read projection type
	pElement = pRoot->FirstChildElement("ProjectionType");
	if (pElement != NULL)
		pElement->QueryIntText(&projectionType);

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read models
	pElement = pRoot->FirstChildElement("Models");

	XMLElement *pModel = pElement->FirstChildElement("Model");
	XMLElement *modelElement;
	while (pModel != NULL)
	{
		Model *model = new Model();

		pModel->QueryIntAttribute("id", &model->modelId);
		pModel->QueryIntAttribute("type", &model->type);

		// read model transformations
		XMLElement *pTransformations = pModel->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		pTransformations->QueryIntAttribute("count", &model->numberOfTransformations);

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			model->transformationTypes.push_back(transformationType);
			model->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		// read model triangles
		XMLElement *pTriangles = pModel->FirstChildElement("Triangles");
		XMLElement *pTriangle = pTriangles->FirstChildElement("Triangle");

		pTriangles->QueryIntAttribute("count", &model->numberOfTriangles);

		while (pTriangle != NULL)
		{
			int v1, v2, v3;

			str = pTriangle->GetText();
			sscanf(str, "%d %d %d", &v1, &v2, &v3);

			model->triangles.push_back(Triangle(v1, v2, v3));

			pTriangle = pTriangle->NextSiblingElement("Triangle");
		}

		models.push_back(model);

		pModel = pModel->NextSiblingElement("Model");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	// if image is filled before, just change color rgb values with the background color
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}
