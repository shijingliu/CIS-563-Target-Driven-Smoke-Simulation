// Modified by Peter Kutz, 2011 and 2012.

#ifndef MACGrid_H_
#define MACGrid_H_

#pragma warning(disable: 4244 4267 4996)

#include "open_gl_headers.h"
#include "vec.h"
#include "grid_data.h"
#include "grid_data_matrix.h"

#include "blurfilter.h"
//#include "..\bmp_manager.h"   

#include "bmp_manager.h"
#include "EasyBMP.h" 

class Camera;

class MACGrid
{

public:
	
	TheBMP mimg;
	TheBMP mimg2;
	int totalframe;

	//for easyBMP   
	BMP AnImage1;
	BMP AnImage2;
	BMP AnImage3;
	BMP AnImage4;
	//added 

	void target_driven();
	int roundUP(double num); 
	bool triangle (int i, int j, int firsti, int firstj, int secondi, int secondj, int thirdi, int thirdj, double &u, double &v, double &w);
	void checkTriangle1 (int i, int j, int & cosI, int &cosJ);
	void checkTriangle2 (int i, int j, int & cosI, int &cosJ); 
	void checkTriangle3 (int i, int j, int & cosI, int &cosJ); 

	bool isValidFace(int i, int j, int k);    
	MACGrid();
	~MACGrid();
	MACGrid(const MACGrid& orig);
	MACGrid& operator=(const MACGrid& orig);

	void reset();

	void draw(const Camera& c);
	void updateSources();
	void advectVelocity(double dt);
	void addExternalForces(double dt);
	void project(double dt);
	void advectTemperature(double dt);
	void advectDensity(double dt);
	enum Direction { X, Y, Z };

	//add several setting functions here    
	void SetParamVf(double vf);
	void SetParamVd(double vd);
	void SetParamVg(double vg);
	void SetParamSigma(double sigma);

protected:

	// Setup:   
	void initialize();

	// Simulation:  
	void computeBouyancy(double dt);
	void computeWind(double dt); 
	void computeVorticityConfinement(double dt);

	// Rendering:
	struct Cube { vec3 pos; vec4 color; double dist; };
	void drawWireGrid();
	void drawSmokeCubes(const Camera& c);
	void drawSmoke(const Camera& c);
	void drawCube(const MACGrid::Cube& c);
	void drawFace(const MACGrid::Cube& c);
	void drawVelocities();
	vec4 getRenderColor(int i, int j, int k);
	vec4 getRenderColor(const vec3& pt);
	void drawZSheets(bool backToFront);
	void drawXSheets(bool backToFront);

	// GridData accessors:    
	//enum Direction { X, Y, Z };
	vec3 getVelocity(const vec3& pt);
	double getVelocityX(const vec3& pt);
	double getVelocityY(const vec3& pt);
	double getVelocityZ(const vec3& pt);
	double getTemperature(const vec3& pt);
	double getDensity(const vec3& pt);
	vec3 getCenter(int i, int j, int k);

	vec3 getFaceCenterX (int i, int j, int k); 
	vec3 getFaceCenterY (int i, int j, int k);
	vec3 getFaceCenterZ (int i, int j, int k); 

	// Sets up the A matrix:
	void setUpAMatrix();

	// Conjugate gradient stuff:
	bool conjugateGradient(const GridDataMatrix & A, GridData & p, const GridData & d, int maxIterations, double tolerance);
	double dotProduct(const GridData & vector1, const GridData & vector2);
	void add(const GridData & vector1, const GridData & vector2, GridData & result);
	void subtract(const GridData & vector1, const GridData & vector2, GridData & result);
	void multiply(const double scalar, const GridData & vector, GridData & result);
	double maxMagnitude(const GridData & vector);
	void apply(const GridDataMatrix & matrix, const GridData & vector, GridData & result);
	bool isValidCell(int i, int j, int k);


	//add the scaleMass functions here    
	void scaleMass();

	//add driving force and momentum attenuation here      
	void applyDrivingForce();
	void attenuateMomentum();

	// Fluid grid cell properties:
	GridDataX mU; // X component of velocity, stored on X faces, size is (dimX+1)*dimY*dimZ                   
	GridDataY mV; // Y component of velocity, stored on Y faces, size is dimX*(dimY+1)*dimZ
	GridDataZ mW; // W component of velocity, stored on Z faces, size is dimX*dimY*(dimZ+1)
	GridData mP;  // Pressure, stored at grid centers, size is dimX*dimY*dimZ
	GridData mD;  // Density, stored at grid centers, size is dimX*dimY*dimZ
	GridData mT;  // Temperature, stored at grid centers, size is dimX*dimY*dimZ


	// The A matrix:   
	GridDataMatrix AMatrix;

	//add three control parameters 
	double m_vf;		// driving force coefficient 
	double m_vd;		// momentum attenuation coefficient
	double m_vg;		// smoke gathering coefficient

	//define source density and target density here 
	//source density should be extracted from update source 
	//target density should be extracted from the image directly      
	GridData srcDensity;      //height * width or row * col 
	GridData tarDensity;      //height * width or row * col 

	//add some other parameters 
	GridData mTargetDensity;   //(row + 2) * (col + 2)     
	GridData mSmoothDensity;   //(row + 2) * (col + 2) 
	GridData mSmoothTargetDensity; //(row + 2) * (col + 2)
	GridData mDensityError; //(row + 2) * (col + 2) 
    GridData mNormalizedGradU; //(row + 2) * (col + 3) 
	GridData mNormalizedGradV; //(row + 3) * (row + 2) 

    
	//declare a pointer for blurfilter class 
	BlurFilter *mFilter;

	//add dt, dx, dt/dx and 2*dx here 
	//dt is exact the same as the dt in the smoke_sim.cpp
	double m_dt; 
	double m_dx;        
	double m_dtdx;		// this is dt/dx     
	double m_2dx;		// this is 2*dx

public:

	enum RenderMode { CUBES, SHEETS };
	static RenderMode theRenderMode;
	static bool theDisplayVel;
	
	// Saves smoke in CIS 460 volumetric format:
	void saveSmoke(const char* fileName);

};


inline void MACGrid::SetParamVf(double vf)
{
	m_vf = vf;
}

inline void MACGrid::SetParamVd(double vd)
{
	m_vd = vd;
}

inline void MACGrid::SetParamVg(double vg)
{
	m_vg = vg;
}

#endif