#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

#include "ofxKinectProjectorToolkit.h"
// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS
// this must match the display resolution of your projector
/*
#define PROJECTOR_RESOLUTION_X 1280
#define PROJECTOR_RESOLUTION_Y 800
*/
/*
#define PROJECTOR_RESOLUTION_X 1920
#define PROJECTOR_RESOLUTION_Y 1080

*/

#define PROJECTOR_RESOLUTION_X 1680
#define PROJECTOR_RESOLUTION_Y 1050


class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
	ofxKinectProjectorToolkit   kpt;

	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	ofPath pathingshape;


	ofVideoPlayer video; //Prerecorded video
	ofFbo fbo;

	ofImage myTexture;
	ofImage image;
	ofPixels fboPixels;
	ofMesh testMesh;
	ofVboMesh tessellation;
	float tiltCurrent = 0;
	float tiltTarget = 0;
	float turnCurrent = 1;
	float turnTarget = 1;


private:

	

	bool cameraDraw = false;
	bool shapeDraw = true;
	bool showText = false;
	vector<ofPolyline> polylines, smoothed, resampled;
	vector<ofRectangle> boundingBoxes;
	vector<ofPoint> closestPoints;
	vector<unsigned int> closestIndices;
	vector<ofMesh> meshes;

	void drawWithNormals(const ofPolyline& polyline);
	
	
	float blobPosX;
	float blobPosY;
	float boundinBoxX;
	float boundinBoxY;
	float boundinBoxWidth;
	float boundinBoxHeight;
	float sizeTrash = 1000.0f;
	ofVec2f testPoint;
};
