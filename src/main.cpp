#include "ofApp.h"


int main() {
	//monitor laptop 1366,768
	//seconod monitor 1680,1050
	// proiettore 1920, 1080
    ofSetupOpenGL(PROJECTOR_RESOLUTION_X, PROJECTOR_RESOLUTION_Y, OF_WINDOW);
	ofRunApp(new ofApp());
 

}
