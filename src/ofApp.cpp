#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/



//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	//kinect.open();		// opens first available kinect
	kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	/*if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}*/
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 199;
	farThreshold = 190;
	bThreshWithOpenCV = true;
	
	
	video.loadMovie("video.ogv");
	
	video.play();
	video.setPaused(true);
	kpt.loadCalibration("C:/Users/ray/Desktop/of_v0.9.1_vs_release/apps/myApps/projectionTest/bin/data/calibration.xml");

	
	fbo.allocate(1920,1080);
	
	// clear fbo
	fbo.begin();
	ofClear(0, 0, 0, 0);
	fbo.end();
	
	ofSetFrameRate(25);
	

}

//--------------------------------------------------------------
void ofApp::update() {
	
//	ofBackground(100, 100, 100);
	
	kinect.update();
	video.update();
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
	
		grayImage.setFromPixels(kinect.getDepthPixels());
		// load grayscale depth image from the kinect source
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
					
					
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		polylines.clear();
		smoothed.clear();
		resampled.clear();
		meshes.clear();
		boundingBoxes.clear();
		closestPoints.clear();
		closestIndices.clear();
		pathingshape.clear();
		contourFinder.findContours(grayImage, 1000, (kinect.width*kinect.height)/3, 3, false,true);
	
		
		fbo.readToPixels(fboPixels);
		image.setFromPixels(fboPixels);
		
		fbo.begin();
			
			if(contourFinder.blobs.size()>0){
				video.setPaused(false);
			}
			else {
				video.setPaused(true);
			}
			for (unsigned int i = 0; i < contourFinder.blobs.size(); i++) {
				ofPolyline cur;
				// add all the current vertices to cur polyline
					
					cur.addVertices(contourFinder.blobs[i].pts);
					cur.setClosed(true);
					
					
					float area = abs(cur.getArea());
					//cout << area << endl;

					if (area > sizeTrash) {
					// add the cur polyline to all these vector<ofPolyline>
					polylines.push_back(cur);
					smoothed.push_back(cur.getSmoothed(3));
					resampled.push_back(cur.getResampledByCount(30).getSmoothed(10));


					boundingBoxes.push_back(cur.getBoundingBox());
					float getHeight = cur.getBoundingBox().getHeight();
					float getWidth = cur.getBoundingBox().getWidth();

					ofVec3f worldBoxPosition = kinect.getWorldCoordinateAt(cur.getBoundingBox().getPosition().x, cur.getBoundingBox().getPosition().y);
					ofVec2f projectedBoxPosition = kpt.getProjectedPoint(worldBoxPosition);

					ofVec3f worldBoxSize = kinect.getWorldCoordinateAt(cur.getBoundingBox().getWidth(), cur.getBoundingBox().getHeight());
					ofVec2f projectedBoxSize = kpt.getProjectedPoint(worldBoxSize);

					/*					
					cout << projectedBoxSize.x << " x" << endl;
					cout << projectedBoxSize.y << " y" << endl;
					*/

					float boundingBoxMappedx = ofMap(projectedBoxPosition.x, 0, 1, 0, PROJECTOR_RESOLUTION_X);
					float boundingBoxMappedy = ofMap(projectedBoxPosition.y, 0, 1, 0, PROJECTOR_RESOLUTION_Y);
					float boundingBoxWidth = ofMap(projectedBoxSize.x*-1 , 0, 1, 0, PROJECTOR_RESOLUTION_X);
					float boundingBoxHeight = ofMap(projectedBoxSize.y , 0, 1, 0, PROJECTOR_RESOLUTION_Y);
					/*cout << boundingBoxWidth << " x" << endl;
					cout << boundingBoxHeight << " y" << endl;*/
					
					ofClear(0, 0, 0, 0);
					
					video.draw(boundingBoxMappedx, boundingBoxMappedy, boundingBoxWidth, boundingBoxHeight*3);
					//video.draw(boundingBoxMappedx, boundingBoxMappedy, boundingBoxWidth, boundingBoxHeight);

					}
				}
		fbo.end();
		
		}
	#ifdef USE_TWO_KINECTS
		kinect2.update();
	#endif
}



void ofApp::drawWithNormals(const ofPolyline& polyline) {
	for (int i = 0; i< (int)polyline.size(); i++) {
		bool repeatNext = i == (int)polyline.size() - 1;
		const ofPoint& cur = polyline[i];
		const ofPoint& next = repeatNext ? polyline[0] : polyline[i + 1];
	/*	
		
		ofVec3f worldStartingPoint = kinect.getWorldCoordinateAt(cur.x, cur.y);
		ofVec2f projectedStartingPoint = kpt.getProjectedPoint(worldStartingPoint);

		
		
		ofVec3f worldPoint = kinect.getWorldCoordinateAt(next.x, next.y);
		ofVec2f projectedPoint = kpt.getProjectedPoint(worldPoint);
		
		float projectedStartingPointxMapped = ofMap(projectedStartingPoint.x, 0, 1, 0, PROJECTOR_RESOLUTION_X);
		float projectedStartingPointyMapped = ofMap(projectedStartingPoint.y, 0, 1, 0, PROJECTOR_RESOLUTION_Y);
		
		float projectedPointxMapped = ofMap(projectedPoint.x, 0, 1, 0, PROJECTOR_RESOLUTION_X);
		float projectedPointyMapped = ofMap(projectedPoint.y, 0, 1, 0, PROJECTOR_RESOLUTION_Y);
		
		pathingshape.setMode(ofPath::POLYLINES);
		*/

		float projectedStartingPointx = ofMap(cur.x, 0, 640, 0, PROJECTOR_RESOLUTION_X);
		float projectedStartingPointy = ofMap(cur.y, 0, 480, 0, PROJECTOR_RESOLUTION_Y);
		float projectedPointx = ofMap(next.x, 0, 640, 0, PROJECTOR_RESOLUTION_X);
		float projectedPointy = ofMap(next.y, 0, 480, 0, PROJECTOR_RESOLUTION_Y);

		if (i == 0) {
			pathingshape.newSubPath();
			pathingshape.moveTo(projectedStartingPointx, projectedStartingPointy);
			
			//pathingshape.moveTo(projectedStartingPointxMapped, projectedStartingPointyMapped);
			
		}
		else {
			pathingshape.lineTo(projectedPointx, projectedPointy);
			
			
		//pathingshape.lineTo(projectedPointxMapped, projectedPointyMapped);
		
		}
		
		pathingshape.setFillColor(0);
		pathingshape.setFilled(true);
		pathingshape.simplify(0.1f);
		tessellation = pathingshape.getTessellation();
	
		
		//Set up a texture coordinates for all the vertices
		pathingshape.draw();
	}
	for (unsigned i = 0; i < tessellation.getNumVertices(); i++) {

		tessellation.addTexCoord(ofPoint(tessellation.getVertex(i).x, tessellation.getVertex(i).y));
		

	}
	

}



//--------------------------------------------------------------
void ofApp::draw() {
	
	


		ofBackground(ofColor::blue);
	
		if (shapeDraw) {


			for (unsigned int i = 0; i < polylines.size(); i++) {
				//drawWithNormals(resampled[i]);
				drawWithNormals(smoothed[i]);
			}
			image.bind();
			tessellation.draw();
			image.unbind();
			ofSetHexColor(0xCCCCCC);
			ofDrawCircle(20, 20, 20);
			
		}
		else {
			fbo.begin();
			ofClear(0, 0, 0, 0);
			fbo.end();
		}
	

	//SET UP WINDOWS
	
	if (cameraDraw ) {
	// draw from the live kinect
	kinect.drawDepth(10, 10, 400, 300);
	kinect.draw(420, 10, 400, 300);

	grayImage.draw(10, 320, 400, 300);
	contourFinder.draw(0, 0, PROJECTOR_RESOLUTION_X, PROJECTOR_RESOLUTION_Y);
	}
	
	// draw poliLine
	
	
	
	
	if (showText == true) {
		ofSetColor(255, 255, 255);
		stringstream reportStream;
		reportStream << "set near threshold " << nearThreshold << " (press: + -)" << endl
			<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
			<< ", fps: " << ofGetFrameRate() << endl
			<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
			<< ", size trash " << sizeTrash << endl;


		ofDrawBitmapString(reportStream.str(), 420, 320);
	}




}



//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
			
		case '>':
		case '.':
			farThreshold ++;
			cout << farThreshold << " far" << endl;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			cout << farThreshold << " far" << endl;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			cout << nearThreshold << " near" << endl;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			cout << nearThreshold << " near" << endl;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			//kinect.setCameraTiltAngle(angle); // go back to prev tilt
			//kinect.open();
			break;
			
		case 'c':
			//kinect.setCameraTiltAngle(0); // zero the tilt
			//kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			sizeTrash++;
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			sizeTrash--;
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
		case 't':
			showText = !showText;
			break;
			
		case OF_KEY_UP:
			angle++;
			cout << angle << endl;
			if(angle>1000) angle=1000;
			//kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			cout << angle << endl;
			if(angle<0) angle=0;
			//kinect.setCameraTiltAngle(angle);
			break;
		case 's':
			shapeDraw = !shapeDraw;
			break;
		case 'd':
			cameraDraw = !cameraDraw;

			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	
}
//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}