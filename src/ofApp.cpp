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
    
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.init();
    kinect2.open();
#endif
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    

    
    // zero the tilt on startup
//    angle = -20;
    angle = kinectAngle;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;
    bDrawTriangleCloud = false;
    
    
    //--- setup audio
    soundStream.printDeviceList();
    soundStream.setDeviceID(0);
    soundStream.setup(2, 0, 44100, 256, 4);
    
    if(live){
        fftLive.setMirrorData(false);
        fftLive.setup();
        
    }else{
        fftFile.setMirrorData(false);
        fftFile.setup();
    }
    
    meshSpecOrig = meshSpecWarped = ofMesh::sphere(200, 30);

    camDist = 1000;
    
    //--- end audio
    
//----- flow tools
    drawWidth = 1024;
    drawHeight = 786;
    // process all but the density on 16th resolution
    flowWidth = drawWidth / 4;
    flowHeight = drawHeight / 4;
    
    // FLOW & MASK
    opticalFlow.setup(flowWidth, flowHeight);
    velocityMask.setup(drawWidth, drawHeight);
    
    velocityField.setup(flowWidth / 4, flowHeight / 4);

    // FLUID & PARTICLES
    fluidSimulation.setup(flowWidth, flowHeight, drawWidth, drawHeight);
    particleFlow.setup(flowWidth, flowHeight, drawWidth/5, drawHeight/5);
    
    displayScalar.setup(flowWidth, flowHeight);

    
    // cams etc
    fboTest.allocate(drawWidth, drawHeight, GL_RGBA32F);  //PB test

//----- end flow tools
    
//     Light
    
    ofEnableLighting();
    ofEnableDepthTest();
    areaLight.setup();
    areaLight.enable();
    areaLight.setAreaLight(2000, 2000);
//    areaLight.setSpotlight(80,3);
    areaLight.setAmbientColor(ofFloatColor(0.9,0.9,0.9));
    areaLight.setAttenuation(0.000001,0.000001,0.000001);
    areaLight.setDiffuseColor(ofFloatColor(1,1,1));
    areaLight.setSpecularColor(ofFloatColor(1,1,1));
    areaLight.rotate(-30,ofVec3f(0,1,0));
    areaLight.rotate(210,ofVec3f(1,0,0));
    areaLight.rotate(10,ofVec3f(0,0,1));
    areaLight.setPosition(0,300,-1000);

    //  end light
}

void ofApp::setupGui(){
    
    // Audio GUI
    string guiPath = "audio.xml";
    guiAudio.setup("audio", guiPath, 20, 20);
    guiAudio.add(audioThreshold.setup("audioThreshold", 1.0, 0.0, 1.0));
    guiAudio.add(audioPeakDecay.setup("audioPeakDecay", 0.95, 0.0, 1.0));
    guiAudio.add(audioMaxDecay.setup("audioMaxDecay", 0.995, 0.0, 1.0));
    guiAudio.add(cloudDisplacement.setup("cloudDisplacement", 200, 0, 1000));
    guiAudio.add(audioMirror.setup("audioMirror", true));
    guiAudio.add(spectrumRects.setup("Spec Rect", true));
    guiAudio.add(randRects.setup("Rand Rect", false));
    guiAudio.add(kinectAngle.setup("Kinect Angle", -30, -30, 30));
    guiAudio.add(kinectFarThresh.setup("Kinect Far Thresh", 1400, 100, 5000));
    guiAudio.add(kinectNearThresh.setup("Kinect Near Thresh", 200, 10, 2000));
    guiAudio.add(speedTunnel.setup("Tunnel Speed", 2, -20, 20));
    guiAudio.add(guiCamDist.setup("Cam Distance", 1000, 10, 3000));
    guiAudio.add(speedForms.setup("speed abstract forms", 20, 1, 120));
    guiAudio.loadFromFile(guiPath);
    
    
    // Timeline GUI
    //lets you use COMMAND+C and COMMAND+V actions on mac
    ofxTimeline::removeCocoaMenusFromGlut("Camera Track");
    
    timeline.setup();
    timeline.setLoopType(OF_LOOP_NORMAL);
    timeline.setDurationInSeconds(600);
    
//    timeline.addAudioTrack("audio", "265.m1-235 (COMPLETE).wav");
//    timeline.setDurationInSeconds(timeline.getAudioTrack("audio")->getDuration());
    
//    cameraTrack = new ofxTLCameraTrack();
//    cameraTrack->setCamera(easyCam);
//    timeline.addTrack("Camera", cameraTrack);
    //each call to "add keyframes" add's another track to the timeline
    timeline.addCurves("Rotate X", ofRange(0, 360));
    timeline.addCurves("Rotate Y", ofRange(0, 360));
    timeline.addCurves("camDist", ofRange(1,3000));
    timeline.addCurves("Persons Amount", ofRange(0,6));
    timeline.addSwitches("Noise");
    timeline.addCurves("Point Size", ofRange(1,50));
    timeline.addCurves("Displacement Range", ofRange(0,1000));
    timeline.addCurves("Displacement Style", ofRange(1,3));
    timeline.addCurves("Mesh Style", ofRange(1,2.5));

    timeline.addColors("Colors");
    
    
//    cameraTrack->lockCameraToTrack = true;
//    timeline.play();
}

//--------------------------------------------------------------
void ofApp::update() {
   
    
    //--- audio
    if( live){
        fftLive.setThreshold(audioThreshold);
        fftLive.setPeakDecay(audioPeakDecay);
        fftLive.setMaxDecay(audioMaxDecay);
        fftLive.setMirrorData(audioMirror);
        fftLive.update();
    }else{
        fftFile.setThreshold(audioThreshold);
        fftFile.setPeakDecay(audioPeakDecay);
        fftFile.setMaxDecay(audioMaxDecay);
        fftFile.setMirrorData(audioMirror);
        fftFile.update();
    }
    // --- end audio

    
    
//    ofBackground(10, 10, 10);
    
    
    if (bDrawPointcloudOnly || bDrawPointCloud){
        kinect.update();
        angle = kinectAngle;
        kinect.setCameraTiltAngle(angle);

#ifdef USE_TWO_KINECTS
        kinect2.update();
#endif

    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    

    
    ofSetColor(255, 255, 255);
    ofBackgroundGradient(ofColor(255*.15), ofColor::black, OF_GRADIENT_CIRCULAR);

    int width = ofGetWindowWidth();
    int height = ofGetWindowHeight();
    
    if(bDrawAbstractForms){
        
        drawAbstractForms(0,0,width, height);
        
    }
    else if(bDrawAbstractForms2){
        easyCam.begin();
        drawAbstractForms2(0, 0, width, height);
        ofTranslate(-width, -height);
        longitude += 1;
        latitude += 1;
        degree += 0.000051;
//        easyCam.set
        easyCam.setDistance(guiCamDist);
//        if(bOrbit)  easyCam.orbit(longitude,latitude, guiCamDist, ofVec3f(0, 0,0));
        if(bOrbit) easyCam.roll(degree);
        
        easyCam.end();
    }
    else if(bDrawTunnel){
//        ofBackgroundGradient(ofColor::black, ofColor(255*.15), OF_GRADIENT_CIRCULAR);
        drawTunnel(0,0,width,height);
    }
    else if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud(0,0,width,height);
//        easyCam.roll(60);
        easyCam.orbit(timeline.getValue("Rotate X"),timeline.getValue("Rotate Y"),timeline.getValue("camDist"));
        
//        areaLight.draw();
        
        easyCam.end();
        
        
    }else if(bDrawPointcloudOnly){
        easyCam.begin();
        if(bOrbit){
            longitude += 1;
            latitude += 0;
            easyCam.orbit(longitude,latitude, timeline.getValue("camDist"));
        }
        drawTriangleCloud(0, 0, width, height);
        easyCam.end();

    }else if(bDrawSpectroSphere){
        
        drawSprectroSphere(0,0,width, height);
    }else
    {
        // draw from the live kinect
        kinect.drawDepth(10, 10, 400, 300);

     }
    

    
    
    
    // draw instructions
    if(0) {
        ofSetColor(255, 255, 255);
        stringstream reportStream;
    

        if(kinect.hasAccelControl()) {
            reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
            << ofToString(kinect.getMksAccel().y, 2) << " / "
            << ofToString(kinect.getMksAccel().z, 2) << endl;
        } else {
            reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
            << "motor / led / accel controls are not currently supported" << endl << endl;
        }
    
        reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
        << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
        << "set near threshold " << nearThreshold << " (press: + -)" << endl
        << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
        << ", fps: " << ofGetFrameRate() << endl
        << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
    
        if(kinect.hasCamTiltControl()) {
            reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
            << "press 1-5 & 0 to change the led mode" << endl;
        }
    
        ofDrawBitmapString(reportStream.str(), 20, 652);
    }

}


//--------------------------------------------------------------
void ofApp::drawGui(ofEventArgs & args){
    ofBackground(0);
    
    if(0){
        if (live){
            fftLive.draw(30,500);
        }else{
            fftFile.draw(30, 500);
        }
    }

    timeline.draw();
    guiAudio.draw();

}

void ofApp::drawPointCloud(int _x, int _y, int _width, int _height) {

    ofSetColor(255);
    
    int w = _width; //640;
    int h = _height; //480;
    ofMesh mesh;
    
    if (timeline.getValue("Mesh Style") < 2.)
        mesh.setMode(OF_PRIMITIVE_POINTS);
    else if(timeline.getValue("Mesh Style") <= 3.)
        mesh.setMode(OF_PRIMITIVE_LINE_STRIP);
    else
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    
    int step = 3;
    
    
    for(int y = 1; y < h; y += step) {
        for(int x = 1; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) < kinectFarThresh && kinect.getDistanceAt(x,y) > kinectNearThresh) {
//                mesh.addColor(ofColor(255, 255, 255));
                if(timeline.isSwitchOn("Noise"))
//                    mesh.addColor(ofFloatColor(ofRandomuf()*.8));
                    mesh.addColor(ofFloatColor(ofRandomuf()*.9,ofRandomuf()*.9,ofRandomuf()*.9));
                else
                    mesh.addColor(timeline.getColor("Colors"));
                
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(timeline.getValue("Point Size"));
    
    // ---- audio
    meshWarped = mesh;
    vector<ofVec3f> & vertsOriginal = mesh.getVertices();
    vector<ofVec3f> & vertsWarped = meshWarped.getVertices();
    int numOfVerts = mesh.getNumVertices();
    
    float * audioData = new float[numOfVerts];
    
    if(live){
        fftLive.getFftPeakData(audioData, numOfVerts);
    }else{
        fftFile.getFftPeakData(audioData, numOfVerts);
    }
    
    
    float meshDisplacement = cloudDisplacement;
    meshDisplacement = timeline.getValue("Displacement Range");
    

    for(int i=0; i<numOfVerts; i++) {
        float audioValue = audioData[i];
        ofVec3f & vertOriginal = vertsOriginal[i];
        ofVec3f & vertWarped = vertsWarped[i];
        
        ofVec3f direction = vertOriginal.getNormalized();
        ofVec3f randDir = direction.getRotated(90, ofVec3f(ofRandom(1), ofRandom(1), ofRandom(1)));
        ofVec3f transDir = direction.getRotated(90, ofVec3f(1,0,0));

        if (timeline.getValue("Displacement Style") < 2.)
            vertWarped = vertOriginal + direction * meshDisplacement * audioValue;
        else if (timeline.getValue("Displacement Style") < 3.)
            vertWarped = vertOriginal +  transDir * meshDisplacement * audioValue;
        else
            vertWarped = vertOriginal +  randDir * meshDisplacement * audioValue;

        
    }
    delete[] audioData;
    
    // end audio

    personsDist = 500;
    float maxPersons = timeline.getValue("Persons Amount");
    
    for (int persons = 0; persons <= maxPersons; persons++){
        ofPushMatrix();
        // the projected points are 'upside down' and 'backwards'
        //ofTranslate(1000*(persons/maxPersons), -2000*(persons/maxPersons), -1000*(persons/maxPersons)); // center the points a bit
        
//        ofRotate(30,0,0,1);
        ofRotate(kinectAngle, 1, 0, 0);
        
        if (maxPersons < 1.){
            ofScale(1, -1, -1);
            ofTranslate(0 , 0, -2000); // center the points a bit -1000
            ofEnableDepthTest();
//            ofRotate(-30,0,1,0);
//            ofRotate(-30,1,0,0);

            meshWarped.drawVertices();
            ofDisableDepthTest();
        }else{
            ofScale(0.5, -0.5, -0.5);
//            ofRotate(30,0,1,0);


            ofTranslate(-personsDist * persons, -personsDist* persons, -personsDist * persons); // center the points a bit
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
        
            ofTranslate(personsDist * persons, -personsDist * persons, -personsDist * persons);
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
        
            ofTranslate(-personsDist * persons, personsDist * persons, -personsDist * persons);
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
        
            ofTranslate(personsDist * persons, personsDist * persons, -personsDist * persons);
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
            
            ofTranslate(-personsDist * persons, -personsDist* persons, personsDist * persons); // center the points a bit
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
            
            ofTranslate(personsDist * persons, -personsDist * persons, personsDist * persons);
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
            
            ofTranslate(-personsDist * persons, personsDist * persons, personsDist * persons);
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
            
            ofTranslate(personsDist * persons, personsDist * persons, personsDist * persons);
            ofEnableDepthTest();
            meshWarped.drawVertices();
            ofDisableDepthTest();
            
            
            
        }
        ofPopMatrix();
            
    }
    
    
}

void ofApp::drawTriangleCloud(int _x, int _y, int _width, int _height) {
    int w = _width; //640;
    int h = _height; //480;
    int i = 0;
    ofMesh mesh;
    
    if (bDrawLines)
        mesh.setMode(OF_PRIMITIVE_LINES);
    else
        mesh.setMode(OF_PRIMITIVE_POINTS);
    
    
    int step = 3;
    for(int y = 1; y < h; y += step) {
        for(int x = 1; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) < kinectFarThresh && kinect.getDistanceAt(x,y) > kinectNearThresh) {
                //mesh.addColor(kinect.getColorAt(x,y));
                if (bDrawNoise)
                    mesh.addColor(ofFloatColor(ofRandomuf()*.9,ofRandomuf()*.9,ofRandomuf()*.9));
                else
                   // mesh.addColor(ofColor(0,255,0));
                    mesh.addColor(timeline.getColor("Colors"));
                
                
                //// add something
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));

            }
        }
    }

    
    
    glPointSize(timeline.getValue("Point Size"));
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit ofTranslate(0, 0, -1);
    ofEnableDepthTest();
    //    mesh.drawVertices();
    //    mesh.drawWireframe();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}


void ofApp::drawAbstractForms(int _x, int _y, int _width, int _height){
    ofSetColor(255);
    bool isSignal = false;

    int numOfVerts = 20;
    
    int maxDistance = _width/3;
    
    counter++;
    
    
        float * audioData = new float[numOfVerts];
        fftLive.getFftPeakData(audioData, numOfVerts);
    
    
    // ofLogNotice() << "time: " << ofGetElapsedTimeMicros() << "ms";
    
    
    if(spectrumRects){
        for(int i=1; i<numOfVerts; i++) {
            float audioValue = audioData[i];
            
            degree += 0.002;
            
            if(bOrbit)   ofRotate(degree, 1, 0.1, 0.2);

            int rectSize = 50;
            
            
            
            
            ofDrawBox((_width/2)-30*i, (_height/2) - 30*1, audioValue*rectSize, audioValue*rectSize);
            ofDrawBox((_width/2)+30*i, (_height/2) + 30*1, audioValue*rectSize, audioValue*rectSize);
            ofDrawBox((_width/2)-30*i, (_height/2) + 30*1, audioValue*rectSize, audioValue*rectSize);
            ofDrawBox((_width/2)+30*i, (_height/2) - 30*1, audioValue*rectSize, audioValue*rectSize);
            
            ofDrawBox((_width/2), (_height/2) - 30*i, audioValue*rectSize, audioValue*rectSize);
            ofDrawBox((_width/2), (_height/2) + 30*i, audioValue*rectSize, audioValue*rectSize);
            ofDrawBox((_width/2), (_height/2) + 30*i, audioValue*rectSize, audioValue*rectSize);
            ofDrawBox((_width/2), (_height/2) - 30*i, audioValue*rectSize, audioValue*rectSize);
            
            
//            ofSetColor( ofColor::yellowGreen.getLerped(ofColor::blueViolet, 1.0*i/numOfVerts) );
            ofSetColor( ofColor::green.getLerped(ofColor::blue, 1.0*i/numOfVerts) );
            
        }
    }
    
    
    
    if (counter % speedForms == 1 || bNewForms == true){
        for(int i=1; i<numOfVerts/2; i++) {
            float audioValue = audioData[i];
   
            if (audioValue > audioThreshold){
           // ofDrawRectangle((_width/2)-(audioValue*500/2), (_height/2)-(audioValue*500/2), 500*audioValue, 500*audioValue);
                rectWidth   = ofRandom(1, 500);//500*audioValue;
                rectHeight  = rectWidth;//500*audioValue;
                rectPosX    = (_width/2)-(rectWidth/2) + ofRandom(-maxDistance,maxDistance);
                rectPosY    = (_height/2)-(rectHeight/2) + ofRandom(-maxDistance,maxDistance);
            }
        }
    
        ofPushMatrix();

//        ofDrawRectangle(rectPosX, rectPosY, rectWidth, rectHeight);
//    ofDrawRectangle(ofRandom(1,_width), ofRandom(1,_height), 50, 50);
    
        for(int i=numOfVerts/2; i<numOfVerts; i++) {
            float audioValue = audioData[i];
            if (audioValue > audioThreshold){
                // ofDrawRectangle((_width/2)-(audioValue*500/2), (_height/2)-(audioValue*500/2), 500*audioValue, 500*audioValue);
                rectWidth   = ofRandom(1, 500);//500*audioValue;
                rectHeight  = rectWidth;//500*audioValue;
                rectPosX    = (_width/2)-(rectWidth/2) + ofRandom(-maxDistance,maxDistance);
                rectPosY    = (_height/2)-(rectHeight/2) + ofRandom(-maxDistance,maxDistance);
            
            
            }
        }
        
        randTr = ofRandom(-300,300);
        bNewForms = false;
    
    }
        delete[] audioData;

    if(randRects){
        ofSetColor(255);

        ofDrawRectangle(rectPosX, rectPosY, rectWidth, rectHeight);
        ofDrawRectangle(rectPosX+(randTr/2), (rectPosY/2)-randTr, rectWidth/2, rectHeight/2);
        ofDrawRectangle(rectPosX*2, rectPosY/2, rectWidth/4, rectHeight/4);
        ofDrawRectangle(rectPosX/3, rectPosY*5, rectWidth/6, rectHeight/6);
        
        int triPosX = rectPosX + randTr;
        int triPosY = rectPosY + randTr;
        int triWidth = _width + randTr;
        int triHeight = rectHeight + randTr;
    
//        ofDrawTriangle(ofVec2f(triPosX, triPosY), ofVec2f((triPosX - rectWidth), (triPosX )),  ofVec2f(triPosY, triPosY-triHeight));
        ofDrawTriangle(ofVec2f(rectPosX/2, rectPosY*1.5), ofVec2f(rectPosY/1.5 - rectWidth, rectPosY*2),  ofVec2f(rectPosY, rectPosY-rectPosY));

    }
    ofPopMatrix();
    
}



void ofApp::drawAbstractForms2(int _x, int _y, int _width, int _height){
    
    ofSetColor(255);
    
    int maxPoints = 60;
    
    int maxWidth = 100;
   

//    bNewForms = true;
    

    
    if(bNewForms){
        posx.clear();
        posy.clear();
        posz.clear();
        sizex.clear();
        sizey.clear();
        sizez.clear();
        
        pointAmount = (int)ofRandom(1, maxPoints);
        
        for(int i=0; i<pointAmount; i++){
            posx.push_back(ofRandom(0, _width));
            posy.push_back(ofRandom(0, _height));
            posz.push_back(ofRandom(0, 500));
            sizex.push_back(ofRandom(maxWidth));
            sizey.push_back(ofRandom(maxWidth));
            sizez.push_back(ofRandom(maxWidth));
        }
        bNewForms = false;
    }
    
        for(int i=0; i<pointAmount; i++){
            if(bDrawNoise)  ofSetColor(ofRandom(255),ofRandom(255),ofRandom(255));
            
            ofDrawBox(posx[i], posy[i], posz[i], sizex[i], sizey[i], sizez[i]);
//            ofDrawRectangle(posx[i], posy[i], sizex[i], sizey[i]);
        }
    
    
//    ofTranslate(-_width, -_height);
}





void ofApp::drawSprectroSphere(int _x, int _y, int _width, int _height){

    
//    //Update Function
//    fftLive.setThreshold(audioThreshold);
//    fftLive.setPeakDecay(audioPeakDecay);
//    fftLive.setMaxDecay(audioMaxDecay);
//    fftLive.setMirrorData(audioMirror);
//    fftLive.update();

    
//    meshSpecWarped.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

    
    //---------------------------------------------------------- dispacing mesh using audio.
    vector<ofVec3f> & vertsOriginal = meshSpecOrig.getVertices();
    vector<ofVec3f> & vertsWarped = meshSpecWarped.getVertices();
    int numOfVerts = meshSpecOrig.getNumVertices();
    //    numOfVerts = _width/4;//20;//flowWidth/8;
    
    float * audioData = new float[numOfVerts];
    fftLive.getFftPeakData(audioData, numOfVerts);
    
    float meshDisplacement = cloudDisplacement;
    
    for(int i=0; i<numOfVerts; i++) {
        float audioValue = audioData[i];
        ofVec3f & vertOriginal = vertsOriginal[i];
        ofVec3f & vertWarped = vertsWarped[i];
        
        ofVec3f direction = vertOriginal.getNormalized();
        vertWarped = vertOriginal + direction * meshDisplacement * audioValue;
       
//        ofColor color((int)ofRandom(200, 256.0f), (int)ofRandom(200, 256.0f), (int)ofRandom(200, 256.0f));
//        meshSpecWarped.addIndex(i);
//        meshSpecWarped.addIndex(i + 1);
//        meshSpecWarped.addIndex(i + 2);
//        meshSpecWarped.addIndex(i);
//        meshSpecWarped.addColor(color);
//        meshSpecWarped.addColor(color);
//        meshSpecWarped.addColor(color);
    }

    
    delete[] audioData;
    

    
    ////////
    ofPushMatrix();
    ofPushStyle();
    
    ofEnableAlphaBlending();
    ofDisableAntiAliasing();
    
    int richtung = 1;
    
    
//    fboTest.clear();
//    fboTest.begin();
    camMesh.begin();
    meshSpecWarped.drawWireframe();
//    meshWarped.drawVertices();
    

//    if(camDist > 1000.0 ) {
//        camDist -= 2;
//        richtung = -1;
//    }
//    else if(camDist < 1000.0 && richtung < 0) camDist -= 2;
//    else if ( camDist < 200 ) {
//        camDist += 2;
//        richtung = 1;
//    }
//    else camDist += 2;
    
//    camDist = abs(sin(ofGetElapsedTimeMicros()/2000)*100)+400;
    
    
    angleH += 0.5;
    angleV += 0.5;
    
    camMesh.orbit(angleH, angleV, camDist);
    camMesh.end();
//    fboTest.draw(0,0, _width, _height);
//    fboTest.end();
    
    
    ofEnableAntiAliasing();
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    
    // velocity Field mit Optical Flow
    if(0){
        velocityField.setVelocity(opticalFlow.getOpticalFlowDecay());
        velocityField.draw(0, 0, _width, _height);
    }

    // field mit fbo
    if(0){
        velocityField.setVelocity(fboTest.getTexture());
        velocityField.draw(0, 0, _width, _height);
    }
    
    
    if (bShowScalar) {
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        
        fluidSimulation.addVelocity(fboTest.getTexture());
        //        fluidSimulation.addDensity(fboTest.getTexture());
        displayScalar.setSource(fluidSimulation.getVelocity());
        displayScalar.draw(0, 0, _width, _height);
        
    }
    
    ofPopMatrix();
    ofPopStyle();
}


void ofApp::drawTunnel(int _x, int _y, int _width, int _height){
    
    int maxTriangles = 200;
    int range = 50;
    ofVec2f pos1, pos2, pos3;
    float midX = float(_width/2);
    float midY = float(_height/2);
    float size;
//    float triRotation = 1;
    float audioValue = 0;
    

    // auf audio reagieren?
    if(0){
        int numOfVerts = 20;
        float * audioData = new float[numOfVerts];
        fftLive.getFftPeakData(audioData, numOfVerts);
    
        for (int i=0; i<numOfVerts; i++){
            audioValue += audioData[i];
        }
    
        if (audioValue > audioThreshold){
            speed += speedTunnel * audioValue;
        }else
            speed += 0;
    }else {
        // get speed from guiAudio:
        speed += speedTunnel;
    }
    
    
    
    ofPushMatrix();
    
    // wachsend
    if (speed > 0 ){
        for (int i=0; i<maxTriangles; i++){
            size = i * range + speed; // (ofGetElapsedTimef()*100)
        
            if (size > 100)  size -= 100;
            if (speed > 100) speed -= 100;
        
            pos1.set(midX, midY+(size/2));
            pos2.set(midX-(size/2), midY-(size/2));
            pos3.set(midX+(size/2), midY-(size/2));
        
            if(i%2)
                ofSetColor(255);
            else
                ofSetColor(2);
        
            ofDrawTriangle(pos1, pos2, pos3);
        }
    }
    else{ // schrumpfend
        for (int i=maxTriangles; i>0; i--){
            size = (i * range) + speed; // (ofGetElapsedTimef()*100)
            
            if (size < 1)  size += 100;
            if (speed < 1) speed += 100;
            
            pos1.set(midX, midY+(size/2));
            pos2.set(midX-(size/2), midY-(size/2));
            pos3.set(midX+(size/2), midY-(size/2));
            
            if(i%2)
                ofSetColor(2);
            else
                ofSetColor(255);
            
            ofDrawTriangle(pos1, pos2, pos3);
        }
    }
    
    ofPopMatrix();
}


//--------------------------------------------------------------
void ofApp::exit() {
    	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case ' ':
            break;
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case'l':
            bDrawLines = !bDrawLines;
            break;
            
        case 'k':
            bDrawNoise = !bDrawNoise;
            break;
            
        case'o':
            bOrbit = !bOrbit;
            break;
        
        case'i':
            bNewForms = !bNewForms;
            break;
            
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'q':
            bShowScalar = !bShowScalar;
            break;

            
        case 'c':
//            kinect.setCameraTiltAngle(0); // zero the tilt
//            kinect.close();
            break;
            
        case '1':
            
            break;
            
        case '2':
            bDrawPointcloudOnly = true;
            bDrawPointCloud = false;
            bDrawAbstractForms = false;
            bDrawTunnel = false;
            bDrawAbstractForms2 = false;

            break;
            
        case '3':
            bDrawAbstractForms = true;
            bDrawPointCloud = false;
            bDrawPointcloudOnly = false;
            bDrawTunnel = false;
            counter = 1;
            break;
            
        case '4':
            bDrawTunnel = true;
            bDrawPointCloud = false;
            bDrawPointcloudOnly = false;
            bDrawAbstractForms = false;
            bDrawAbstractForms2 = false;
            speed = 0;
            break;
            
        case '5':
            bDrawAbstractForms2 = true;
            bDrawPointCloud = false;
            bDrawPointcloudOnly = false;
            bDrawAbstractForms = false;
            bDrawTunnel = false;
            degree = 1;
            break;
            
        case '6':
            bDrawSpectroSphere = true;
            bDrawPointCloud = false;
            bDrawPointcloudOnly = false;
            bDrawAbstractForms = false;
            bDrawAbstractForms2 = false;
            bDrawTunnel = false;

            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    
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
