#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxFFTLive.h"
#include "ofxFFTFile.h"
#include "ofxGui.h"
#include "ofxTimeline.h"
#include "ofxTLCameraTrack.h"
#include "ofxFlowTools.h"






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

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void setupGui();
    void update();
    void draw();
    void drawGui(ofEventArgs & args);
    void exit();
    
    void drawPointCloud(int x, int y, int _width, int _height);
    void drawTriangleCloud(int x, int y, int _width, int _height);
    void drawAbstractForms(int x, int y, int _width, int _height);
    void drawAbstractForms2(int x, int y, int _width, int _height);
    void drawTunnel(int x, int y, int _width, int _height);
    void drawSprectroSphere(int _x, int _y, int _width, int _height);
    
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    
    ofxKinect kinect;
    
#ifdef USE_TWO_KINECTS
    ofxKinect kinect2;
#endif
    
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
    bool bDrawTriangleCloud;
    
    int nearThreshold;
    int farThreshold;
    
    int angle;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    
    float personsDist, camDist, angleH, angleV;
    
    //---- audio visualization
    ofxFFTLive fftLive;
    ofSoundStream soundStream;
    ofxFFTFile fftFile;
    bool live = true;

    
    ofMesh meshOriginal;
    ofMesh meshWarped;
    ofMesh meshSpecOrig;
    ofMesh meshSpecWarped;

    ofxPanel guiAudio;
    ofxSlider<float> audioThreshold;
    ofxSlider<float> audioPeakDecay;
    ofxSlider<float> audioMaxDecay;
    ofxSlider<int> cloudDisplacement;
    ofxToggle audioMirror;
    ofxToggle spectrumRects;
    ofxToggle randRects;
    
    ofxSlider<int>  kinectAngle;
    ofxSlider<int> kinectFarThresh;
    ofxSlider<int> kinectNearThresh;
    ofxSlider<int> speedTunnel;
    ofxSlider<float> guiCamDist;
    ofxSlider<int> speedForms;
    // end audio
    
    // Timeline
    ofxTimeline timeline;
    ofxTLCameraTrack* cameraTrack;
//    ofEasyCam cam;
    vector<ofVec3f> boxes;
    
    ofRectangle makeCameraPoint;
    ofRectangle toggleLockTrack;
    ofVboMesh particles;
    
//  Light
    
    ofLight areaLight;
    
// Ze Fack
    bool bDrawPointcloudOnly;
    bool bDrawAbstractForms;
    bool bDrawAbstractForms2;
    bool bDrawTunnel;
    bool bDrawLines, bDrawNoise, bOrbit, bDrawSpectroSphere;
    bool bNewForms;
    float longitude, latitude, degree;
    
    ofRectangle abstrRect;
    int rectWidth;
    int rectHeight;
    int rectPosX;
    int rectPosY;
    
    int counter;
    int randTr;
    
    float speed;
    
    
// abstractForms2
    int pointAmount;
    vector <ofVec3f> points;
    ofMesh mesh;
    vector <float> posx;
    vector <float> posy;
    vector <float> posz;
    vector <float> sizex;
    vector <float> sizey;
    vector <float> sizez;
    
    // audio mesh sphere
    ofEasyCam camMesh;
    ofFbo fboTest;

    
    // FlowTools
    bool            bShowScalar;
    int					flowWidth;
    int					flowHeight;
    int					drawWidth;
    int					drawHeight;
    
    flowTools::ftOpticalFlow		opticalFlow;
    flowTools::ftVelocityMask		velocityMask;
    flowTools::ftFluidSimulation	fluidSimulation;
    flowTools::ftParticleFlow		particleFlow;
    
    flowTools::ftVelocityField		velocityField;
    flowTools::ftDisplayScalar		displayScalar;




};
