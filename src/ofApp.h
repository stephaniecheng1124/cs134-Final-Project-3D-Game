//
//   CS 134 - Midterm Starter File - Fall 2018
//
//
//   This file contains all the necessary startup code for the Midterm problem Part II
//   Please make sure to you the required data files installed in your $OF/data directory.
//
//                                             (c) Kevin M. Smith  - 2018
#pragma once

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"
#include  "ofxGui.h"
#include "ParticleSystem.h"
#include "ParticleEmitter.h"
#include "box.h"
#include "ray.h"
#include "Octree.h"


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
    
    void toggleSelectTerrain();
    void setCameraTarget();
    bool  doPointSelection();
    void drawBox(const Box &box);
    Box meshBounds(const ofMesh &);
    void subDivideBox8(const Box &b, vector<Box> & boxList);
    
    bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
    
    Box boundingBox;
    ofxAssimpModelLoader mars, rover;
    bool bPointSelected;
    bool bRoverLoaded;
    bool bTerrainSelected;
    
    ofVec3f selectedPoint;
    ofVec3f intersectPoint;
    Octree myTree;
    TreeNode selectedNode;
    
    const float selectionRange = 4.0;

		ofEasyCam cam;
		ofxAssimpModelLoader lander;
		ofLight light;
		ofImage backgroundImage;
		ofCamera *theCam = NULL;
		ofCamera topCam;

		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
	
		bool bBackgroundLoaded = false;
		bool bLanderLoaded = false;
    
        ParticleEmitter emitter;
        ParticleEmitter engineEmitter;
        TurbulenceForce* turbForce;
        GravityForce* gravityForce;
        ThrustForce* thrustForceLunar;
    
        TurbulenceForce* turbForce2;
        GravityForce* gravityForce2;
        ThrustForce* thrustForceLunar2;
    
        //GUI
        ofxPanel gui;
        ofxFloatSlider gravity;
        //ofxFloatSlider restitution;
	

};
