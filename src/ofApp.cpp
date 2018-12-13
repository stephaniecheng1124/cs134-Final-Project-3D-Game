//
//   CS 134 - Midterm Starter File - Fall 2018
//
//
//   This file contains all the necessary startup code for the Midterm problem Part II
//   Please make sure to you the required data files installed in your $OF/data directory.
//
//                                             (c) Kevin M. Smith  - 2018
//Stephanie Cheng, Nicholas Gadjali - cs134

#include "ofApp.h"
#include "Util.h"
#include "Octree.h"
#include "box.h"

//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){
    
    //Stephanie
    
    // texture loading
    ofDisableArbTex();     // disable rectangular textures
    
    // load textures
    if (!ofLoadImage(particleTex, "images/dot.png")) {
        cout << "Particle Texture File: images/dot.png not found" << endl;
        ofExit();
    }
    
    // load the shader
    #ifdef TARGET_OPENGLES
        shader.load("shaders_gles/shader");
    #else
        shader.load("shaders/shader");
    #endif
    
    //Set particle color to red first
    particleColor = ofColor(255, 98, 20);
    
    //Stephanie
    gui.setup();
    gui.add(gravity.setup("Gravity", 0, 0, 2));
    gui.add(restitution.setup("Restitution", 0, 0, 1));

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;
    
    //Stephanie
    bRoverLoaded = false;
    bTerrainSelected = true;
    bCollision = false;
    bSideCollision = false;
    bSoundPlaying = false;
    bDrawBounding = false;
    
    
    engineSound.load("sounds/wind.mp3");
    engineSound.setLoop(true);
    
	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
    
    ofSetVerticalSync(true);       //Stephanie
    ofEnableSmoothing();
    ofEnableDepthTest();
    
    
	cam.disableMouseInput();

	// set current camera;
	//
	theCam = &cam;
	
	ofSetVerticalSync(true);
	ofEnableSmoothing();
	ofEnableDepthTest();

	// load BG image
	//
	bBackgroundLoaded = backgroundImage.load("images/starfield-plain.jpg");


	// setup rudimentary lighting
	initLightingAndMaterials();
    
    mars.loadModel("geo/smallTerrain_singleMesh_final.obj");		//single mesh for octree
    marsTextured.loadModel("geo/smallTerrain_final.obj");            //textured model w/ lots of meshes
    mars.setScaleNormalization(false);
	marsTextured.setScaleNormalization(false);
    //mars.setScale(.025, .025, .025); //breaks octree position!
    boundingBox = meshBounds(mars.getMesh(0), 1);
    cout << "num meshes land: " << mars.getMeshCount() << endl;
    
    //Build the octree w/ 9 levels                    ////////////////////////////////////////////Stephanie
    myTree.create(mars.getMesh(0), 7);
    
    //Turn the mesh over                              //Stephanie
    mars.setRotation(0, 180, 0, 0, 1);
	marsTextured.setRotation(0, 180, 0, 0, 1);

	// load lander model
	//    
	if (lander.loadModel("geo/ship_singleMesh_assem.obj")) {
		landerTextured.loadModel("geo/ship_assem.obj");
		lander.setScaleNormalization(false);
		landerTextured.setScaleNormalization(false);
        lander.setRotation(0, 180, 0, 0, 1);
		landerTextured.setRotation(0, 180, 0, 0, 1);
        
		int scale = 1;
		lander.setScale(scale, scale, scale);
		landerTextured.setScale(scale, scale, scale);
		
		lander.setPosition(0,15,-10);
		landerTextured.setPosition(lander.getPosition().x, lander.getPosition().y, lander.getPosition().z);
        landerBox = meshBounds(lander.getMesh(0), .1);
		bLanderLoaded = true;
        
        cout << "num vert: " << lander.getMesh(0).getNumVertices() << endl;
        cout << "num lander meshes: " << lander.getMeshCount() << endl;
        cout << "Mesh 0 name: " << lander.getMeshNames()[0] << endl;
	}
	else {
		cout << "Error: Can't load model" << "geo/ship_singleMesh_assem.obj" << endl;
		ofExit(0);
	}
    
	//////////Camera Setup: Nicholas

	trackingCam.setNearClip(0.1);
	trackingCam.setFov(75.5);
	trackingCam.setPosition(ofVec3f(0, 20, 40));
	trackingCam.lookAt(lander.getPosition());

	downCam.setNearClip(3.3);
	downCam.setFov(65.5);
	downCam.setPosition(lander.getPosition());
	downCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

	frontCam.setNearClip(4.8);
	frontCam.setFov(65.5);
	frontCam.setPosition(lander.getPosition());
	float forward = lander.getPosition().x == 0 ? 1 : lander.getPosition().x;
	frontCam.lookAt(ofVec3f(abs(forward), lander.getPosition().y, lander.getPosition().z) * ofVec3f(-2, 1, 1));
    
    
    /////////////////////////////////Set up the emitter that has only 1 particle
    
    //Create Forces
    turbForce = new TurbulenceForce(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0));
    gravityForce = new GravityForce(ofVec3f(0, -gravity, 0));  //Stephanie changed for slider
    thrustForceLunar = new ThrustForce(ofVec3f(0, 0, 0));
    impulseForce = new ImpulseForce(); //Create new impulseforce to repel the ship
    
    //Add Forces
    emitter.sys->addForce(turbForce);
    emitter.sys->addForce(gravityForce);
    emitter.sys->addForce(thrustForceLunar);
    emitter.sys->addForce(impulseForce);
    emitter.setPosition(lander.getPosition());
    
    //Emitter settings
    emitter.setVelocity(ofVec3f(0, 0, 0));
    emitter.setOneShot(true);
    emitter.setEmitterType(RadialEmitter);
    emitter.setGroupSize(1);
    emitter.setLifespan(100000);
    
    
    
    
    //////////////////////////////////Set up emitter 2 & 3///////////////////////////////////
    
    //Create Forces
    turbForce2 = new TurbulenceForce(ofVec3f(-3, -3, -3), ofVec3f(3, 3, 3));
    gravityForce2 = new GravityForce(ofVec3f(0, 0, 0));
    thrustForceLunar2 = new ThrustForce(ofVec3f(10, 0, 0));
    
    //Add forces to emitter2
    engineEmitter.sys->addForce(turbForce2);
    engineEmitter.sys->addForce(gravityForce2);
    engineEmitter.sys->addForce(thrustForceLunar2);
    
    //Emitter2 settings
    engineEmitter.setVelocity(ofVec3f(0, 0, 0));
    engineEmitter.setOneShot(true);
    engineEmitter.setGroupSize(30);
    engineEmitter.setRandomLife(true);
    engineEmitter.setLifespanRange(ofVec2f(0.5, 0.8));
    engineEmitter.setPosition(lander.getPosition() + ofVec3f(4.3, -2.60,1.25));
    //cout << "POS: " << lander.getPosition() << endl;
    engineEmitter.setEmitterType(DiscEmitter);
    
    
    //Add forces to emitter2 -Stephanie
    engineEmitter2.sys->addForce(turbForce2);
    engineEmitter2.sys->addForce(gravityForce2);
    engineEmitter2.sys->addForce(thrustForceLunar2);
    
    //Emitter3 settings -Stephanie
    engineEmitter2.setVelocity(ofVec3f(0, 0, 0));
    engineEmitter2.setOneShot(true);
    engineEmitter2.setGroupSize(30);
    engineEmitter2.setRandomLife(true);
    engineEmitter2.setLifespanRange(ofVec2f(0.5, 0.8));
    engineEmitter2.setPosition(lander.getPosition() + ofVec3f(4.3, -2.60,-1.25));
    engineEmitter2.setEmitterType(DiscEmitter);
    
    ///////////////////////////////////Spawn particle////////////////////////////////////
    
    //Spawn the single particle that will control the position of the lander
    emitter.spawn(ofGetElapsedTimef());
    
}

//---------------------------------------------------------------------------------------------------------------------------


void ofApp::update() {
    
    // live update of forces  (with sliders)  Stephanie
    //
    
    if (!bCollision) {
        gravityForce->set(ofVec3f(0, -gravity, 0));
    }
    
    
    //Update both emitters
    emitter.sys -> update();
    if (emitter.sys -> particles.size() > 0) {
        lander.setPosition(emitter.sys -> particles[0].position.x,emitter.sys -> particles[0].position.y,emitter.sys -> particles[0].position.z);
    }
    engineEmitter.setPosition(lander.getPosition()+ ofVec3f(4.3, -2.60,1.25)); //Attach emitter to thruster and follow it
    engineEmitter.update();
    
    engineEmitter2.setPosition(lander.getPosition()+ ofVec3f(4.3, -2.60,-1.25)); //Attach emitter2 to thruster2 and follow it
    engineEmitter2.update();
    
    //Does not call collision detect if movement is too small (caused by turbulence force)
    
    //only check bottom collision if the lander is moving downwards --Stephanie
    if (emitter.sys->particles[0].velocity.y < 0.5) {
        checkCollision();
    }
    
    //Check front bumper --Stephanie
    if (emitter.sys->particles[0].velocity.x < -0.5) {
        checkFrontCollision();
    }
    //Check back bumper --Stephanie
    if (emitter.sys->particles[0].velocity.x > 0.5) {
        checkBackCollision();
    }
    //Check left bumper --Stephanie
    if (emitter.sys->particles[0].velocity.z < -0.5) {
        checkLeftCollision();
    }
    //Check right bumper --Stephanie
    if (emitter.sys->particles[0].velocity.z > 0.5) {
        checkRightCollision();
    }

    
    /////////Camera position updating: --------------------------- Nicholas
	trackingCam.lookAt(lander.getPosition());
	downCam.setPosition(lander.getPosition());
	downCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

	frontCam.setPosition(lander.getPosition());
	float forward = lander.getPosition().x == 0 ? 1 : lander.getPosition().x;
	frontCam.lookAt(ofVec3f(abs(forward), lander.getPosition().y, lander.getPosition().z) * ofVec3f(-2, 1, 1));

	//Update textured lander location - Nicholas
	landerTextured.setPosition(lander.getPosition().x, lander.getPosition().y, lander.getPosition().z);
}

//---------------------------------------------------------------------------------------------------------------------------

// load vertex buffer in preparation for rendering
void ofApp::loadVbo() {
    if (engineEmitter.sys->particles.size() < 1) return;
    
    vector<ofVec3f> sizes;
    vector<ofVec3f> points;
    for (int i = 0; i < engineEmitter.sys->particles.size(); i++) {
        points.push_back(engineEmitter.sys->particles[i].position);
        sizes.push_back(ofVec3f(10));
    }
    for (int i = 0; i < engineEmitter2.sys->particles.size(); i++) {
        points.push_back(engineEmitter2.sys->particles[i].position);
        sizes.push_back(ofVec3f(10));
    }
    //upload the data to the vbo
    int total = (int)points.size();
    vbo.clear();
    vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
    vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    //Stephanie
    loadVbo();

	//	ofBackgroundGradient(ofColor(20), ofColor(0));   // pick your own backgroujnd
	//	ofBackground(ofColor::black);
	if (bBackgroundLoaded) {
		ofPushMatrix();
		ofDisableDepthTest();
		ofSetColor(50, 50, 50);
		ofScale(2, 2);
		backgroundImage.draw(-200, -100);
		ofEnableDepthTest();
		ofPopMatrix();
	}
    
	theCam->begin();
	ofPushMatrix();
    
    //Stephanie - draw the light
    //keyLight.draw();
    
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
        //Causes frame drop
        mars.drawWireframe();
		if (bLanderLoaded) {
            
			lander.drawWireframe();
		}
        if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
        marsTextured.drawFaces();  //Stephanie, Nicholas: draw TEXTURED not singlemesh!!
		if (bLanderLoaded) {
			landerTextured.drawFaces(); //Nicholas: draw TEXTURED not singlemesh!!

		}
        if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
    
    if (bDisplayPoints) {                // display points as an option
        glPointSize(3);
        ofSetColor(ofColor::green);
        //Causes frame drop
        mars.drawVertices();
    }
    
    // highlight selected point (draw sphere around selected point) ///////////////////////////////////
    if (bPointSelected) {
        ofSetColor(ofColor::darkViolet);
        ofDrawSphere(selectedPoint, .1);
    }
    
    //Debug - contact point
    //Vector3 center2 = landerBox.center();
    //contactPointFront = ofVec3f(center2.x() - landerBox.length()/2, center2.y(), center2.z())    + lander.getPosition();
    //ofDrawSphere(contactPointFront, .9);

    
    //---------------------------------------------------------Draw glowing particles - Stephanie
    //Draw the particles
    //engineEmitter.sys -> draw();
    //engineEmitter2.sys -> draw();
    //emitter.sys -> draw();
    
    glDepthMask(GL_FALSE);
    
    //Draw explosion
    //Fireworks red explosion
    ofSetColor(particleColor);
    
    // this makes everything look glowy :)
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofEnablePointSprites();
    
    // begin drawing the particles
    shader.begin();
    
    // draw particle emitter here..
    //    emitter.draw();
    particleTex.bind();
    vbo.draw(GL_POINTS, 0, (int)(engineEmitter.sys->particles.size() + engineEmitter2.sys->particles.size()));
    particleTex.unbind();
    
    //  end drawing in the camera
    shader.end();
    ofDisablePointSprites();
    ofDisableBlendMode();
    ofEnableAlphaBlending();
    
    // set back the depth mask
    //
    glDepthMask(GL_TRUE);
    
    //Change color back to normal
    ofSetColor(50, 50, 50);
    
    //--------------------------------------------------------- Draw glowing particles End - Stephanie
    
    //////////////////// Draw the bounding boxes - Stephanie
    ofNoFill();
    ofSetColor(ofColor::white);
    drawBox(boundingBox);
    if (bDrawBounding) {
        drawMovingBox(landerBox, lander.getPosition());
    }
    //ofFill();  -Stephanie
    
    //////////////Draw either the whole octree or only the leaf nodes///////////////
    //myTree.drawLeafNodes(myTree.root);
    //myTree.draw(myTree.root, myTree.numOfLevels, 1);
    ////////////////////////////////////////////////////////////////////////////////
    
	ofPopMatrix();
    
    ofDisableDepthTest();
	theCam->end();

    gui.draw();
    
    
    /////////////////////////// Draw screen data - Stephanie
	
    //Framerate
	string str;
	str += "Frame Rate: " + std::to_string(ofGetFrameRate());
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 205, 15);
    
    //Altitude
    string str2;
    str2 += "Altitude (AGL): " + std::to_string(lander.getPosition().y);
    ofSetColor(ofColor::white);
    ofDrawBitmapString(str2, ofGetWindowWidth() - 205, 30);
    

}


// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {

	switch (key) {
    case 'B':
    case 'b':
        toggleDrawBoundingBox();  //Stephanie
        break;
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
        case 'i': {  //Test impulse force - Stephanie
        ofVec3f vel = emitter.sys->particles[0].velocity;
        impulseForce -> apply(-60 * vel * (restitution + 1));
        }
        break;
	case 'P':
	case 'p':
		break;
	case 'r':
		cam.reset();
		break;
	case 's':
		savePicture();
		break;
	case 't':
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case 'w':
		toggleWireframeMode();
		break;
	case OF_KEY_F1:
		theCam = &cam;
		break;
	case OF_KEY_F2:
		theCam = &trackingCam; //Nicholas
		break;
	case OF_KEY_F3:
		theCam = &frontCam;  //Nicholas
		break;
	case OF_KEY_F4:
		theCam = &downCam;  //Nicholas
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;
	case OF_KEY_UP:
        playEngineSound();
        bSoundPlaying = true;
        bCollision = false;
        emitter.sys->reset();
        impulseForce -> set(ofVec3f(0, 0, 0)); //Reset the impulse force
        turbForce -> set(ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)); //Stephanie
        if (bCtrlKeyDown) {
            thrustForceLunar ->set(ofVec3f(0, 0, 4));
        }
        else {
            thrustForceLunar ->set(ofVec3f(0, 4, 0));
        }
        engineEmitter.sys->reset();
        engineEmitter.start();
        engineEmitter2.sys->reset();
        engineEmitter2.start();
		break;
	case OF_KEY_DOWN:
        if (!bCollision) {
            playEngineSound();
            bSoundPlaying = true;
            emitter.sys->reset();
            impulseForce -> set(ofVec3f(0, 0, 0)); //Reset the impulse force
            turbForce -> set(ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)); //Stephanie
            if (bCtrlKeyDown) {
                thrustForceLunar ->set(ofVec3f(0, 0, -4));
            }
            else {
                thrustForceLunar ->set(ofVec3f(0, -4, 0));
            }
            engineEmitter.sys->reset();
            engineEmitter.start();
            engineEmitter2.sys->reset();
            engineEmitter2.start();
        }
		break;
    case 'z':
    case OF_KEY_CONTROL:
        bCtrlKeyDown = true;
        break;
	case OF_KEY_LEFT:
        playEngineSound();
        bSoundPlaying = true;
        emitter.sys->reset();
        impulseForce -> set(ofVec3f(0, 0, 0)); //Reset the impulse force
        thrustForceLunar ->set(ofVec3f(-4, 0, 0));
        if (!bCollision) {turbForce -> set(ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1));} //Stephanie
        engineEmitter.sys->reset();
        engineEmitter.start();
        engineEmitter2.sys->reset();
        engineEmitter2.start();
		break;
	case OF_KEY_RIGHT:
        playEngineSound();
        bSoundPlaying = true;
        emitter.sys->reset();
        impulseForce -> set(ofVec3f(0, 0, 0)); //Reset the impulse force
        thrustForceLunar ->set(ofVec3f(4, 0, 0));
        if (!bCollision) {turbForce -> set(ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1));}  //Stephanie
        engineEmitter.sys->reset();
        engineEmitter.start();
        engineEmitter2.sys->reset();
        engineEmitter2.start();
		break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

//Stephanie
void ofApp::toggleDrawBoundingBox() {
    bDrawBounding = !bDrawBounding;
}


void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

	switch (key) {
	
	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
   
	case OF_KEY_SHIFT:
		break;
    case 'i':
        impulseForce -> set(ofVec3f(0, 0, 0));
        break;
    case OF_KEY_UP:
    case OF_KEY_DOWN:
    case OF_KEY_LEFT:
    case OF_KEY_RIGHT:
        engineSound.stop();
        bSoundPlaying = false;
        thrustForceLunar ->set(ofVec3f(0, 0, 0));
        break;
    case 'z':
    case OF_KEY_CONTROL:
        bCtrlKeyDown = false;
        break;
	default:
		break;

	}
}


//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    
    //reset the  bool every time clicked
    bPointSelected = false;
    
    ofVec3f mouse(mouseX, mouseY);
    ofVec3f rayPoint = cam.screenToWorld(mouse);
    ofVec3f rayDir = rayPoint - cam.getPosition();
    rayDir.normalize();
    //Gives a ray
    Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
                  Vector3(rayDir.x, rayDir.y, rayDir.z));
    
    
    //Start the timer to measure the time for selection
    int startTime = ofGetElapsedTimeMillis();
    
    //If the ray intersects with a leaf node, set the selected point to the first point in that node
    myTree.intersect(ray, myTree.root, selectedNode);
    if (selectedNode.points.size() > 0) {
        bPointSelected = true;
        selectedPoint = mars.getMesh(0).getVertex(selectedNode.points[0]);
    }
    cout << "Select Time: " << ofGetElapsedTimeMillis() - startTime << endl;
    
    //cout << "SelectedNode: " << selectedNode.points.size() << " " << selectedNode.children.size() << endl;
    
    //Does not actually give intersection point.
    //if (level3[1].intersect(ray, -1000, 1000)) cout << "intersects" << endl;
}

//--------------------------------------------------------------


//Used part of professor Smith's example ------------------- Stephanie
void ofApp::checkCollision() {
    Vector3 center = landerBox.center();
    contactPoint = ofVec3f(center.x(), center.y() - landerBox.height()/2, center.z()) + lander.getPosition();
    ofVec3f vel = emitter.sys->particles[0].velocity; //velocity of the lander
    
    //Don't check if ship is moving up or not moving
    if (vel.y >= 0) return;
    cout << "Checking!!!!!" << ofGetElapsedTimeMillis() << endl;
    TreeNode node;
    myTree.pointIntersect(contactPoint, myTree.root, node);
    if (node.points.size() > 0 || lander.getPosition().y <= 6.4) {
        bCollision = true;
        cout << "Intersects!!!!!" << ofGetElapsedTimeMillis() << endl;
        
        //apply impulse force --------- Stephanie
        //ofVec3f vel = emitter.sys->particles[0].velocity;
        impulseForce -> apply(-60 * vel * (restitution + 1));
        
        //Stop gravity from sinking the ship down when it reaches a stop - Stephanie
        if (vel.y <= 0 && vel.y >= -0.05){
            gravityForce ->set(ofVec3f(0, 0, 0));
            emitter.sys->particles[0].velocity.set(0,0,0); //Bring the ship to a halt - Stephanie
            cout << "INNNN------------------> " << endl;
        }
        
        
        //gravityForce ->set(ofVec3f(0, 0, 0));
        cout << "Gravity: " << gravityForce -> gravity.y << "Velo:" << vel.y << endl;
        turbForce -> set(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0)); //Stop ship movement
        
    }
    
}

//--------------------------------------------------------------


//Referenced professor Smith's example ------------------- Stephanie
void ofApp::checkFrontCollision() {
    
    //Stephanie
    Vector3 center = landerBox.center();
    //Setup the frontal contactpoint
    contactPointFront = ofVec3f(center.x() - landerBox.length()/2, center.y(), center.z())    + lander.getPosition();
  
    //Stephanie
    ofVec3f vel = emitter.sys->particles[0].velocity; //velocity of the lander
    
    cout << "Checking Front------------!!!!!" << ofGetElapsedTimeMillis() << endl;
    
    TreeNode node;
    myTree.pointIntersect(contactPointFront, myTree.root, node);
    if (node.points.size() > 0) {
        //apply impulse force --------- Stephanie
        impulseForce -> apply(-60 * vel * (restitution + 1));
        cout << "Bounce-----------&---&--------&------> " << ofGetElapsedTimeMillis() <<  endl;
    }
    
    
}

//--------------------------------------------------------------


//Referenced professor Smith's example ------------------- Stephanie
void ofApp::checkBackCollision() {
    
    //Stephanie
    Vector3 center = landerBox.center();
    //Setup the back contactpoint
    contactPointBack = ofVec3f(center.x() + landerBox.length()/2, center.y(), center.z())    + lander.getPosition();
    //Stephanie
    ofVec3f vel = emitter.sys->particles[0].velocity; //velocity of the lander
    
    
    cout << "Checking back!------------!!!!!" << ofGetElapsedTimeMillis() << endl;
    
    //for (int i = 0; i < 4; i++) {
    TreeNode node;
    myTree.pointIntersect(contactPointBack, myTree.root, node);
    if (node.points.size() > 0) {
        //apply impulse force --------- Stephanie
        impulseForce -> apply(-60 * vel * (restitution + 1));
        cout << "Bounce-----------&---&--------&------> " << ofGetElapsedTimeMillis() <<  endl;
        //break;
    }
    //}
    
}

//--------------------------------------------------------------


//Referenced professor Smith's example ------------------- Stephanie
void ofApp::checkLeftCollision() {
    
    //Stephanie
    Vector3 center = landerBox.center();
    //Setup the various contactpoints
    contactPointLeft = ofVec3f(center.x(), center.y(), center.z() - landerBox.width()/2)    + lander.getPosition();
    //Stephanie
    ofVec3f vel = emitter.sys->particles[0].velocity; //velocity of the lander
    
    
    cout << "Checking Left!!-----!!-------!!!!!" << ofGetElapsedTimeMillis() << endl;
    
    TreeNode node;
    myTree.pointIntersect(contactPointLeft, myTree.root, node);
    if (node.points.size() > 0) {
        //apply impulse force --------- Stephanie
        impulseForce -> apply(-60 * vel * (restitution + 1));
        cout << "Bounce-----------&---&--------&------> " << ofGetElapsedTimeMillis() <<  endl;
    }
    
    
}

//--------------------------------------------------------------


//Referenced professor Smith's example ------------------- Stephanie
void ofApp::checkRightCollision() {
    
    //Stephanie
    Vector3 center = landerBox.center();
    //Setup the various contactpoints
    contactPointRight = ofVec3f(center.x(), center.y(), center.z() + landerBox.width()/2)    + lander.getPosition();
    
    //Stephanie
    ofVec3f vel = emitter.sys->particles[0].velocity; //velocity of the lander
    
    
    cout << "Checking right------&------!!!!!" << ofGetElapsedTimeMillis() << endl;
    
    TreeNode node;
    myTree.pointIntersect(contactPointRight, myTree.root, node);
    if (node.points.size() > 0) {
        //apply impulse force --------- Stephanie
        impulseForce -> apply(-60 * vel * (restitution + 1));
        cout << "Bounce-----------&---&--------&------> " << ofGetElapsedTimeMillis() <<  endl;
    }
    
}

//--------------------------------------------------------------
//Stephanie
void ofApp::playEngineSound() {
    if (bSoundPlaying) return;
    if(!engineSound.isPlaying()) engineSound.play();
}
    

//draw a box from a "Box" class
//
void ofApp::drawBox(const Box &box) {
    Vector3 min = box.parameters[0];
    Vector3 max = box.parameters[1];
    Vector3 size = max - min;
    Vector3 center = size / 2 + min;
    ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
    float w = size.x();
    float h = size.y();
    float d = size.z();
    ofDrawBox(p, w, h, d);
}


//draw a moving box from a "Box" class //-------------Stephanie
//
void ofApp::drawMovingBox(const Box &box, const ofVec3f &offset) {
    Vector3 min = box.parameters[0];
    Vector3 max = box.parameters[1];
    Vector3 size = max - min;
    Vector3 center = size / 2 + min;
    ofVec3f p = ofVec3f(center.x() + offset.x, center.y() + offset.y, center.z() + offset.z);
    float w = size.x();
    float h = size.y();
    float d = size.z();
    ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh & mesh, float scale) {
    int n = mesh.getNumVertices();
    ofVec3f v = mesh.getVertex(0);
    ofVec3f max = v;
    ofVec3f min = v;
    for (int i = 1; i < n; i++) {
        ofVec3f v = mesh.getVertex(i);
        
        if (v.x > max.x) max.x = v.x;
        else if (v.x < min.x) min.x = v.x;
        
        if (v.y > max.y) max.y = v.y;
        else if (v.y < min.y) min.y = v.y;
        
        if (v.z > max.z) max.z = v.z;
        else if (v.z < min.z) min.z = v.z;
    }
    //Stephanie -added scaling factor
    return Box(Vector3(min.x, min.y, min.z)*scale , Vector3(max.x, max.y, max.z)*scale);
}

//  Subdivide a Box into eight(8) equal size boxes, return them in boxList;
//
void ofApp::subDivideBox8(const Box &box, vector<Box> & boxList) {
    Vector3 min = box.parameters[0];
    Vector3 max = box.parameters[1];
    Vector3 size = max - min;
    Vector3 center = size / 2 + min;
    float xdist = (max.x() - min.x()) / 2;
    float ydist = (max.y() - min.y()) / 2;
    float zdist = (max.z() - min.z()) / 2;
    Vector3 h = Vector3(0, ydist, 0);
    
    //  generate ground floor
    //
    Box b[8];
    b[0] = Box(min, center);
    b[1] = Box(b[0].min() + Vector3(xdist, 0, 0), b[0].max() + Vector3(xdist, 0, 0));
    b[2] = Box(b[1].min() + Vector3(0, 0, zdist), b[1].max() + Vector3(0, 0, zdist));
    b[3] = Box(b[2].min() + Vector3(-xdist, 0, 0), b[2].max() + Vector3(-xdist, 0, 0));
    
    boxList.clear();
    for (int i = 0; i < 4; i++)
        boxList.push_back(b[i]);
    
    // generate second story
    //
    for (int i = 4; i < 8; i++) {
        b[i] = Box(b[i - 4].min() + h, b[i - 4].max() + h);
        boxList.push_back(b[i]);
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
    
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    
}


//
//  ScreenSpace Selection Method:
//  This is not the octree method, but will give you an idea of comparison
//  of speed between octree and screenspace.
//
//  Select Target Point on Terrain by comparing distance of mouse to
//  vertice points projected onto screenspace.
//  if a point is selected, return true, else return false;
//
bool ofApp::doPointSelection() {
    
    ofMesh mesh = mars.getMesh(0);
    int n = mesh.getNumVertices();
    float nearestDistance = 0;
    int nearestIndex = 0;
    
    bPointSelected = false;
    
    ofVec2f mouse(mouseX, mouseY);
    vector<ofVec3f> selection;
    
    // We check through the mesh vertices to see which ones
    // are "close" to the mouse point in screen space.  If we find
    // points that are close, we store them in a vector (dynamic array)
    //
    for (int i = 0; i < n; i++) {
        ofVec3f vert = mesh.getVertex(i);
        ofVec3f posScreen = cam.worldToScreen(vert);
        float distance = posScreen.distance(mouse);
        if (distance < selectionRange) {
            selection.push_back(vert);
            bPointSelected = true;
        }
    }
    
    //  if we found selected points, we need to determine which
    //  one is closest to the eye (camera). That one is our selected target.
    //
    if (bPointSelected) {
        float distance = 0;
        for (int i = 0; i < selection.size(); i++) {
            ofVec3f point =  cam.worldToCamera(selection[i]);
            
            // In camera space, the camera is at (0,0,0), so distance from
            // the camera is simply the length of the point vector
            //
            float curDist = point.length();
            
            if (i == 0 || curDist < distance) {
                distance = curDist;
                selectedPoint = selection[i];
            }
        }
    }
    return bPointSelected;
}

// Set the camera to use the selected point as it's new target
//
void ofApp::setCameraTarget() {
    
}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {
    
    //Stephanie - Light setup
    keyLight.setup();
    keyLight.enable();
    keyLight.setAreaLight(2, 5);
    keyLight.setAmbientColor(ofColor(142, 233, 255));
    keyLight.setDiffuseColor(ofColor(209, 245, 255));
    keyLight.setSpecularColor(ofColor(226, 248, 255));

    keyLight.rotate(-90, ofVec3f(1, 0, 0));
    keyLight.rotate(-90, ofVec3f(0, 1, 0));
    keyLight.setPosition(0, 7, 1);

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ .7f, .7f, .7f, 1.0f };

	static float position[] =
	{20.0, 20.0, 20.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
//	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
//	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
//	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
    
    ofVec3f point;
    mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);
    
    if (rover.loadModel(dragInfo.files[0])) {
        rover.setScaleNormalization(false);
        rover.setScale(.005, .005, .005);
        rover.setPosition(point.x, point.y, point.z);
        bRoverLoaded = true;
    }
    else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
    glm::vec3 mouse(mouseX, mouseY, 0);
    ofVec3f rayPoint = cam.screenToWorld(mouse);
    ofVec3f rayDir = rayPoint - cam.getPosition();
    rayDir.normalize();
    return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}
