#include "ofApp.h"

// Draw the boid
void Boid::draw() {

	/*ofPushMatrix();
	ofTranslate(mPosition);
	ofScale(scale.x, scale.y, scale.z);
	ofRotateZDeg(heading);
	ofSetColor(ofColor::white);
	ofDrawTriangle(vertices[0], vertices[1], vertices[2]);
	ofPopMatrix();*/

}

// Turbulence
void Boid::applyForce(glm::vec3 force) {
	acceleration += force;
}

// Physics
void Boid::integrate(float maxSpeed, float minSpeed) {

	float fps = ofGetFrameRate();
	float dt = (1.0f / 150); // fps doesn't work for some reason

	// Euler integration
	velocity += acceleration * dt;	// update velocity
	mPosition += velocity * dt;	// update position
	acceleration *= 0;		// reset acceleration

	// Limit the speed
	float speed = glm::length(velocity);
	if (speed > maxSpeed) {
		velocity = glm::normalize(velocity) * maxSpeed;
	}
	if (speed < minSpeed && speed != 0) {
		velocity = glm::normalize(velocity) * minSpeed;
	}

}

// Function to make the boid wander
void Boid::wander(float wanderIntensity) {

	// Randomly change the force a little bit based on random probabaility
	float random = ofRandom(0, 1);
	if (random < 0.0125) {

		wanderForce = glm::vec3(ofRandom(-1, 1), ofRandom(-1, 1), ofRandom(-1, 1));
		wanderForce = glm::normalize(wanderForce);
		wanderForce *= 10000;
		applyForce(wanderForce);
	}

}

// Seperation
void Boid::separation(vector <Boid>& boids, float maxSpeed, float seperateIntensity) {
	int perceptionRadius = 25;
	glm::vec3 steer = glm::vec3(0, 0, 0);
	int total = 0;

	for (int i = 0; i < boids.size(); i++) {
		int d = glm::distance(mPosition, boids[i].mPosition);
		if ((this != &boids[i]) && (d < perceptionRadius)) {
			glm::vec3 diff = mPosition - boids[i].mPosition;
			diff = glm::normalize(diff);
			diff /= (d);
			steer += diff;
			total++;
		}
	}
	if (total > 0) {
		steer /= total;
		if (glm::length(steer) < 1e-4f) {
			return;
		}
		steer = glm::normalize(steer) * maxSpeed;
		steer -= velocity;
		steer *= seperateIntensity;
	}
	applyForce(steer);
}

// Align
void Boid::align(vector <Boid>& boids, float maxSpeed, float alignIntensity) {
	int perceptionRadius = 25;
	glm::vec3 steer = glm::vec3(0, 0, 0);
	int total = 0;

	for (int i = 0; i < boids.size(); i++) {
		int d = glm::distance(mPosition, boids[i].mPosition);
		if ((this != &boids[i]) && (d < perceptionRadius)) {
			steer += boids[i].velocity;
			total++;
		}
	}
	if (total > 0) {
		steer /= total;
		if (glm::length(steer) < 1e-4f) {
			return;
		}
		steer = glm::normalize(steer) * maxSpeed;
		steer -= velocity;
		steer *= alignIntensity;
		applyForce(steer);
	}
}

// Cohesion
void Boid::cohesion(vector <Boid>& boids, float maxSpeed, float cohesionIntensity) {
	int perceptionRadius = 50;
	glm::vec3 steer = glm::vec3(0, 0, 0);
	int total = 0;

	for (int i = 0; i < boids.size(); i++) {
		int d = glm::distance(mPosition, boids[i].mPosition);
		if ((this != &boids[i]) && (d < perceptionRadius)) {
			steer += boids[i].mPosition;
			total++;
		}
	}
	if (total > 0) {
		steer /= total;
		if (glm::length(steer) < 1e-4f) {
			return;
		}
		steer = glm::normalize(steer) * maxSpeed;
		steer -= velocity;
		steer *= cohesionIntensity;
	}
	applyForce(steer);
}

constexpr float kBoundsX = 400.0f;
constexpr float kBoundsY = 400.0f;
constexpr float kBoundsZ = 400.0f;

// wrap around the screen
void Boid::wrapAround() {
	if (mPosition.x < 0) mPosition.x = kBoundsX;
	if (mPosition.x > kBoundsX) mPosition.x = 0;
	if (mPosition.y < 0) mPosition.y = kBoundsY;
	if (mPosition.y > kBoundsY) mPosition.y = 0;
	if (mPosition.z < 0) mPosition.z = kBoundsZ;
	if (mPosition.z > kBoundsZ) mPosition.z = 0;
}

// Line draw function
void Line::draw() {
	ofSetColor(ofColor::red);
	ofDrawLine(start, end);
}

// Circle draw function
void Circle::draw() {
	ofSetColor(ofColor::yellow);
	ofDrawCircle(position, radius);
}

// Load Model
void ofApp::loadModel(int modelIndex) {
	std::string modelPath = "geo/butterfly_flap" + ofToString(modelIndex) + "_texd.obj";
	cout << modelPath << endl;
	if (!boidModels[modelIndex].loadModel(modelPath)) {
		cout << "Can't load model" << endl;
		ofExit();
	}
	else {
		bBoidModelLoaded = true;
		boidModels[modelIndex].setScaleNormalization(false);
	}
}

//--------------------------------------------------------------
void ofApp::setup() {
	gui.setup();

	cam.setDistance(100);
	cam.setNearClip(.5);
	cam.setFarClip(1000);
	cam.setPosition(kBoundsX / 2.0f, kBoundsY / 2.0f, kBoundsZ / 2.0f);

	// Add sliders to the gui
	gui.add(turbulenceSlider.setup("Turbulence", glm::vec3(0, 0, 0), glm::vec3(-500, -500, -500), glm::vec3(500, 500, 500)));
	gui.add(numBoidsSlider.setup("Num Boids", 200, 1, 200));
	gui.add(turningRate.setup("Turning Rate", 0.04, 0.0, 0.2));
	gui.add(maxSpeedSlider.setup("Max Speed", 200, 0, 600));
	gui.add(minSpeedSlider.setup("Min Speed", 2, 0, 5));
	gui.add(seperateIntensitySlider.setup("Separation", 2.5, 0, 12));
	gui.add(alignIntensitySlider.setup("Align", 10.0, 0, 12));
	gui.add(conhesionIntensitySlider.setup("Cohesion", 10.0, 0, 12));
	gui.add(wanderIntensitySlider.setup("Wander", 10000, 0, 10000));
	gui.add(flapFrequencySlider.setup("Flap Frequency", 365, 0, 400));
	gui.add(resetButton.setup("Reset"));
	pauseButton.addListener(this, &ofApp::pauseButtonPressed);
	gui.add(pauseButton.setup("Pause/Unpause"));
	resetButton.addListener(this, &ofApp::resetButtonPressed);

	// Load models
	loadModel(0);
	loadModel(1);
	loadModel(2);
	loadModel(3);
	loadModel(4);
	loadModel(5);
	loadModel(6);
	lastModelChangeTime = ofGetElapsedTimeMillis();

	// Set up lighting
	ofSetSmoothLighting(true);

	// setup one point light
	light1.enable();
	light1.setDirectional();
	light1.setPosition(1, 1, 1);
	light1.setDiffuseColor(ofColor(255.f, 255.f, 255.f));
	light1.setSpecularColor(ofColor(255.f, 255.f, 255.f));
	light1.setAmbientColor(ofColor(200, 200, 200));

	// setup background image
	backgroundImage.load("images/blue.jpg");
}

//--------------------------------------------------------------
void ofApp::update() {


	if (!isPaused) {
		// If the number of boids has changed, resize the vector
		if (numBoidsSlider > boids.size()) {
			int diffToResize = numBoidsSlider - boids.size();
			for (int i = 0; i < diffToResize; i++) {
				boids.emplace_back(glm::vec3(ofRandom(kBoundsX), ofRandom(kBoundsY), ofRandom(kBoundsZ)));
			}
		}

		else if (numBoidsSlider < boids.size()) {
			int diffToResize = boids.size() - numBoidsSlider;
			for (int i = 0; i < diffToResize; i++) {
				boids.pop_back();
			}
		}

		// All real-time updates to the boids
		for (int i = 0; i < numBoidsSlider; i++) {

			// Debug mode
			if (mMode == Mode::Debug) {
				// Set the heading of the boid towards the mouse position
				/*boids[i].targetHeading = ofRadToDeg(-atan2(mousePos.y - boids[i].mPosition.y, -(mousePos.x - boids[i].mPosition.x))) + 90;*/

				// Draw the circle on the mouse position
				circle.position = glm::vec2(mousePos.x, mousePos.y);

				// Resets velocity from forces in other modes
				boids[i].velocity *= 0.0;
			}

			// Real mode
			else if (mMode == Mode::Real) {
				// Update position where boid looks

				boids[i].wander(wanderIntensitySlider);
				boids[i].align(boids, maxSpeedSlider, alignIntensitySlider);
				boids[i].cohesion(boids, maxSpeedSlider, conhesionIntensitySlider);
				boids[i].separation(boids, maxSpeedSlider, seperateIntensitySlider);

				boids[i].integrate(maxSpeedSlider, minSpeedSlider);
				boids[i].applyForce(turbulenceSlider);

				boids[i].wrapAround();  // uncomment to wrap around the screen but buggy

				if (glm::length(boids[i].velocity) > 1e-4f) {
					glm::vec3 targetHeading = glm::normalize(boids[i].velocity);
					boids[i].heading.x = ofLerpDegrees(boids[i].heading.x, targetHeading.x, turningRate);
					boids[i].heading.y = ofLerpDegrees(boids[i].heading.y, targetHeading.y, turningRate);
					boids[i].heading.z = ofLerpDegrees(boids[i].heading.z, targetHeading.z, turningRate);

				}

			}

			// Model change animation

			if (ofGetElapsedTimeMillis() - lastModelChangeTime >= 0.0 + 400.0 - flapFrequencySlider) {
				for (int i = 0; i < numBoidsSlider; i++) {
					auto& currentModelIndex = boids[i].currentModelIndex;
					auto& direction = boids[i].direction;

					// Check if currentIndex reached the end or beginning
					if (currentModelIndex == numModelsIndex && direction == 1) {
						direction = -1; // Change direction to backward
					}
					else if (currentModelIndex == 0 && direction == -1) {
						direction = 1; // Change direction to forward
					}

					// Update currentIndex based on direction
					currentModelIndex += direction;

				}

				// Record the time of this model change
				lastModelChangeTime = ofGetElapsedTimeMillis();

			}

		}
	}

}

//--------------------------------------------------------------
void ofApp::draw() {

	if (controlCam) {
		cam.enableMouseInput();
	}
	else {
		cam.disableMouseInput();
	}
	cam.begin();

	// Draw the boids
	for (int i = 0; i < numBoidsSlider; i++) {

		auto& boidModel = boidModels[boids[i].currentModelIndex];
		ofEnableLighting();
		ofPushMatrix();
		birdMaterial.setDiffuseColor(ofColor::red);
		birdMaterial.begin();
		ofSetColor(ofColor::blue);
		boidModel.enableMaterials();
		boidModel.enableColors();
		boidModel.enableNormals();

		// Calculate the rotation angle and axis
		glm::vec3 rotationAxis = - glm::cross(boids[i].heading, glm::vec3(0, 0, 1));
		float rotationAngle = glm::acos(glm::dot(boids[i].heading, glm::vec3(0, 0, 1)));
		float rotationAngleDeg = ofRadToDeg(rotationAngle);
		
		ofTranslate(boids[i].mPosition.x, boids[i].mPosition.y, boids[i].mPosition.z);
		ofRotateDeg(rotationAngleDeg, rotationAxis.x, rotationAxis.y, rotationAxis.z);
		ofRotateDeg(180, 0, 0, 1);

		ofScale(glm::vec3(2, 2, 2));

		boidModel.drawFaces();
		birdMaterial.end();
		ofPopMatrix();
		ofDisableLighting();
	}

	if (mMode == Mode::Debug) {
		// Draw the circle
		circle.draw();

		// Draw the line
		line.draw();
	}

	// Draw the background image
	if (backgroundImage.isAllocated()) {
		backgroundImage.bind(); // Bind the texture
		// Draw a rectangle with the image texture mapped onto it
		ofSetColor(255); // Set color to white to display the image without any tint
		ofDrawRectangle(-ofGetWidth() / 2, -ofGetHeight() / 2, ofGetWidth(), ofGetHeight()); // Draw a rectangle covering the entire screen
		backgroundImage.unbind(); // Unbind the texture
	}

	cam.end();

	ofDisableDepthTest();

	// Draw the GUI
	gui.draw();

	ofEnableDepthTest();

}

// reset the boids to their initial state
void ofApp::resetButtonPressed() {
	turbulenceSlider = glm::vec3(0.0, 0.0, 0.0);
	scaleSlider = 1.0;
	numBoidsSlider = numDefaultBoids;
	turningRate = 0.1;
	seperateIntensitySlider = 2.5;
	alignIntensitySlider = 10.0;
	conhesionIntensitySlider = 10.0;
	minSpeedSlider = 2;
	maxSpeedSlider = 200;
	boids.clear();
	cam.setPosition(kBoundsX / 2.0f, kBoundsY / 2.0f, kBoundsZ / 2.0f);
}

void ofApp::pauseButtonPressed() {
	isPaused = !isPaused; // Toggle pause state
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	if (key == OF_KEY_CONTROL) {
		controlCam = false; // Disable camera control when Ctrl key is pressed
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	if (key == OF_KEY_CONTROL) {
		controlCam = true; // Enable camera control when Ctrl key is released
	}
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	if (ofGetKeyPressed(OF_KEY_CONTROL) && button == OF_MOUSE_BUTTON_LEFT) {
			
		if (controlCam){
			controlCam = false;
		}
		else {
			controlCam = true;
		}
		glm::vec3 mouseWorldPos = cam.screenToWorld(ofVec3f(x, y, 0));
		glm::vec3 camPosition = cam.getPosition();
		glm::vec3 mouseDir = glm::normalize(mouseWorldPos - camPosition);
		
		//test ray plane intersection
		float dist;
		bool bIntersect = glm::intersectRayPlane(camPosition, mouseDir, glm::vec3(0,
			0, 0), glm::vec3(0, 0, 1), dist);

		cout<<"intersect: "<<bIntersect<< endl;

		glm::vec3 p;

		if (bIntersect) {
			cout << "distance: " << dist << endl;
			// compute intersection point
			// r(t) = o + dt
			//
			p = camPosition + mouseDir * dist;
			cout << "intersection point: " << p << endl;
		}

		boids.emplace_back(p);
		numBoidsSlider = numBoidsSlider + 1;
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
