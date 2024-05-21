#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofEasyCam.h"
#include "ofxAssimpModelLoader.h"
#include "ofImage.h"

#include <glm/gtx/intersect.hpp>

class Boid {
public:
	Boid(const glm::vec3& position) {
		mPosition = position;

		/*vertices.push_back(glm::vec3(5, -5, 0));
		vertices.push_back(glm::vec3(-5, -5, 0));
		vertices.push_back(glm::vec3(0, 8, 0));*/
		
		scale = glm::vec3(2.0, 2.0, 2.0);

		currentModelIndex = static_cast<int>(ofRandom(0, 6));

	}

	void draw();
	void integrate(float maxSpeed, float minSpeed);
	void applyForce(glm::vec3 force);
	void wander(float wanderIntensity);
	void separation(vector <Boid>& boids, float maxSpeed, float seperateIntensity);
	void align(vector <Boid>& boids, float maxSpeed, float alignIntensity);
	void cohesion(vector <Boid>& boids, float maxSpeed, float conhesionIntensity);
	void wrapAround();

	vector<glm::vec2> vertices;
	glm::vec3 mPosition;
	glm::vec3 velocity;
	glm::vec3 acceleration;
	glm::vec3 wanderForce;
	glm::vec3 scale;

	float rotation;
	glm::vec3 heading = glm::vec3(0, 0, 1);

	int currentModelIndex = 0;
	int direction = 1;

};

class Line {
public:
	Line() {
		start = glm::vec2(ofRandom(ofGetWidth()), ofRandom(ofGetHeight()));
		end = glm::vec2(ofRandom(ofGetWidth()), ofRandom(ofGetHeight()));
		length = 150;
	}
	void draw();

	glm::vec2 start;
	glm::vec2 end;
	float length;
};

class Circle {
public:
	Circle() {
		position = glm::vec2(ofRandom(ofGetWidth()), ofRandom(ofGetHeight()));
		radius = 8;
	}
	void draw();
	glm::vec2 position;
	float radius;
};

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();
	void loadModel(int modelIndex);

	ofxPanel gui;
	ofxButton resetButton;
	ofxButton pauseButton;
	ofxFloatSlider scaleSlider;
	ofxIntSlider numBoidsSlider;
	ofxVec3Slider turbulenceSlider;
	ofxFloatSlider turningRate;
	ofxFloatSlider maxSpeedSlider;
	ofxFloatSlider minSpeedSlider;
	ofxFloatSlider wanderIntensitySlider;
	ofxFloatSlider seperateIntensitySlider;
	ofxFloatSlider alignIntensitySlider;
	ofxFloatSlider conhesionIntensitySlider;
	ofxFloatSlider flapFrequencySlider;

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	void resetButtonPressed();
	void pauseButtonPressed();

	ofEasyCam cam;
	bool controlCam = true;
	ofLight light1;

	//Model
	glm::vec3 modelPosition;
	unsigned long long lastModelChangeTime = 0.0;
	std::array<ofxAssimpModelLoader, 7> boidModels;
	int numModelsIndex = 6;

	bool bBoidModelLoaded = false;
	bool bWireFrame = false;
	bool isPaused = false;

	ofMaterial birdMaterial;

	ofImage backgroundImage;

	vector<Boid> boids;
	Line line;
	Circle circle;

	enum class Mode {
		Debug,
		Real,
		BoidAdd
	};
	glm::vec3 mousePos;
	Mode mMode{ Mode::Real };

	static constexpr int numDefaultBoids = 100;

};
