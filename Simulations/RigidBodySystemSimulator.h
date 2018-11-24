#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody {
	RigidBody(float m, Vec3 vel, Vec3 tr, Vec3 rot, Vec3 sc) {
		mass = m;
		velocity = vel;
		translatMat.initTranslation(tr.X, tr.Y, tr.Z);
		rotMat.initRotationXYZ(rot.X, rot.Y, rot.Z);
		scaleMat.initScaling(sc.X, sc.Y, sc.Z);
		invTensor = Mat4(12. / m / (sc.Y * sc.Y + sc.Z * sc.Z), 0, 0, 0,
			0, 12. / m / (sc.X * sc.X + sc.Z * sc.Z), 0, 0,
			0, 0, 12. / m / (sc.X * sc.X + sc.Y * sc.Y), 0,
			0, 0, 0, 1);
		L = Vec3(0, 0, 0);
		F = Vec3(0, 0, 0);
		q = Vec3(0, 0, 0);
		w = Vec3(0, 0, 0);
	}
	float mass;
	Vec3 velocity;
	Vec3 w;
	Vec3 L;
	Vec3 F;
	Vec3 q;
	Mat4 translatMat;
	Mat4 rotMat;
	Mat4 scaleMat;
	Mat4 invTensor;
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	vector<RigidBody> rigidBodies;
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif