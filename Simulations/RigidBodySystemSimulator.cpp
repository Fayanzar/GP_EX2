#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	RigidBody rb = RigidBody(2, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 90), Vec3(1, 0.6, 0.5));
	rigidBodies.push_back(rb);
	
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
}

const char * RigidBodySystemSimulator::getTestCasesStr() {
	return "1";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
	for each (RigidBody rigidBody in rigidBodies)
	{
		DUC->drawRigidBody(rigidBody.scaleMat * rigidBody.rotMat * rigidBody.translatMat);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < rigidBodies.size(); i++) {
		RigidBody* body = &rigidBodies.at(i);
		Mat4 vMat;
		vMat.initTranslation(timeStep * body->velocity.X, timeStep * body->velocity.Y, timeStep * body->velocity.Z);
		body->translatMat = vMat * body->translatMat;
		body->velocity += body->F / body->mass * timeStep;
		Mat4 rotMat = body->rotMat;
		rotMat.transpose();
		Quat r = Quat(rotMat);
		Quat h = Quat(body->w.X, body->w.Y, body->w.Z, 0);
		Quat add = h * r * (timeStep / 2.);
		r += add;
		r /= r.normSq() == 0 ? 1 : r.norm();
		body->F = Vec3(0, 0, 0);
		body->L += timeStep * body->q;
		body->q = Vec3(0, 0, 0);
		Mat4 invI = r.getRotMat() * body->invTensor;
		rotMat = r.getRotMat();
		rotMat.transpose();
		invI *= rotMat;
		body->w = invI * body->L;
		body->rotMat = r.getRotMat();
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RigidBody* body = &rigidBodies.at(i);
	body->F = force;
	body->q = cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
}
