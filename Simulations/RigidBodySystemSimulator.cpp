#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	RigidBody rb = RigidBody(2, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 90), Vec3(1, 0.6, 0.5));
	rigidBodies.push_back(rb);
	
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	extPoint = Vec3(0, 0, 0);
	extForce = Vec3(0, 0, 0);
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
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);

		Vec3 point = Vec3((float)m_trackmouse.x, -(float)m_trackmouse.y, 0);
		Vec3 pointWorld = worldViewInv.transformVectorNormal(point);

		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.05f;
		inputWorld = inputWorld * inputScale;
		extForce = inputWorld;
		extPoint = pointWorld;
	}
	else {
		if (extForce.X != 0 || extForce.Y != 0 || extForce.Z != 0) {
			applyForceOnBody(0, getPositionOfRigidBody(0), extForce);
			cout << extForce << endl;
		}
		extPoint = Vec3(0, 0, 0);
		extForce = Vec3(0, 0, 0);
	}
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
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	RigidBody body = rigidBodies.at(i);
	return Vec3(body.translatMat.value[0][3], body.translatMat.value[1][3], body.translatMat.value[0][3]);
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidBodies.at(i).velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidBodies.at(i).w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RigidBody* body = &rigidBodies.at(i);
	body->F = force;
	body->q = cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody body = RigidBody(mass, Vec3(0, 0, 0), position, Vec3(0, 0, 0), size);
	rigidBodies.push_back(body);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBodies.at(i).rotMat = orientation.getRotMat();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodies.at(i).velocity = velocity;
}
