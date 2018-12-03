#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	extPoint = Vec3(0, 0, 0);
	extForce = Vec3(0, 0, 0);
}

const char * RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1,Demo2,Demo3,Demo4";
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
	rigidBodies.clear();

	switch (m_iTestCase) {
	case 0: {
		RigidBody rb = RigidBody(2, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 90), Vec3(1, 0.6, 0.5));
		rigidBodies.push_back(rb);
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		EulerStep(0, 2);
		RigidBody body = rigidBodies.at(0);
		cout << "Linear velocity: " << body.velocity << endl;
		cout << "Angular velocity: " << body.w << endl;
		cout << "World space velocity of a point: " << cross(body.w, Vec3(0.3, 0.5, 0.25) - getPositionOfRigidBody(0)) << endl;
		break;
	}
	case 1: {
		RigidBody rb = RigidBody(2, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 90), Vec3(1, 0.6, 0.5));
		rigidBodies.push_back(rb);
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	}
	case 2: {
		RigidBody rb1 = RigidBody(2, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 90), Vec3(1, 0.6, 0.5));
		RigidBody rb2 = RigidBody(2, Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(8, -10, 0), Vec3(1, 1.2, 1));

		rigidBodies.push_back(rb1);
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		rigidBodies.push_back(rb2);
		break;
	}
	case 3: {
		RigidBody rb1 = RigidBody(2, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 90), Vec3(1, 0.6, 0.5));
		RigidBody rb2 = RigidBody(2, Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(8, -10, 0), Vec3(1, 1.2, 1));
		RigidBody rb3 = RigidBody(1, Vec3(0, 0, 0), Vec3(-1.8, 0, 0), Vec3(10, 0, -10), Vec3(1, 1, 1));
		RigidBody rb4 = RigidBody(1.2, Vec3(0, 0.1, 0), Vec3(0, 0, 1.8), Vec3(45, 0, 35), Vec3(1, 1.5, 0.8));
		rigidBodies.push_back(rb1);
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		rigidBodies.push_back(rb2);
		rigidBodies.push_back(rb3);
		rigidBodies.push_back(rb4);
	}
	}
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
	m_iTestCase = testCase;
	reset();
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
			for (int i = 0; i < rigidBodies.size(); i++) {
				Vec3 newPos = extPoint - getPositionOfRigidBody(i);
				newPos /= sqrt(newPos.squaredDistanceTo(Vec3(0, 0, 0))) / 2;
				newPos += getPositionOfRigidBody(i);
				if (m_iTestCase != 0) {
					applyForceOnBody(i, newPos, extForce);
					applyForceOnBody(i, getPositionOfRigidBody(i), extForce);
				}
			}
		}
		extPoint = Vec3(0, 0, 0);
		extForce = Vec3(0, 0, 0);
	}
}

void RigidBodySystemSimulator::resolveCollision(int i, int j) {
	RigidBody* A = &rigidBodies.at(i);
	RigidBody* B = &rigidBodies.at(j);
	Mat4 matA = A->scaleMat * A->rotMat * A->translatMat;
	Mat4 matB = B->scaleMat * B->rotMat * B->translatMat;
	CollisionInfo collision = checkCollisionSAT(matA, matB);
	if (collision.isValid) {
		Vec3 xA = collision.collisionPointWorld - getPositionOfRigidBody(i);
		Vec3 xB = collision.collisionPointWorld - getPositionOfRigidBody(j);
		Vec3 vrel = A->velocity - B->velocity + cross(A->w, xA) - cross(B->w, xB);
		//cout << vrel << cross(xA, A->w) << cross(xB, B->w) << collision.normalWorld << dot(vrel, collision.normalWorld) << endl;
		if (dot(vrel, collision.normalWorld) <= 0) {
			Mat4 rotA = A->rotMat;
			Mat4 rotB = B->rotMat;
			Mat4 IA = rotA.inverse() * A->invTensor;
			Mat4 IB = rotB.inverse() * B->invTensor;
			IA = IA * rotA;
			IB = IB * rotB;
			Vec3 n = collision.normalWorld;
			float J = -2 * dot(vrel, collision.normalWorld) /
				(1 / A->mass + 1 / B->mass + dot(cross(IA.transformVector(cross(xA, n)), xA) + cross(IB.transformVector(cross(xB, n)), xB), n));
			A->velocity += J * n / A->mass;
			B->velocity -= J * n / B->mass;
			A->L += J * cross(xA, n);
			B->L -= J * cross(xB, n);
		}
	}
}

void RigidBodySystemSimulator::resolveCollision(int i, Mat4 &wall) {
	RigidBody* A = &rigidBodies.at(i);
	Mat4 matA = A->scaleMat * A->rotMat * A->translatMat;
	CollisionInfo collision = checkCollisionSAT(matA, wall);
	if (collision.isValid) {
		Vec3 xA = collision.collisionPointWorld - getPositionOfRigidBody(i);
		Vec3 vrel = A->velocity + cross(A->w, xA);
		if (dot(vrel, collision.normalWorld) < 0) {
			Mat4 rotA = A->rotMat;
			Mat4 IA = rotA.inverse() * A->invTensor;
			IA = IA * rotA;
			Vec3 n = collision.normalWorld;
			float J = -2 * dot(vrel, collision.normalWorld) /
				(1 / A->mass + dot(cross(IA.transformVector(cross(xA, n)), xA), n));
			A->velocity += J * n / A->mass;
			A->L += J * cross(xA, n);
		}
	}
}

void RigidBodySystemSimulator::EulerStep(int i, float timeStep) {
	RigidBody* body = &rigidBodies.at(i);
	Mat4 vMat;
	vMat.initTranslation(timeStep * body->velocity.X, timeStep * body->velocity.Y, timeStep * body->velocity.Z);
	body->translatMat = vMat * body->translatMat;
	body->velocity += body->F / body->mass * timeStep;
	if (m_iTestCase == 3) {
		body->velocity -= Vec3(0, 0.1, 0) * timeStep;
	}
	Mat4 rotMat = body->rotMat;
	Quat r = Quat(rotMat);
	Quat h = Quat(body->w.X, body->w.Y, body->w.Z, 0);
	Quat add = h * r * (timeStep / 2.);
	r -= add;
	r /= r.normSq() == 0 ? 1 : r.norm();
	body->F = Vec3(0, 0, 0);
	body->L += timeStep * body->q;
	body->q = Vec3(0, 0, 0);
	rotMat.transpose();
	Mat4 invI = rotMat * body->invTensor;
	rotMat.transpose();
	invI = invI * rotMat;
	body->w = invI.transformVector(body->L);
	body->rotMat = r.getRotMat().inverse();
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase != 0)
		for (int i = 0; i < rigidBodies.size(); i++) {
			if (m_iTestCase == 3) {
				Mat4 trans;
				trans.initTranslation(0, -2, 0);
				Mat4 scale;
				scale.initScaling(100, 2, 100);
				resolveCollision(i, scale * trans);
			}
			for (int j = i + 1; j < rigidBodies.size(); j++) 
				resolveCollision(i, j);		
			EulerStep(i, timeStep);
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
	body->F += force;
	Mat4 rot = body->rotMat;
	body->q += (cross(loc - getPositionOfRigidBody(i), force));
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
