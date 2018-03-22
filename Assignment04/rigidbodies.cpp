#include "rigidbodies.h"

#include <iostream>

using namespace chai3d;
using namespace std;

// ------------------------------------------------------------------------------------------------------
// Constants
// ------------------------------------------------------------------------------------------------------

const cVector3d Constants::GRAVITY = cVector3d(0.0, 0.0, -9.81);	// m/s^2
const double Constants::AIR_DAMPING = 1.0;							// Ns/m
const size_t Constants::MAX_COLLISIONS = -1;						// Infinite
const double Constants::TIME_STEP = 1.0 / 1000.0;					// s

// ------------------------------------------------------------------------------------------------------
// COLLIDERS
// ------------------------------------------------------------------------------------------------------

// BASE COLLIDER

Collider::Collider(double a_collisionStiffness, double a_dampingConstant, size_t a_collisionFlag, size_t a_collidesWithMask) :
	m_parent(nullptr), m_renderMesh(nullptr), m_collisionStiffness(a_collisionStiffness), m_dampingConstant(a_dampingConstant),
	m_collisionFlag(a_collisionFlag), m_collidesWithMask(a_collidesWithMask), m_currentCollisionCount(0) {}

void Collider::AddCollisionForces(Collider *a_collider, bool& a_result0, bool& a_result1) {
	// By default, no collisions
	a_result0 = false;
	a_result1 = false;

	// Check which colliders collide with each other
	bool body0CollidesWithBody1 = (m_collidesWithMask & a_collider->m_collisionFlag) != 0;
	bool body1CollidesWithBody0 = (a_collider->m_collidesWithMask & m_collisionFlag) != 0;
	if (!body0CollidesWithBody1 && !body1CollidesWithBody0) return;

	// Get the collision normal and return if there was no collision
	Collision collision = GetCollision(a_collider);
	if (!collision.valid) return;

	// Get stiffness and constants
	double collisionStiffness0 = m_collisionStiffness;
	double dampingConstant0 = m_dampingConstant;
	double collisionStiffness1 = a_collider->m_collisionStiffness;
	double dampingConstant1 = a_collider->m_dampingConstant;

	// Get the rigidbodies
	Rigidbody* body0 = m_parent;
	Rigidbody* body1 = a_collider->m_parent;

	// Get the relative velocity of the bodies
	cVector3d relativeVelocity = body1->GetLinearVelocity() - body0->GetLinearVelocity();

	// Calculate and add force on body0
	if (body0CollidesWithBody1 && m_currentCollisionCount < Constants::MAX_COLLISIONS) {
		// Increment collision count
		m_currentCollisionCount++;

		// Calculate and add force of collision (with damping)
		cVector3d collisionForce0 = -collisionStiffness0 * collision.normal;
		cVector3d collisionDampingForce0 = -dampingConstant0 * cProject(relativeVelocity, collision.normal);
		cVector3d netForce0 = collisionForce0 + collisionDampingForce0;
		body0->AddForce(-netForce0, collision.intersect0 - body0->GetPosition());

		// Return collision status
		a_result0 = true;
	}

	// Calculate and add force on body1
	if (body1CollidesWithBody0 && a_collider->m_currentCollisionCount < Constants::MAX_COLLISIONS) {
		// Increment collision count
		a_collider->m_currentCollisionCount++;

		// Calculate and add force of collision (with damping)
		cVector3d collisionForce1 = -collisionStiffness1 * collision.normal;
		cVector3d collisionDampingForce1 = -dampingConstant1 * cProject(relativeVelocity, collision.normal);
		cVector3d netForce1 = collisionForce1 + collisionDampingForce1;
		body1->AddForce(netForce1, collision.intersect1 - body1->GetPosition());

		// Return collision status
		a_result1 = true;
	}
}

void Collider::UpdateRenderMesh() const {
	if (!m_parent || !m_renderMesh) return;
	m_renderMesh->setLocalPos(m_parent->GetPosition());
}

cMesh* Collider::GetRenderMesh() const {
	return m_renderMesh;
}

void Collider::SetParent(Rigidbody* a_parent) {
	m_parent = a_parent;
}

Rigidbody* Collider::GetParent() const {
	return m_parent;
}

void Collider::SetCollisionFlag(size_t a_collisionFlag) {
	m_collisionFlag = a_collisionFlag;
}

size_t Collider::GetCollisionFlag() const {
	return m_collisionFlag;
}

void Collider::SetCollidesWithMask(size_t a_collidesWithMask) {
	m_collidesWithMask = a_collidesWithMask;
}

size_t Collider::GetCollidesWithMask() const {
	return m_collidesWithMask;
}

void Collider::SetCollisionStiffness(double a_collisionStiffness) {
	m_collisionStiffness = a_collisionStiffness;
}

double Collider::GetCollisionStiffness() const {
	return m_collisionStiffness;
}

void Collider::SetDampingConstant(double a_dampingConstant) {
	m_dampingConstant = a_dampingConstant;
}

double Collider::GetDampingConstant() const {
	return m_dampingConstant;
}

void Collider::ClearCollisionCount() {
	m_currentCollisionCount = 0;
}

size_t Collider::GetCollisionCount() const {
	return m_currentCollisionCount;
}

// SPHERE COLLIDER

SphereCollider::SphereCollider(double a_radius, double a_collisionStiffness, double a_dampingConstant,
	size_t a_collisionFlag, size_t a_collidesWithMask) :
	Collider(a_collisionStiffness, a_dampingConstant, a_collisionFlag, a_collidesWithMask), m_radius(a_radius) {
	
	m_renderMesh = new cMesh();
	CreateRenderMesh();
}

Collision SphereCollider::GetCollision(Collider* a_collider) {
	Collision collision;
	if (!m_parent || !a_collider->GetParent()) {
		collision.valid = false;
		return collision;
	}

	// Get the positions of the bodies and the collision normal
	cVector3d pos0 = m_parent->GetPosition();
	cVector3d pos1 = a_collider->GetParent()->GetPosition();
	cVector3d intersectionNormal = cNormalize(pos1 - pos0);

	// This ruins the purpose of inheritance
	// TODO: Find a better way to do this
	SphereCollider* sphereCollider = static_cast<SphereCollider*>(a_collider);

	// Get the radii of the colliders
	double radius0 = m_radius;
	double radius1 = sphereCollider->m_radius;

	// Get the intersection positions
	cVector3d intersect0 = pos0 + intersectionNormal * radius0;
	cVector3d intersect1 = pos1 - intersectionNormal * radius1;

	// Check if there was an intersection
	if ((intersect0 - pos1).length() <= radius1) {
		collision.valid = true;
		collision.intersect0 = intersect0;
		collision.intersect1 = intersect1;
		collision.normal = intersect1 - intersect0;
		return collision;
	}

	// Return the zero vector if there was no intersection
	collision.valid = false;
	return collision;
}

void SphereCollider::SetRadius(double a_radius) {
	m_radius = a_radius;
	m_renderMesh->getParent()->deleteChild(m_renderMesh);
	CreateRenderMesh();
	m_parent->UpdateInertiaMatrix();
}

double SphereCollider::GetRadius() const {
	return m_radius;
}

cMatrix3d SphereCollider::GetInertiaMatrix() const {
	// From: http://scienceworld.wolfram.com/physics/MomentofInertiaSphere.html
	cMatrix3d mat;
	mat.identity();
	mat *= (2.0 / 5.0) * m_radius * m_radius;
	return mat;
}

void SphereCollider::CreateRenderMesh() {
	cCreateSphere(m_renderMesh, m_radius);
}

// ------------------------------------------------------------------------------------------------------
// RIGIDBODIES
// ------------------------------------------------------------------------------------------------------

// BASE RIGIDBODY

Rigidbody::~Rigidbody() {
	delete m_collider;
}

Rigidbody::Rigidbody(cVector3d a_position) : m_position(a_position), m_currentNetForce(cVector3d(0.0, 0.0, 0.0)),
	m_currentNetMoment(cVector3d(0.0, 0.0, 0.0)), m_collider(nullptr), m_rotation(cQuaternion(1.0, 0.0, 0.0, 0.0)) { }

void Rigidbody::ClearForces() {
	m_currentNetForce = cVector3d(0.0, 0.0, 0.0);
	m_currentNetMoment = cVector3d(0.0, 0.0, 0.0);
}

void Rigidbody::AddForce(cVector3d a_force, cVector3d a_localPosition) {
	m_currentNetForce += a_force;

	m_currentNetMoment += cCross(a_localPosition, a_force);
}

cVector3d Rigidbody::GetCurrentNetForce() const {
	return m_currentNetForce;
}

void Rigidbody::SetPosition(cVector3d a_position) {
	m_position = a_position;
}

cVector3d Rigidbody::GetPosition() const {
	return m_position;
}

void Rigidbody::SetRotation(cQuaternion a_rotation) {
	m_rotation = a_rotation;
}

cQuaternion Rigidbody::GetRotation() const {
	return m_rotation;
}

chai3d::cMatrix3d Rigidbody::GetRotationMatrix() const {
	cMatrix3d mat;
	m_rotation.toRotMat(mat);
	return mat;
}

void Rigidbody::SetCollider(Collider* a_collider) {
	if (m_collider) delete m_collider;		// Good idea?
	m_collider = a_collider;
	m_collider->SetParent(this);
}

Collider* Rigidbody::GetCollider() const {
	return m_collider;
}

void Rigidbody::UpdateInertiaMatrix() {
	if (m_collider) {
		m_inertiaMatrix = m_collider->GetInertiaMatrix();
	} else {
		m_inertiaMatrix.identity();
	}
	m_inertiaMatrix *= GetMass();
}

// STATIC RIGIDBODY

RigidStatic::RigidStatic(cVector3d a_position) : Rigidbody(a_position) {}

void RigidStatic::ApplyForces(double a_deltaTime) {
	// We do not actually apply forces to ourselves because we are static

	// Clear the forces from this frame
	ClearForces();
	
	// Update colliders (clear collision count and update render mesh)
	if (m_collider) {
		m_collider->ClearCollisionCount();
		m_collider->GetRenderMesh()->setLocalPos(m_position);
	}
}

cVector3d RigidStatic::GetLinearVelocity() const {
	return cVector3d(0.0, 0.0, 0.0);	// Return the 0 vector because we are a static body
}

cVector3d RigidStatic::GetAngularVelocity() const {
	return cVector3d(0.0, 0.0, 0.0);	// Return the 0 vector because we are a static body
}

double RigidStatic::GetMass() const {
	return 0;	// Might as well be 0
}

// DYNAMIC RIGIDBODY

RigidDynamic::RigidDynamic(double a_mass, cVector3d a_position, cVector3d a_linearVelocity) :
	Rigidbody(a_position), m_mass(a_mass), m_linearVelocity(a_linearVelocity), m_angularVelocity(cVector3d(0.0, 0.0, 0.0)), m_angularMomentum(cVector3d(0.0, 0.0, 0.0)) {
	
	UpdateInertiaMatrix();
}

void RigidDynamic::ApplyForces(double a_deltaTime) {
	// POSITIONAL

	// Compute current linear acceleration from current forces (a = F/m)
	cVector3d acceleration = GetCurrentNetForce() / m_mass;

	// Integrate linear acceleration to linear velocity
	m_linearVelocity = m_linearVelocity + a_deltaTime * acceleration;

	// Integrate linear velocity to position using updated velocity (semi-implicit integration)
	m_position = m_position + a_deltaTime * m_linearVelocity;

	// ROTATIONAL

	// Integrate angular momentum from the current net moment
	m_angularMomentum = m_angularMomentum + a_deltaTime * m_currentNetMoment;
	
	// Compute angular velocity from angular momentum
	// NOTE: The 1/deltaTime seems necessary but I don't see it in my notes
	m_angularVelocity = cInverse(m_inertiaMatrix) * m_angularMomentum / a_deltaTime;

	// Take the derivative of the current rotation to get the next rotation
	cVector3d qV = cVector3d(m_rotation.x, m_rotation.y, m_rotation.z);
	double q0Derivative = -0.5 * cDot(m_angularVelocity, qV);
	cVector3d qVDerivative = 0.5 * (m_rotation.w * m_angularVelocity + cCross(m_angularVelocity, qV));
	cQuaternion derivative = cQuaternion(q0Derivative, qVDerivative.x(), qVDerivative.y(), qVDerivative.z());
	m_rotation = m_rotation + a_deltaTime * derivative;
	m_rotation.normalize();		// Fix wobblies at high speeds

	// GRAPHICS

	// Update colliders (clear collision count and update render mesh)
	if (m_collider) {
		m_collider->ClearCollisionCount();
		
		m_collider->GetRenderMesh()->setLocalPos(m_position);
		
		cMatrix3d rotMat;
		m_rotation.toRotMat(rotMat);
		m_collider->GetRenderMesh()->setLocalRot(rotMat);
	}

	// RESET

	// Clear the forces from this frame
	ClearForces();
}

void RigidDynamic::SetMass(double a_mass) {
	m_mass = a_mass;
}

double RigidDynamic::GetMass() const {
	return m_mass;
}

void RigidDynamic::SetLinearVelocity(cVector3d a_linearVelocity) {
	m_linearVelocity = a_linearVelocity;
}

cVector3d RigidDynamic::GetLinearVelocity() const {
	return m_linearVelocity;
}

void RigidDynamic::SetAngularVelocity(cVector3d a_angularVelocity) {
	m_angularVelocity = a_angularVelocity;
}

cVector3d RigidDynamic::GetAngularVelocity() const {
	return m_angularVelocity;
}

// ------------------------------------------------------------------------------------------------------
// CONSTRAINTS
// ------------------------------------------------------------------------------------------------------

// BASE CONSTRAINT

cGenericObject* Constraint::GetRenderMesh() const {
	return m_renderMesh;
}

// BASE SPRING

Spring::Spring(Rigidbody *a_body0, Rigidbody *a_body1, double a_naturalLength, double a_springStiffness, double a_dampingConstant) :
	Spring(a_body0, cVector3d(0.0, 0.0, 0.0), a_body1, cVector3d(0.0, 0.0, 0.0), a_naturalLength, a_springStiffness, a_dampingConstant) { }

Spring::Spring(Rigidbody *a_body0, cVector3d a_pos0, Rigidbody *a_body1, cVector3d a_pos1, double a_naturalLength, double a_springStiffness, double a_dampingConstant) :
	NConstraint({ a_body0, a_body1 }), m_naturalLength(a_naturalLength), m_springStiffness(a_springStiffness), m_dampingConstant(a_dampingConstant) {

	m_localPositions[0] = a_pos0;
	m_localPositions[1] = a_pos1;

	m_renderMesh = new cShapeLine();
}

void Spring::AddForces() const {
	// Get the normal of the spring
	cVector3d pos0 = GetPos0();
	cVector3d pos1 = GetPos1();
	cVector3d springNormal = pos1 - pos0;

	// Calculate the length difference of the spring
	double currentLength = springNormal.length();
	double lengthDiff = currentLength - m_naturalLength;

	// Normalize the spring normal
	if (springNormal.length() == 0.0) {
		springNormal = cVector3d(0.0, 0.0, -1.0);
	} else {
		springNormal.normalize();
	}

	// Get the relative velocity of the bodies
	cVector3d vel0 = m_bodies[0]->GetLinearVelocity();
	cVector3d vel1 = m_bodies[1]->GetLinearVelocity();
	cVector3d relativeVel = vel1 - vel0;

	// Project the relative velocity onto the spring normal
	cVector3d projectedVel;
	if (springNormal.equals(relativeVel)) {
		projectedVel = relativeVel;
	} else {
		projectedVel = cProject(relativeVel, springNormal);
	}

	// Compute forces
	cVector3d springForce = -m_springStiffness*lengthDiff * springNormal;		// F = -k(l-l_0)(p-q/||p-q||)
	cVector3d dampingForce = -m_dampingConstant * projectedVel;					// F = -bv
	cVector3d netForce = springForce + dampingForce;

	// Add forces to bodies
	m_bodies[0]->AddForce(-netForce, pos0 - m_bodies[0]->GetPosition());
	m_bodies[1]->AddForce(netForce, pos1 - m_bodies[1]->GetPosition());
}

void Spring::SetNaturalLength(double a_naturalLength) {
	m_naturalLength = a_naturalLength;
}

double Spring::GetNaturalLength() const {
	return m_naturalLength;
}

void Spring::SetSpringStiffness(double a_springStiffness) {
	m_springStiffness = a_springStiffness;
}

double Spring::GetSpringStiffness() const {
	return m_springStiffness;
}

void Spring::SetDampingConstant(double a_dampingConstant) {
	m_dampingConstant = a_dampingConstant;
}

double Spring::GetDampingConstant() const {
	return m_dampingConstant;
}

void Spring::UpdateRenderMesh() const {
	// Get positions and length diff
	cVector3d pos0 = GetPos0();
	cVector3d pos1 = GetPos1();
	double currentLength = (pos1 - pos0).length();
	double lengthDiff = currentLength - m_naturalLength;

	cShapeLine* line = static_cast<cShapeLine*>(m_renderMesh);

	// Update the positions of the line
	line->m_pointA = pos0;
	line->m_pointB = pos1;

	// Update the colour of the line
	constexpr double lengthEps = 0.01;
	if (lengthDiff < -lengthEps) {
		line->m_colorPointA.setYellow();
		line->m_colorPointB.setYellow();
	} else if (lengthDiff > lengthEps) {
		line->m_colorPointA.setRed();
		line->m_colorPointB.setRed();
	} else {
		line->m_colorPointA.setGreen();
		line->m_colorPointB.setGreen();
	}
}

cVector3d Spring::GetPos0() const {
	Rigidbody* body = m_bodies[0];
	cMatrix3d rotMat = body->GetRotationMatrix();
	return body->GetPosition() + rotMat * m_localPositions[0];
}

cVector3d Spring::GetPos1() const  {
	Rigidbody* body = m_bodies[1];
	cMatrix3d rotMat = body->GetRotationMatrix();
	return body->GetPosition() + rotMat * m_localPositions[1];
}

// ROPE SPRING

Rope::Rope(Rigidbody *a_body0, Rigidbody *a_body1, double a_naturalLength, double a_springStiffness, double a_dampingConstant) :
	Spring(a_body0, a_body1, a_naturalLength, a_springStiffness, a_dampingConstant) { }

Rope::Rope(Rigidbody *a_body0, cVector3d a_pos0, Rigidbody *a_body1, cVector3d a_pos1, double a_naturalLength, double a_springStiffness, double a_dampingConstant) :
	Spring(a_body0, a_pos0, a_body1, a_pos1, a_naturalLength, a_springStiffness, a_dampingConstant) { }

void Rope::AddForces() const {
	// Get the normal of the spring
	cVector3d pos0 = GetPos0();
	cVector3d pos1 = GetPos1();
	cVector3d springNormal = pos1 - pos0;

	// Calculate the length difference of the spring
	double currentLength = springNormal.length();
	double lengthDiff = currentLength - m_naturalLength;

	// Only add spring forces when the rope is over-extended
	if (lengthDiff > 0) {
		Spring::AddForces();
	}
}

// TORSION SPRING

TorsionSpring::TorsionSpring(Rigidbody *a_body0, Rigidbody *a_body1, Rigidbody *a_body2, double a_naturalAngleRadians, double a_springStiffness, double a_dampingConstant) :
	NConstraint({ a_body0, a_body1, a_body2 }), m_naturalAngleRadians(a_naturalAngleRadians), m_springStiffness(a_springStiffness), m_dampingConstant(a_dampingConstant) {

	m_renderMesh = new cMesh();

	m_line1to0 = new cShapeLine();
	m_renderMesh->addChild(m_line1to0);

	m_line1to2 = new cShapeLine();
	m_renderMesh->addChild(m_line1to2);
}

void TorsionSpring::AddForces() const {
	// Get positions and angle diff
	cVector3d pos0 = m_bodies[0]->GetPosition();
	cVector3d pos1 = m_bodies[1]->GetPosition();
	cVector3d pos2 = m_bodies[2]->GetPosition();
	cVector3d pos1to0 = pos0 - pos1;
	cVector3d pos1to2 = pos2 - pos1;
	cVector3d torqueAxis = cCross(pos1to0, pos1to2);
	double currentAngleRadians = cAngle(pos0 - pos1, pos2 - pos1);
	double angleDiffRadians = currentAngleRadians - m_naturalAngleRadians;

	cVector3d force0Dir = cNormalize(cCross(pos1to0, torqueAxis));
	double force0Mag = -angleDiffRadians * m_springStiffness;
	m_bodies[0]->AddForce(force0Dir * force0Mag);

	cVector3d force2Dir = cNormalize(cCross(pos1to2, torqueAxis));
	double force2Mag = angleDiffRadians * m_springStiffness;
	m_bodies[2]->AddForce(force2Dir * force2Mag);
}

void TorsionSpring::UpdateRenderMesh() const {
	// Get positions and angle diff
	cVector3d pos0 = m_bodies[0]->GetPosition();
	cVector3d pos1 = m_bodies[1]->GetPosition();
	cVector3d pos2 = m_bodies[2]->GetPosition();
	double currentAngleRadians = cAngle(pos0 - pos1, pos2 - pos1);
	double angleDiffRadians = currentAngleRadians - m_naturalAngleRadians;

	// Update the positions of the lines
	m_line1to0->m_pointA = pos1;
	m_line1to0->m_pointB = pos0;
	m_line1to2->m_pointA = pos1;
	m_line1to2->m_pointB = pos2;

	// Update the colour of the line
	constexpr double angleEps = 0.01;
	if (angleDiffRadians < -angleEps) {
		m_line1to0->m_colorPointA.setYellow();
		m_line1to0->m_colorPointB.setYellow();
		m_line1to2->m_colorPointA.setYellow();
		m_line1to2->m_colorPointB.setYellow();
	} else if (angleDiffRadians > angleEps) {
		m_line1to0->m_colorPointA.setRed();
		m_line1to0->m_colorPointB.setRed();
		m_line1to2->m_colorPointA.setRed();
		m_line1to2->m_colorPointB.setRed();
	} else {
		m_line1to0->m_colorPointA.setGreen();
		m_line1to0->m_colorPointB.setGreen();
		m_line1to2->m_colorPointA.setGreen();
		m_line1to2->m_colorPointB.setGreen();
	}
}