#pragma once

#include "chai3d.h"

#include <array>

// ------------------------------------------------------------------------------------------------------
// Constants
// ------------------------------------------------------------------------------------------------------

class Constants {
public:
	static const chai3d::cVector3d GRAVITY;
	static const double AIR_DAMPING;
	static const size_t MAX_COLLISIONS;
	static const double TIME_STEP;
};

// ------------------------------------------------------------------------------------------------------
// COLLIDERS
// ------------------------------------------------------------------------------------------------------

// COLLISION DATA

struct Collision {
	chai3d::cVector3d intersect0;
	chai3d::cVector3d intersect1;
	chai3d::cVector3d normal;
	bool valid;
};

// BASE COLLIDER

class Rigidbody;

class Collider {
friend class Rigidbody;
public:
	Collider(double a_collisionStiffness = 1000.0, double a_dampingConstant = 10.0, size_t a_collisionFlag = 1, size_t a_collidesWithMask = 1);
	virtual ~Collider() = default;

	virtual Collision GetCollision(Collider* a_collider) = 0;
	void AddCollisionForces(Collider *a_collider, bool& a_result0, bool& a_result1);

	void UpdateRenderMesh() const;
	chai3d::cMesh* GetRenderMesh() const;

	Rigidbody* GetParent() const;

	void SetCollisionFlag(size_t a_collisionFlag);
	size_t GetCollisionFlag() const;

	void SetCollidesWithMask(size_t a_collidesWithMask);
	size_t GetCollidesWithMask() const;

	void SetCollisionStiffness(double a_collisionStiffness);
	double GetCollisionStiffness() const;

	void SetDampingConstant(double a_dampingConstant);
	double GetDampingConstant() const;

	void ClearCollisionCount();
	size_t GetCollisionCount() const;

	virtual chai3d::cMatrix3d GetInertiaMatrix() const = 0;
private:
	// Only the Rigidbody should use this
	void SetParent(Rigidbody* a_parent);
protected:
	Rigidbody* m_parent;
	chai3d::cMesh* m_renderMesh;

	double m_collisionStiffness;
	double m_dampingConstant;

	size_t m_collisionFlag;
	size_t m_collidesWithMask;

	size_t m_currentCollisionCount;
};

// SPHERE COLLIDER

class SphereCollider : public Collider {
public:
	SphereCollider(double a_radius = 0.01, double a_collisionStiffness = 1000.0, double a_dampingConstant = 10.0, size_t a_collisionFlag = 1, size_t a_collidesWithMask = 1);

	Collision GetCollision(Collider* a_collider);

	void SetRadius(double a_radius);
	double GetRadius() const;

	chai3d::cMatrix3d GetInertiaMatrix() const override;
private:
	void CreateRenderMesh();

	double m_radius;
};

// ------------------------------------------------------------------------------------------------------
// RIGIDBODIES
// ------------------------------------------------------------------------------------------------------

// BASE RIGIDBODY

class Rigidbody {
public:
	virtual ~Rigidbody();
	Rigidbody(chai3d::cVector3d a_position = chai3d::cVector3d(0.0, 0.0, 0.0));

	void ClearForces();
	void AddForce(chai3d::cVector3d a_force, chai3d::cVector3d a_localPosition = chai3d::cVector3d(0.0, 0.0, 0.0));
	chai3d::cVector3d GetCurrentNetForce() const;
	virtual void ApplyForces(double a_deltaTime) = 0;

	virtual chai3d::cVector3d GetLinearVelocity() const = 0;
	virtual chai3d::cVector3d GetAngularVelocity() const = 0;
	virtual double GetMass() const = 0;

	void SetPosition(chai3d::cVector3d a_position);
	chai3d::cVector3d GetPosition() const;

	void SetRotation(chai3d::cQuaternion a_rotation);
	chai3d::cQuaternion GetRotation() const;
	chai3d::cMatrix3d GetRotationMatrix() const;

	// TODO: Should be add/get index
	void SetCollider(Collider* a_collider);
	Collider* GetCollider() const;

	void UpdateInertiaMatrix();
protected:
	chai3d::cVector3d m_position;
	chai3d::cQuaternion m_rotation;
	chai3d::cMatrix3d m_inertiaMatrix;

	chai3d::cVector3d m_currentNetForce;
	chai3d::cVector3d m_currentNetMoment;

	// TODO: Should really be a vector of colliders, but I don't think it is ever necessary for this assignment
	Collider* m_collider;
};

// STATIC RIGIDBODY

class RigidStatic : public Rigidbody {
public:
	RigidStatic(chai3d::cVector3d a_position = chai3d::cVector3d(0.0, 0.0, 0.0));

	void ApplyForces(double a_deltaTime) override;

	double GetMass() const override;

	chai3d::cVector3d GetLinearVelocity() const override;
	chai3d::cVector3d GetAngularVelocity() const override;
};

// DYNAMIC RIGIDBODY

class RigidDynamic : public Rigidbody {
public:
	RigidDynamic(double a_mass = 1.0,
		chai3d::cVector3d a_position = chai3d::cVector3d(0.0, 0.0, 0.0),
		chai3d::cVector3d a_linearVelocity = chai3d::cVector3d(0.0, 0.0, 0.0));

	void ApplyForces(double a_deltaTime) override;

	void SetMass(double a_mass);
	double GetMass() const override;

	void SetLinearVelocity(chai3d::cVector3d a_linearVelocity);
	chai3d::cVector3d GetLinearVelocity() const override;

	void SetAngularVelocity(chai3d::cVector3d a_angularVelocity);
	chai3d::cVector3d GetAngularVelocity() const override;
private:
	double m_mass;
	chai3d::cVector3d m_linearVelocity;
	chai3d::cVector3d m_angularVelocity;
	chai3d::cVector3d m_angularMomentum;
};

// ------------------------------------------------------------------------------------------------------
// CONSTRAINTS
// ------------------------------------------------------------------------------------------------------

// BASE CONSTRAINT

class Constraint {
public:
	virtual ~Constraint() = default;
	Constraint() : m_renderMesh(nullptr) {}

	virtual void AddForces() const = 0;

	virtual size_t GetRigidbodyCount() const = 0;
	virtual Rigidbody* GetRigidbody(size_t index) const = 0;
	virtual bool ConstrainsRigidbody(Rigidbody* a_body) const = 0;

	chai3d::cGenericObject* GetRenderMesh() const;
	virtual void UpdateRenderMesh() const = 0;
protected:
	chai3d::cGenericObject* m_renderMesh;
};

// BASE N-CONSTRAINT

template <size_t N>
class NConstraint : public Constraint {
public:
	virtual ~NConstraint() = default;
	NConstraint(std::array<Rigidbody*, N> a_bodies) : Constraint(), m_bodies(a_bodies) {}

	size_t GetRigidbodyCount() const override {
		return N;
	}

	Rigidbody* GetRigidbody(size_t index) const override {
		return m_bodies[index];
	}
	
	bool ConstrainsRigidbody(Rigidbody* a_body) const override {
		for (Rigidbody* body : m_bodies) {
			if (body == a_body) return true;
		}
		return false;
	}

	virtual void AddForces() const = 0;
	virtual void UpdateRenderMesh() const = 0;
protected:
	std::array<Rigidbody*, N> m_bodies;
};

// BASE SPRING

class Spring : public NConstraint<2> {
public:
	Spring(Rigidbody *a_body0, Rigidbody *a_body1, double a_naturalLength = 0.05, double a_springStiffness = 200.0, double a_dampingConstant = 5.0);
	Spring(Rigidbody *a_body0, chai3d::cVector3d a_pos0, Rigidbody *a_body1, chai3d::cVector3d a_pos1, double a_naturalLength = 0.05, double a_springStiffness = 200.0, double a_dampingConstant = 5.0);

	void AddForces() const override;

	void UpdateRenderMesh() const override;

	void SetNaturalLength(double a_naturalLength);
	double GetNaturalLength() const;

	void SetSpringStiffness(double a_springStiffness);
	double GetSpringStiffness() const;

	void SetDampingConstant(double a_dampingConstant);
	double GetDampingConstant() const;
protected:
	chai3d::cVector3d GetPos0() const;
	chai3d::cVector3d GetPos1() const;

	double m_naturalLength;
	double m_springStiffness;
	double m_dampingConstant;

	chai3d::cVector3d m_localPositions[2];
};

// ROPE SPRING

class Rope : public Spring {
public:
	Rope(Rigidbody *a_body0, Rigidbody *a_body1, double a_naturalLength = 0.05, double a_springStiffness = 200.0, double a_dampingConstant = 5.0);
	Rope(Rigidbody *a_body0, chai3d::cVector3d a_pos0, Rigidbody *a_body1, chai3d::cVector3d a_pos1, double a_naturalLength = 0.05, double a_springStiffness = 200.0, double a_dampingConstant = 5.0);

	void AddForces() const override;
};

// TODO: TORSION SPRING

class TorsionSpring : public NConstraint<3> {
public:
	TorsionSpring(Rigidbody *a_body0, Rigidbody *a_body1, Rigidbody *a_body2, double a_naturalAngleRadians, double a_springStiffness = 200.0, double a_dampingConstant = 5.0);

	void AddForces() const override;

	void UpdateRenderMesh() const override;

protected:
	double m_naturalAngleRadians;
	double m_springStiffness;
	double m_dampingConstant;

	chai3d::cShapeLine* m_line1to0;
	chai3d::cShapeLine* m_line1to2;
};