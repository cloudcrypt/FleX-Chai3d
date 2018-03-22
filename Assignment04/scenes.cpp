#include "scenes.h";

using namespace chai3d;

SceneManager::SceneManager(cWorld *world) : currentIndex(-1) {
	sceneWorld = new cMesh();
	world->addChild(sceneWorld);
}

SceneManager::~SceneManager() {
	delete sceneWorld;
}

void SceneManager::AddScene(Scene *scene) {
	scenes.push_back(scene);
	scene->m_manager = this;
}

void SceneManager::SetScene(int index) {
	if (currentIndex >= 0) {
		GetCurrentScene()->m_loaded = false;
	}

	while (index < 0) index += scenes.size();		// Wrap negative values
	index = index % scenes.size();					// Wrap positive values
	
	currentIndex = index;							// Load new scene
	LoadCurrentScene();
	GetCurrentScene()->m_loaded = true;
}

void SceneManager::NextScene() {
	SetScene(currentIndex + 1);
}

void SceneManager::PreviousScene() {
	SetScene(currentIndex - 1);
}

Scene* SceneManager::GetCurrentScene() {
	return scenes[currentIndex];
}

void SceneManager::LoadCurrentScene() {
	sceneWorld->clearAllChildren();
	Scene* scene = GetCurrentScene();

	for (Constraint* object : scene->m_constraints) {
		LoadConstraint(object);
	}

	for (RigidStatic* object : scene->m_statics) {
		LoadRigidbody(object);
	}

	for (RigidDynamic* object : scene->m_dynamics) {
		LoadRigidbody(object);
	}
}

void SceneManager::LoadConstraint(Constraint* a_constraint) {
	sceneWorld->addChild(a_constraint->GetRenderMesh());
}

void SceneManager::LoadRigidbody(Rigidbody* a_body) {
	Collider *collider = a_body->GetCollider();
	if (!collider) return;
	sceneWorld->addChild(collider->GetRenderMesh());
}

std::vector<Scene*> SceneManager::GetScenes() {
	return scenes;
}

Scene::Scene() : m_loaded(false), m_cursorOffset(cVector3d(0.1, 0.0, 0.0)), m_cameraOffset(cVector3d(0.25, 0.0, 0.0)), m_workspaceLocked(false), m_killFloor(-1.0) { }

Spring* Scene::CreateSpring(Rigidbody *a_body0, Rigidbody *a_body1, double a_naturalLength, double a_springStiffness, double a_dampingConstant) {
	Spring* spring = new Spring(a_body0, a_body1, a_naturalLength, a_springStiffness, a_dampingConstant);
	AddConstraint(spring);
	return spring;
}

void Scene::AddConstraint(Constraint* a_constraint) {
	m_constraints.insert(a_constraint);
	if (m_loaded) m_manager->LoadConstraint(a_constraint);
}

void Scene::DestroyConstraint(Constraint* a_constraint) {
	m_constraintsToDelete.insert(a_constraint);
}

void Scene::DestroyConstraint(Rigidbody* a_body) {
	for (Constraint* constraint : m_constraints) {
		if (constraint->ConstrainsRigidbody(a_body)) {
			DestroyConstraint(constraint);
		}
	}
}

RigidStatic* Scene::CreateStatic(chai3d::cVector3d a_position) {
	RigidStatic* body = new RigidStatic(a_position);
	AddStatic(body);
	return body;
}

void Scene::AddStatic(RigidStatic* a_static) {
	m_statics.insert(a_static);
	if (m_loaded) m_manager->LoadRigidbody(a_static);
}

void Scene::DestroyStatic(RigidStatic* a_static) {
	DestroyConstraint(a_static);
	m_staticsToDelete.insert(a_static);
}

RigidDynamic* Scene::CreateDynamic(double a_mass, chai3d::cVector3d a_position, chai3d::cVector3d a_velocity) {
	RigidDynamic* body = new RigidDynamic(a_mass, a_position, a_velocity);
	AddDynamic(body);
	return body;
}

void Scene::AddDynamic(RigidDynamic* a_dynamic) {
	m_dynamics.insert(a_dynamic);
	if (m_loaded) m_manager->LoadRigidbody(a_dynamic);
}

void Scene::DestroyDynamic(RigidDynamic* a_dynamic) {
	DestroyConstraint(a_dynamic);
	m_dynamicsToDelete.insert(a_dynamic);
}

void Scene::DestroyMarkedChai3dObjects() {
	for (cGenericObject* object : m_objectsToDelete) {
		object->getParent()->deleteChild(object);
	}
	m_objectsToDelete.clear();
}

void Scene::DestroyMarkedObjects() {
	for (Constraint* constraint : m_constraintsToDelete) {
		m_objectsToDelete.insert(constraint->GetRenderMesh());
		m_constraints.erase(constraint);
		delete constraint;
	}
	m_constraintsToDelete.clear();

	for (RigidStatic* body : m_staticsToDelete) {
		if (body->GetCollider()) {
			m_objectsToDelete.insert(body->GetCollider()->GetRenderMesh());
		}
		m_statics.erase(body);
		delete body;
	}
	m_staticsToDelete.clear();

	for (RigidDynamic* body : m_dynamicsToDelete) {
		if (body->GetCollider()) {
			m_objectsToDelete.insert(body->GetCollider()->GetRenderMesh());
		}
		m_dynamics.erase(body);
		delete body;
	}
	m_dynamicsToDelete.clear();
}

void Scene::SetCursor(RigidStatic* a_cursor) {
	AddStatic(a_cursor);
	m_cursor = a_cursor;
}
