//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "rigidbodies.h"
#include "scenes.h"

#include <iterator>
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;

SceneManager *sceneManager;

// Declare the collision groups
constexpr size_t CURSOR_COLLISION_FLAG = 1;
constexpr size_t CURSOR_COLLIDES_WITH = -1;

constexpr size_t MESH_COLLISION_FLAG = CURSOR_COLLISION_FLAG << 1;
constexpr size_t BALL_COLLISION_FLAG = MESH_COLLISION_FLAG << 1;

constexpr size_t MESH_COLLIDES_WITH = CURSOR_COLLISION_FLAG | BALL_COLLISION_FLAG;
constexpr size_t BALL_COLLIDES_WITH = CURSOR_COLLISION_FLAG | MESH_COLLISION_FLAG | BALL_COLLISION_FLAG;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = false;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    TEMPLATE:    application.cpp

    Description of your application.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
	world->m_backgroundColor.setb(0x21, 0x21, 0x21);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (0.25, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0); 

	// Create the scene manager
	sceneManager = new SceneManager(world);

	// ------------------------------------------------------------------------------------------------------
	// SCENE 1
	// ------------------------------------------------------------------------------------------------------

	{
		// Create the scene, and register it with the scene manager
		Scene* scene = new Scene();
		sceneManager->AddScene(scene);

		// Set the camera offset for the scene
		scene->m_cameraOffset = cVector3d(0.25, 0.0, 0.0);

		// Create the cursor as a static rigidbody
		RigidStatic* cursor = new RigidStatic(cVector3d(0.0, 0.0, -0.02));
		SphereCollider* cursorCollider = new SphereCollider(0.0125, 2000.0, 0.0, CURSOR_COLLISION_FLAG, CURSOR_COLLIDES_WITH);
		cursorCollider->GetRenderMesh()->m_material->setWhite();
		cursor->SetCollider(cursorCollider);
		scene->SetCursor(cursor);

		// Declare some constants to describe the scene
		constexpr size_t ballCount = 4;
		constexpr double spacing = 0.025;
		constexpr double height = 0.04;

		// Declare some constants to describe the objects in the scene
		constexpr double mass[ballCount] = { 0.2, 2.0, 0.5, 0.5 };
		constexpr double springStiffness[ballCount] = { 400.0, 4000.0, 200.0, 200.0 };
		constexpr double springDamping[ballCount] = { 10.0, 100.0, 1.0, 25.0 };

		for (int i = 0; i < ballCount; ++i) {
			cVector3d position(0.0, spacing * (static_cast<double>(i) - static_cast<double>(ballCount)/2), height);

			RigidStatic *fixed = scene->CreateStatic(position);

			RigidDynamic *ball = scene->CreateDynamic(mass[i], position);
			SphereCollider *collider = new SphereCollider(0.01, 10000.0, 10.0, BALL_COLLISION_FLAG, BALL_COLLIDES_WITH);
			collider->GetRenderMesh()->m_material->setGreenChartreuse();
			ball->SetCollider(collider);

			Spring *spring = scene->CreateSpring(fixed, ball, 0.05, springStiffness[i], springDamping[i]);
		}
	}

	// ------------------------------------------------------------------------------------------------------
	// SCENE 2
	// ------------------------------------------------------------------------------------------------------

	{
		// Create the scene, and register it with the scene manager
		Scene* scene = new Scene();
		sceneManager->AddScene(scene);

		// Set the camera offset for the scene
		scene->m_cameraOffset = cVector3d(0.25, 0.0, 0.0);

		// Create the cursor as a static rigidbody
		RigidStatic* cursor = new RigidStatic(cVector3d(0.0, 0.0, -0.02));
		SphereCollider* cursorCollider = new SphereCollider(0.0125, 2000.0, 0.0, CURSOR_COLLISION_FLAG, CURSOR_COLLIDES_WITH);
		cursorCollider->GetRenderMesh()->m_material->setWhite();
		cursor->SetCollider(cursorCollider);
		scene->SetCursor(cursor);

		// Declare some constants to describe the scene
		constexpr size_t ballCount = 4;
		constexpr double spacing = 0.025;
		constexpr double height = 0.04;

		// Declare some constants to describe the objects in the scene
		constexpr double radius = 0.01;
		constexpr double mass[ballCount] = { 0.2, 2.0, 0.5, 0.5 };
		constexpr double springStiffness[ballCount] = { 400.0, 4000.0, 200.0, 200.0 };
		constexpr double springDamping[ballCount] = { 10.0, 100.0, 1.0, 25.0 };

		// Declare the ball rotation
		cQuaternion rotate180;
		rotate180.fromAxisAngle(cVector3d(0.0, 0.0, 1.0), cDegToRad(180.0));

		for (int i = 0; i < ballCount; ++i) {
			cVector3d position(0.0, spacing * (static_cast<double>(i) - static_cast<double>(ballCount) / 2), height);

			RigidStatic *fixed = scene->CreateStatic(position);

			RigidDynamic *ball = scene->CreateDynamic(mass[i], position - cVector3d(0.0, 0.0, radius));
			ball->SetRotation(rotate180);
			SphereCollider *collider = new SphereCollider(radius, 10000.0, 10.0, BALL_COLLISION_FLAG, BALL_COLLIDES_WITH);
			ball->SetCollider(collider);

			collider->GetRenderMesh()->m_texture = cTexture2d::create();
			int imageIndex = i + 9;
			collider->GetRenderMesh()->m_texture->loadFromFile("images/Ball" + to_string(imageIndex) + ".jpg");
			collider->GetRenderMesh()->setUseTexture(true);

			Rope *rope = new Rope(fixed, cVector3d(0.0, 0.0, 0.0), ball, cVector3d(0.0, 0.0, radius), 0.05, springStiffness[i], springDamping[i]);
			scene->AddConstraint(rope);
		}
	}

	// ------------------------------------------------------------------------------------------------------
	// SCENE 3
	// ------------------------------------------------------------------------------------------------------

	{
		// Create the scene, and register it with the scene manager
		Scene* scene = new Scene();
		sceneManager->AddScene(scene);

		// Set the camera offset for the scene
		scene->m_cameraOffset = cVector3d(0.25, 0.0, 0.25);

		// Create the cursor as a static rigidbody
		RigidStatic* cursor = new RigidStatic(cVector3d(0.0, 0.0, -0.02));
		SphereCollider* cursorCollider = new SphereCollider(0.0125, 2000.0, 0.0, CURSOR_COLLISION_FLAG, CURSOR_COLLIDES_WITH);
		cursorCollider->GetRenderMesh()->m_material->setWhite();
		cursor->SetCollider(cursorCollider);
		scene->SetCursor(cursor);

		// Declare some constants to describe the scene
		constexpr size_t rowCount = 9;
		Rigidbody* bodies[rowCount][rowCount];

		// Declare some constants to describe the bodies in the scene
		constexpr double mass = 0.05;
		constexpr double spacing = 0.03;
		constexpr double radius = 0.03;

		// Create the rigidbodies in a grid
		for (int x = 0; x < rowCount; ++x) {
			for (int y = 0; y < rowCount; ++y) {
				// Compute the position of the body
				cVector3d position = cVector3d(
					static_cast<double>(x) - static_cast<double>(rowCount) * 0.5,
					static_cast<double>(y) - static_cast<double>(rowCount) * 0.5,
					0.0) * spacing;

				// Create a collider for the body
				SphereCollider* collider = new SphereCollider(radius, 10000.0, 10.0, MESH_COLLISION_FLAG, MESH_COLLIDES_WITH);

				Rigidbody *body;
				if ((x == 0 && y == 0) || (x == 0 && y == rowCount - 1)
					|| (x == rowCount - 1 && y == 0) || (x == rowCount - 1 && y == rowCount - 1)) {

					// Make an anchor at the corners
					body = scene->CreateStatic(position);

					// Make the mesh red
					collider->GetRenderMesh()->m_material->setRedCrimson();
				} else {
					// Make a dynamic otherwise
					body = scene->CreateDynamic(mass, position);

					// Make the mesh green
					collider->GetRenderMesh()->m_material->setGreenChartreuse();
				}

				// Add the collider to the body and save the body in a data structure
				body->SetCollider(collider);
				bodies[x][y] = body;
			}
		}

		// Declare some constants to describe the springs in the scene
		constexpr double springStiffness = 3000.0;
		constexpr double springDamping = 10.0;

		// Create the springs between the bodies
		for (int x = 0; x < rowCount; ++x) {
			for (y = 0; y < rowCount; ++y) {
				Rigidbody* body = bodies[x][y];
				
				// Create a spring between this body and the body to its right
				if (x < rowCount - 1) {
					Rigidbody* right = bodies[x + 1][y];
					scene->CreateSpring(body, right, spacing, springStiffness, springDamping);
				}

				// Create a spring between this body and the body in front of it
				if (y < rowCount - 1) {
					Rigidbody* front = bodies[x][y + 1];
					scene->CreateSpring(body, front, spacing, springStiffness, springDamping);
				}
			}
		}
	}

	// ------------------------------------------------------------------------------------------------------
	// SCENE 4
	// ------------------------------------------------------------------------------------------------------

	{
		// Create the scene, and register it with the scene manager
		Scene* scene = new Scene();
		sceneManager->AddScene(scene);

		// Set the camera offset for the scene
		scene->m_cameraOffset = cVector3d(0.25, 0.0, 0.25);

		// Create the cursor as a static rigidbody
		RigidStatic* cursor = new RigidStatic(cVector3d(0.0, 0.0, -0.02));
		SphereCollider* cursorCollider = new SphereCollider(0.0125, 2000.0, 0.0, CURSOR_COLLISION_FLAG, CURSOR_COLLIDES_WITH);
		cursorCollider->GetRenderMesh()->m_material->setWhite();
		cursor->SetCollider(cursorCollider);
		scene->SetCursor(cursor);

		// Declare some constants to describe the scene
		constexpr size_t rowCount = 9;
		Rigidbody* bodies[rowCount][rowCount];

		// Declare some constants to describe the bodies in the scene
		constexpr double mass = 0.05;
		constexpr double spacing = 0.03;
		constexpr double radius = 0.03;

		// Create the rigidbodies in a grid
		for (int x = 0; x < rowCount; ++x) {
			for (int y = 0; y < rowCount; ++y) {
				// Compute the position of the body
				cVector3d position = cVector3d(
					static_cast<double>(x) - static_cast<double>(rowCount) * 0.5,
					static_cast<double>(y) - static_cast<double>(rowCount) * 0.5,
					0.0) * spacing;

				// Create a collider for the body
				SphereCollider* collider = new SphereCollider(radius, 10000.0, 10.0, MESH_COLLISION_FLAG, MESH_COLLIDES_WITH);

				Rigidbody *body;
				if ((x == 0 && y == 0) || (x == 0 && y == rowCount - 1)
					|| (x == rowCount - 1 && y == 0) || (x == rowCount - 1 && y == rowCount - 1)) {

					// Make an anchor at the corners
					body = scene->CreateStatic(position);

					// Make the mesh red
					collider->GetRenderMesh()->m_material->setRedCrimson();
				}
				else {
					// Make a dynamic otherwise
					body = scene->CreateDynamic(mass, position);

					// Make the mesh green
					collider->GetRenderMesh()->m_material->setGreenChartreuse();
				}

				// Add the collider to the body and save the body in a data structure
				body->SetCollider(collider);
				bodies[x][y] = body;
			}
		}

		// Declare some constants to describe the springs in the scene
		constexpr double springStiffness = 1000.0;
		constexpr double springDamping = 10.0;

		// Create the springs between the bodies
		for (int x = 0; x < rowCount; ++x) {
			for (y = 0; y < rowCount; ++y) {
				Rigidbody* body = bodies[x][y];

				// Create a spring between this body and the body to its right
				if (x < rowCount - 1) {
					Rigidbody* right = bodies[x + 1][y];
					scene->CreateSpring(body, right, spacing, springStiffness, springDamping);
				}

				// Create a spring between this body and the body in front of it
				if (y < rowCount - 1) {
					Rigidbody* front = bodies[x][y + 1];
					scene->CreateSpring(body, front, spacing, springStiffness, springDamping);
				}

				// Create a spring between this body and the body front and right to it
				if (x < rowCount - 1 && y < rowCount - 1) {
					Rigidbody* frontRight = bodies[x + 1][y + 1];
					const double root2 = cSqrt(2.0);
					scene->CreateSpring(body, frontRight, root2 * spacing, springStiffness, springDamping);
				}

				// Create a spring between this body and the body front and left to it
				if (x > 0 && y < rowCount - 1) {
					Rigidbody* frontLeft = bodies[x - 1][y + 1];
					const double root2 = cSqrt(2.0);
					scene->CreateSpring(body, frontLeft, root2 * spacing, springStiffness, springDamping);
				}
			}
		}
	}

	// ------------------------------------------------------------------------------------------------------
	// SCENE 5
	// ------------------------------------------------------------------------------------------------------

	/*{
		// Create the scene, and register it with the scene manager
		Scene* scene = new Scene();
		sceneManager->AddScene(scene);

		// Set the camera offset for the scene
		scene->m_cameraOffset = cVector3d(0.25, 0.0, 0.25);

		// Create the cursor as a static rigidbody
		RigidStatic* cursor = new RigidStatic(cVector3d(0.0, 0.0, -0.02));
		SphereCollider* cursorCollider = new SphereCollider(0.0125, 2000.0, 0.0, CURSOR_COLLISION_FLAG, CURSOR_COLLIDES_WITH);
		cursorCollider->GetRenderMesh()->m_material->setWhite();
		cursor->SetCollider(cursorCollider);
		scene->SetCursor(cursor);

		// Declare some constants to describe the scene
		constexpr size_t rowCount = 3;
		Rigidbody* bodies[rowCount][rowCount];

		// Declare some constants to describe the bodies in the scene
		constexpr double mass = 0.05;
		constexpr double spacing = 0.03;
		constexpr double radius = 0.01;

		// Create the rigidbodies in a grid
		for (int x = 0; x < rowCount; ++x) {
			for (int y = 0; y < rowCount; ++y) {
				// Compute the position of the body
				cVector3d position = cVector3d(
					static_cast<double>(x) - static_cast<double>(rowCount) * 0.5,
					static_cast<double>(y) - static_cast<double>(rowCount) * 0.5,
					0.0) * spacing;

				// Create a collider for the body
				SphereCollider* collider = new SphereCollider(radius, 10000.0, 10.0, MESH_COLLISION_FLAG, MESH_COLLIDES_WITH);

				Rigidbody *body;
				if ((x == 0 && y == 0) || (x == 0 && y == rowCount - 1)
					|| (x == rowCount - 1 && y == 0) || (x == rowCount - 1 && y == rowCount - 1)) {

					// Make an anchor at the corners
					body = scene->CreateStatic(position);

					// Make the mesh red
					collider->GetRenderMesh()->m_material->setRedCrimson();
				}
				else {
					// Make a dynamic otherwise
					body = scene->CreateDynamic(mass, position);

					// Make the mesh green
					collider->GetRenderMesh()->m_material->setGreenChartreuse();
				}

				// Add the collider to the body and save the body in a data structure
				body->SetCollider(collider);
				bodies[x][y] = body;
			}
		}

		// Declare some constants to describe the springs in the scene
		constexpr double springStiffness = 1000.0;
		constexpr double springDamping = 10.0;

		// Create the springs between the bodies
		for (int x = 0; x < rowCount; ++x) {
			for (y = 0; y < rowCount; ++y) {
				Rigidbody* body = bodies[x][y];

				// Create a spring between this body and the body front and right to it
				if (x < rowCount - 1 && y < rowCount - 1) {
					Rigidbody* front = bodies[x][y + 1];
					Rigidbody* right = bodies[x + 1][y];
					TorsionSpring* spring = new TorsionSpring(front, body, right, M_PI*0.5, springStiffness, springDamping);
					scene->AddConstraint(spring);
				}

				// Create a spring between this body and the body back and right to it
				if (x < rowCount - 1 && y > 0) {
					Rigidbody* back = bodies[x][y - 1];
					Rigidbody* right = bodies[x + 1][y];
					TorsionSpring* spring = new TorsionSpring(back, body, right, M_PI*0.5, springStiffness, springDamping);
					scene->AddConstraint(spring);
				}
			}
		}
	}*/

	// Start with the first scene
	sceneManager->SetScene(0);

    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
		for (Scene* scene : sceneManager->GetScenes()) {
			cMesh* cursorMesh = scene->m_cursor->GetCollider()->GetRenderMesh();

			// display reference frame
			cursorMesh->setShowFrame(true);

			// set the size of the reference frame
			cursorMesh->setFrameSize(0.05);
		}
    }

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rates of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if (a_action != GLFW_PRESS)
    {
        return;
    }

	// Scene management
	else if (a_key == GLFW_KEY_LEFT) {
		sceneManager->PreviousScene();
	} else if (a_key == GLFW_KEY_RIGHT) {
		sceneManager->NextScene();
	}

	// Workspace management
	else if (a_key == GLFW_KEY_SPACE) {
		sceneManager->GetCurrentScene()->m_workspaceLocked = !sceneManager->GetCurrentScene()->m_workspaceLocked;
	}

	// Interaction
	else if (a_key == GLFW_KEY_1) {
		// Create a heavy ball

		// Create its collider
		SphereCollider* collider = new SphereCollider(0.03, 10000.0, 10.0, BALL_COLLISION_FLAG, BALL_COLLIDES_WITH);
		collider->GetRenderMesh()->m_material->setBlueMediumSlate();

		// Create its rigidbody
		RigidDynamic* heavyBall = new RigidDynamic(0.75, cVector3d(0.0, 0.0, 1.0));
		heavyBall->SetCollider(collider);
		collider->UpdateRenderMesh();
		sceneManager->GetCurrentScene()->AddDynamic(heavyBall);
	} else if (a_key == GLFW_KEY_2) {
		// Create a light ball
		Scene* scene = sceneManager->GetCurrentScene();

		// Create its collider
		constexpr double radius = 0.01;
		SphereCollider* collider = new SphereCollider(radius, 10000.0, 10.0, BALL_COLLISION_FLAG, BALL_COLLIDES_WITH);
		collider->GetRenderMesh()->m_material->setBlueLightSteel();
		collider->GetRenderMesh()->m_texture = cTexture2d::create();
		int imageIndex = 1 + (rand() % static_cast<int>(15 - 1 + 1));
		collider->GetRenderMesh()->m_texture->loadFromFile("images/Ball" + to_string(imageIndex) + ".jpg");
		collider->GetRenderMesh()->setUseTexture(true);

		// Declare the ball rotation
		cQuaternion rotate180;
		rotate180.fromAxisAngle(cVector3d(0.0, 0.0, 1.0), cDegToRad(180.0));

		// Create its rigidbody
		RigidDynamic* lightBall = new RigidDynamic(0.05, scene->m_cursor->GetPosition());
		lightBall->SetRotation(rotate180);
		lightBall->SetCollider(collider);
		collider->UpdateRenderMesh();
		scene->AddDynamic(lightBall);

		// Create a spring between the cursor and the light ball
		Rope* rope = new Rope(scene->m_cursor, cVector3d(0.0, 0.0, 0.0), lightBall, cVector3d(0.0, 0.0, radius), 0.065, 50.0, 1.0);
		rope->UpdateRenderMesh();
		scene->AddConstraint(rope);
	} else if (a_key == GLFW_KEY_3) {
		// Toggle the visibility of the mesh
		Scene* scene = sceneManager->GetCurrentScene();
		for (RigidDynamic* body : scene->m_dynamics) {
			if (!body->GetCollider()) continue;
			if (body->GetCollider()->GetCollisionFlag() != MESH_COLLISION_FLAG) continue;
			body->GetCollider()->GetRenderMesh()->setEnabled(!body->GetCollider()->GetRenderMesh()->getEnabled());
		}

		for (RigidStatic* body : scene->m_statics) {
			if (!body->GetCollider()) continue;
			if (body->GetCollider()->GetCollisionFlag() != MESH_COLLISION_FLAG) continue;
			body->GetCollider()->GetRenderMesh()->setEnabled(!body->GetCollider()->GetRenderMesh()->getEnabled());
		}
	}

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
	// Clean up objects that should be deleted
	sceneManager->GetCurrentScene()->DestroyMarkedChai3dObjects();

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

cVector3d clampVectors(cVector3d value, cVector3d min, cVector3d max) {
	return cVector3d(
		cClamp(value.x(), min.x(), max.x()),
		cClamp(value.y(), min.y(), max.y()),
		cClamp(value.z(), min.z(), max.z())
	);
}

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        cVector3d position;
        hapticDevice->getPosition(position);

		cVector3d velocity;
		hapticDevice->getLinearVelocity(velocity);

        // read orientation 
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read user-switch status (button 0)
        bool button = false;
        hapticDevice->getUserSwitch(0, button);

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

		Scene *scene = sceneManager->GetCurrentScene();

		if (!scene->m_workspaceLocked) {
			// Define the movement sphere parameters
			constexpr double radius = 0.03;
			constexpr double maxDist = 0.045 - radius;
			constexpr double speed = 0.0001;
			constexpr double distScale = 4;

			// Define the workspace boundaries
			const cVector3d workspaceMin = cVector3d(-1.f, -1.f, -1.f);
			const cVector3d workspaceMax = cVector3d(1.f, 1.f, 1.f);

			// Modify the movement vector. The x-axis needs to be moved forward
			// slightly so that the forward movement can be equal to the backward
			// movement. Then the x-axis needs to be scaled so that it can be
			// equal to the y and z axes
			cVector3d scaledPosition = position;
			scaledPosition.x((-0.0035 + scaledPosition.x()) * 1.1);

			// Joystick simulation force
			constexpr double stiffness = 5.0;
			cVector3d moveForce = cVector3d(0.0, 0.0, 0.0);

			// Move the device position when it leaves the movement sphere
			if (scaledPosition.length() > radius) {
				// Calculate the direction and distance to move the device
				cVector3d norm = cNormalize(scaledPosition);
				double dist = cMin(scaledPosition.length() - radius, maxDist) / maxDist;
				double scaledDist = pow(dist, distScale) * speed;
				cVector3d newPos = scene->m_cursorOffset + (norm * scaledDist);
				scene->m_cursorOffset = clampVectors(newPos, workspaceMin, workspaceMax);

				// Update joystick simulation force
				moveForce = stiffness * dist * -norm;
			}

			// Update the camera
			camera->set(scene->m_cursorOffset + scene->m_cameraOffset, scene->m_cursorOffset, cVector3d(0.0, 0.0, 1.0));
		}

        // update position and orienation of cursor
		scene->m_cursor->SetPosition(scene->m_cursorOffset + position);
        //cursor->setLocalRot(rotation);

        /////////////////////////////////////////////////////////////////////
        // COMPUTE FORCES
        /////////////////////////////////////////////////////////////////////

        cVector3d force(0, 0, 0);
        cVector3d torque(0, 0, 0);
        double gripperForce = 0.0;

		// Update constraint forces
		for (Constraint* constraint : scene->m_constraints) {
			constraint->UpdateRenderMesh();
			constraint->AddForces();
		}

		// Update statics forces
		for (auto it0 = scene->m_statics.begin(); it0 != scene->m_statics.end(); ++it0) {
			RigidStatic* body0 = *it0;

			Collider* collider0 = (*it0)->GetCollider();
			if (!collider0) continue;

			// Collide with statics
			for (auto it1 = next(it0, 1); it1 != scene->m_statics.end(); ++it1) {
				RigidStatic* body1 = *it1;
				Collider* collider1 = body1->GetCollider();
				if (!collider1) continue;

				bool collision0, collision1;
				collider0->AddCollisionForces(collider1, collision0, collision1);

				// Destroy objects the cursor collides with when the button is pressed
				if (button) {
					if (collision1 && body0 == scene->m_cursor) {
						scene->DestroyStatic(body1);
					} else if (collision0 && body1 == scene->m_cursor) {
						scene->DestroyStatic(body0);
					}
				}
			}

			// Collide with dynamics
			for (RigidDynamic* body1 : scene->m_dynamics) {
				Collider* collider1 = body1->GetCollider();
				if (!collider1) continue;
				
				bool collision0, collision1;
				collider0->AddCollisionForces(collider1, collision0, collision1);

				// Destroy objects the cursor collides with when the button is pressed
				if (button && collision1 && body0 == scene->m_cursor) {
					scene->DestroyDynamic(body1);
				}
			}

			// Update haptic device forces
			if (body0 == scene->m_cursor) {
				force = body0->GetCurrentNetForce();
			}

			// Apply/clear forces
			body0->ApplyForces(Constants::TIME_STEP);
		}

		// Update dynamics forces
		for (auto it0 = scene->m_dynamics.begin(); it0 != scene->m_dynamics.end(); ++it0) {
			RigidDynamic* body0 = *it0;

			// Add force of gravity
			cVector3d gravityForce = body0->GetMass() * Constants::GRAVITY;		// F = ma
			body0->AddForce(gravityForce);

			// Add air damping force
			cVector3d airDampingForce = -Constants::AIR_DAMPING * body0->GetLinearVelocity();		// F = -bv
			body0->AddForce(airDampingForce);

			// Collide with dynamics
			Collider* collider0 = body0->GetCollider();
			if (collider0) {
				for (auto it1 = next(it0, 1); it1 != scene->m_dynamics.end(); ++it1) {
					Collider* collider1 = (*it1)->GetCollider();
					if (!collider1) continue;

					bool collision0, collision1;
					collider0->AddCollisionForces(collider1, collision0, collision1);
				}
			}

			// Apply/clear forces
			body0->ApplyForces(Constants::TIME_STEP);

			// Delete dynamics below scene kill floor
			if (body0->GetPosition().z() < scene->m_killFloor) {
				scene->DestroyDynamic(body0);
			}
		}

		// Clean up objects that should be deleted
		sceneManager->GetCurrentScene()->DestroyMarkedObjects();

        /////////////////////////////////////////////////////////////////////
        // APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // send computed force, torque, and gripper force to haptic device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
