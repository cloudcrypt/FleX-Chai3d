
class Streams : public Scene
{
public:

	Streams(const char* name, bool dam, bool ducky = false) : Scene(name), mDam(dam), mDucky(ducky) {}

	void Initialize()
	{
		float radius = 0.1f;

		// deforming bunny
		float s = radius*0.5f;
		float m = 1.f / 4.f;
		int group = 1;

		if (mDucky) {
			float bM = 1.f / 2.f;
			CreateParticleShape(GetFilePathByPlatform("../../data/ducky.obj").c_str(), Vec3(4.0f, 0.0f, 0.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), bM, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
			CreateParticleShape(GetFilePathByPlatform("../../data/ducky.obj").c_str(), Vec3(3.0f, 0.0f, 0.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), bM, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
			CreateParticleShape(GetFilePathByPlatform("../../data/ducky.obj").c_str(), Vec3(2.0f, 0.0f, 1.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), bM, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
		}
		else {
			CreateParticleShape(GetFilePathByPlatform("../../data/bunny.ply").c_str(), Vec3(4.0f, 0.0f, 0.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
			CreateParticleShape(GetFilePathByPlatform("../../data/bunny.ply").c_str(), Vec3(3.0f, 0.0f, 0.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
			CreateParticleShape(GetFilePathByPlatform("../../data/bunny.ply").c_str(), Vec3(2.0f, 0.0f, 1.0f), 0.5f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
		}
		CreateParticleShape(GetFilePathByPlatform("../../data/box.ply").c_str(), Vec3(4.0f, 0.0f, 1.0f), 0.45f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape(GetFilePathByPlatform("../../data/sphere.ply").c_str(), Vec3(3.0f, 0.0f, 1.0f), 0.45f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);
		CreateParticleShape(GetFilePathByPlatform("../../data/box.ply").c_str(), Vec3(2.0f, 0.0f, 0.0f), 0.45f, 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.0f);

		g_numSolidParticles = g_buffers->positions.size();

		float restDistance = radius*0.55f;

		if (mDam)
		{
			CreateParticleGrid(Vec3(0.0f, 0.0f, 0.6f), 42, 48, 24, restDistance, Vec3(0.0f), 1.0f, false, 0.0f, NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseFluid), 0.005f);
			g_lightDistance *= 0.5f;
		}

		g_sceneLower = Vec3(0.0f);

		g_numSubsteps = 2;

		g_params.radius = radius;
		g_params.dynamicFriction = 0.01f;
		g_params.viscosity = 2.0f;
		g_params.numIterations = 4;
		g_params.vorticityConfinement = 40.0f;
		g_params.fluidRestDistance = restDistance;
		g_params.solidPressure = 0.f;
		g_params.relaxationFactor = 0.0f;
		g_params.cohesion = 0.02f;
		g_params.collisionDistance = 0.01f;

		g_maxDiffuseParticles = 64 * 1024;
		g_diffuseScale = 0.5f;

		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.f);

		Emitter e1;
		e1.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e1.mPos = Vec3(radius-0.15f, 3.f, 0.85f);
		e1.mSpeed = (restDistance / g_dt)*2.0f; // 2 particle layers per-frame
		e1.mEnabled = true;

		g_emitters.push_back(e1);

		Emitter e2;
		e2.mDir = Vec3(-1.0f, 0.0f, 0.0f);
		e2.mRight = Vec3(0.0f, 0.0f, 1.0f);
		e2.mPos = Vec3(radius+4.425f, 3.f, 0.45f);
		e2.mSpeed = (restDistance / g_dt)*2.0f; // 2 particle layers per-frame
		e2.mEnabled = true;

		g_emitters.push_back(e2);

		g_numExtraParticles = 48 * 1024;

		g_lightDistance = 1.8f;

		g_params.numPlanes = 5;

		g_waveFloorTilt = 0.0f;
		g_waveFrequency = 1.5f;
		g_waveAmplitude = 2.0f;

		g_warmup = true;

		// draw options		
		g_drawPoints = false;
		g_drawEllipsoids = true;
		g_drawDiffuse = true;
	}

	bool mDam;
	bool mDucky;
};
