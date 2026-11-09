// ----------------------------------------------------------------------------
// fix_physics_rate.cpp
//
// Part of vstack-enhanced, a fork of V-Patch (c) 2021-2024 Vaana
// ----------------------------------------------------------------------------

#include "fix_physics.h"

#include "patching.h"
#include <cmath>
#include <cstring>
#include <vector>

//
// FIXED-RATE VEHICLE PHYSICS
//
// The constant fixes in fix_physics.cpp make every per-frame update scale
// with the frame time, but the handling still differs slightly from 30 FPS:
// the car's forces are explicit feedback (e.g. the yaw damping torque is
// proportional to the yaw rate and grows with speed), and at 30 FPS each
// force was held for a whole 1/30 s step. Some of these are stiff enough
// that the 30 FPS step size is part of the feel.
//
// This mode reproduces that exactly while keeping the motion smooth:
//
// - Every 1/30 s the vehicle update runs as at 30 FPS (dt = 1/30, original
//   constants). The forces and torques it hands to Havok are recorded in the
//   body's local frame and applied over the real frame time.
// - On the frames in between, the recorded forces are applied again, moved
//   with the body. Havok still integrates every frame, so the car moves
//   smoothly, but the forces change at 30 Hz, exactly as they did.
// - The visual update (wheel and body render matrices, vtable slot 9) runs
//   every frame with the real frame time.
//
// Motorbikes go through the same update and are handled the same way.
//
// Independently of the mode, the paused update (frame time 0, which the game
// replaces with a fixed 1/30 s update every frame) is limited to 30 updates
// per second, as at 30 FPS.
//

int physicsRate = 30;

static double frameTime = 1.0 / 30.0;
static unsigned int frameCounter = 0;

struct Vec4
{
	float x, y, z, w;
};

typedef void (__fastcall *UpdateFn)(void* dyn, void* edx, float dt, int flag, int reduced);
typedef void (__fastcall *VisualFn)(void* dyn, void* edx, float dt);
typedef void (__fastcall *PointForceFn)(void* motion, void* edx, float dt, const Vec4* force, const Vec4* point);
typedef void (__fastcall *ForceFn)(void* motion, void* edx, float dt, const Vec4* force);

//
// Vehicle classes
//

enum { KIND_CAR, KIND_BIKE, NUM_KINDS };

struct VehicleClass
{
	const char*	rttiName;
	void**		vtable;
	UpdateFn	update;			// Original vtable slot 4
	bool		visualStubbed;	// Slot 9 call removed from the update
};

static VehicleClass classes[NUM_KINDS] =
{
	{ ".?AVCarDynamicsInstance@@" },
	{ ".?AVMotorbikeDynamicsInstance@@" },
};

static BYTE* physicsEnabledFlag = nullptr; // Car update and slot 9 only run if *(flag + 4)

//
// Havok motion classes whose force functions are recorded
//

struct MotionClass
{
	const char*		rttiName;
	void**			vtable;
	PointForceFn	applyPointForce;	// +0x54 applyForce(dt, force, point)
	ForceFn			applyForce;			// +0x58 applyForce(dt, force)
	ForceFn			applyTorque;		// +0x5C applyTorque(dt, torque)
};

static MotionClass motions[] =
{
	{ ".?AVhkpBoxMotion@@" },
	{ ".?AVhkpThinBoxMotion@@" },
	{ ".?AVhkpSphereMotion@@" },
};

static const size_t numMotions = sizeof(motions) / sizeof(motions[0]);

static MotionClass* MotionClassOf(void* motion)
{
	void** vtable = *(void***)motion;
	for (size_t i = 0; i < numMotions; i++)
	{
		if (motions[i].vtable == vtable)
			return &motions[i];
	}
	return nullptr;
}

//
// Per-vehicle state
//

enum { FORCE_POINT, FORCE_LINEAR, FORCE_TORQUE };

struct RecordedForce
{
	int		type;
	Vec4	force;	// Body space
	Vec4	point;	// Body space (FORCE_POINT only)
};

struct VehicleState
{
	void*						dyn;
	unsigned int				lastFrame;
	double						accumulator;
	double						pausedAccumulator;
	std::vector<RecordedForce>	forces;
};

static const size_t maxVehicles = 128;
static VehicleState states[maxVehicles];

static VehicleState* GetState(void* dyn)
{
	VehicleState* freeSlot = nullptr;

	for (size_t i = 0; i < maxVehicles; i++)
	{
		VehicleState& s = states[i];
		if (s.dyn == dyn)
			return &s;

		// Reuse slots of vehicles that have not been updated for a while
		if (!freeSlot && (!s.dyn || frameCounter - s.lastFrame > 300))
			freeSlot = &s;
	}

	if (!freeSlot)
		return nullptr;

	freeSlot->dyn				= dyn;
	freeSlot->lastFrame			= frameCounter - 2;
	freeSlot->accumulator		= 0.0;
	freeSlot->pausedAccumulator	= 0.0;
	freeSlot->forces.clear();
	return freeSlot;
}

//
// Body frame (Havok rigid body: position at +0x110, rotation quaternion
// (x, y, z, w) at +0x150; the motion is embedded at +0xD0)
//

static BYTE* MotionOf(void* dyn)
{
	BYTE** handle = *(BYTE***)((BYTE*)dyn + 0x14);
	if (!handle || !*handle)
		return nullptr;
	return *handle + 0xD0;
}

static void Rotate(const float* q, const Vec4& v, Vec4& out, bool inverse)
{
	float qx = inverse ? -q[0] : q[0];
	float qy = inverse ? -q[1] : q[1];
	float qz = inverse ? -q[2] : q[2];
	float qw = q[3];

	// t = 2 * cross(q, v); out = v + w * t + cross(q, t)
	float tx = 2.0f * (qy * v.z - qz * v.y);
	float ty = 2.0f * (qz * v.x - qx * v.z);
	float tz = 2.0f * (qx * v.y - qy * v.x);

	out.x = v.x + qw * tx + (qy * tz - qz * ty);
	out.y = v.y + qw * ty + (qz * tx - qx * tz);
	out.z = v.z + qw * tz + (qx * ty - qy * tx);
	out.w = v.w;
}

static void ToBody(BYTE* motion, const Vec4& world, Vec4& body, bool isPoint)
{
	const float* position = (const float*)(motion + 0x40);
	const float* rotation = (const float*)(motion + 0x80);

	Vec4 v = world;
	if (isPoint)
	{
		v.x -= position[0];
		v.y -= position[1];
		v.z -= position[2];
	}

	Rotate(rotation, v, body, true);
}

static void ToWorld(BYTE* motion, const Vec4& body, Vec4& world, bool isPoint)
{
	const float* position = (const float*)(motion + 0x40);
	const float* rotation = (const float*)(motion + 0x80);

	Rotate(rotation, body, world, false);

	if (isPoint)
	{
		world.x += position[0];
		world.y += position[1];
		world.z += position[2];
	}
}

//
// Recording
//

static BYTE*			recordMotion	= nullptr;	// Body being recorded
static float			recordDt		= 0.0f;		// Time the recorded forces are applied over
static VehicleState*	recordState		= nullptr;

static void Record(BYTE* motion, int type, const Vec4* force, const Vec4* point)
{
	RecordedForce r;
	r.type = type;
	ToBody(motion, *force, r.force, false);
	if (point)
		ToBody(motion, *point, r.point, true);
	else
		r.point = { 0.0f, 0.0f, 0.0f, 0.0f };

	recordState->forces.push_back(r);
}

static void __fastcall Hook_ApplyPointForce(void* motion, void* edx, float dt, const Vec4* force, const Vec4* point)
{
	MotionClass* mc = MotionClassOf(motion);

	if (motion == recordMotion)
	{
		Record((BYTE*)motion, FORCE_POINT, force, point);
		dt = recordDt;
	}

	mc->applyPointForce(motion, edx, dt, force, point);
}

static void __fastcall Hook_ApplyForce(void* motion, void* edx, float dt, const Vec4* force)
{
	MotionClass* mc = MotionClassOf(motion);

	if (motion == recordMotion)
	{
		Record((BYTE*)motion, FORCE_LINEAR, force, nullptr);
		dt = recordDt;
	}

	mc->applyForce(motion, edx, dt, force);
}

static void __fastcall Hook_ApplyTorque(void* motion, void* edx, float dt, const Vec4* torque)
{
	MotionClass* mc = MotionClassOf(motion);

	if (motion == recordMotion)
	{
		Record((BYTE*)motion, FORCE_TORQUE, torque, nullptr);
		dt = recordDt;
	}

	mc->applyTorque(motion, edx, dt, torque);
}

static void Replay(VehicleState* state, BYTE* motion, float dt)
{
	MotionClass* mc = MotionClassOf(motion);
	if (!mc)
		return;

	for (size_t i = 0; i < state->forces.size(); i++)
	{
		const RecordedForce& r = state->forces[i];

		Vec4 force;
		ToWorld(motion, r.force, force, false);

		switch (r.type)
		{
		case FORCE_POINT:
		{
			Vec4 point;
			ToWorld(motion, r.point, point, true);
			mc->applyPointForce(motion, nullptr, dt, &force, &point);
			break;
		}
		case FORCE_LINEAR:
			mc->applyForce(motion, nullptr, dt, &force);
			break;
		case FORCE_TORQUE:
			mc->applyTorque(motion, nullptr, dt, &force);
			break;
		}
	}
}

//
// Update wrapper
//

static void RunVisual(void* dyn, int kind, float dt)
{
	if (!classes[kind].visualStubbed)
		return;

	// Cars only run it while physics is enabled, bikes always do
	if (kind == KIND_CAR && !(physicsEnabledFlag && *(BYTE**)physicsEnabledFlag && (*(BYTE**)physicsEnabledFlag)[4]))
		return;

	VisualFn visual = (VisualFn)(*(void***)dyn)[9];
	visual(dyn, nullptr, dt);
}

static void __fastcall Wrap_Update(void* dyn, void* edx, float dt, int flag, int reduced, int kind)
{
	const float step = 1.0f / 30.0f;
	UpdateFn update = classes[kind].update;
	VehicleState* state = GetState(dyn);

	if (!state)
	{
		update(dyn, edx, dt, flag, reduced);
		RunVisual(dyn, kind, dt);
		return;
	}

	bool continuous = (state->lastFrame + 1 == frameCounter || state->lastFrame == frameCounter);
	state->lastFrame = frameCounter;

	if (reduced)
	{
		// The paused update uses a fixed 1/30 s every frame
		if (fabsf(dt - step) < 1e-6f)
		{
			if (!continuous)
				state->pausedAccumulator = step;

			state->pausedAccumulator += frameTime;
			if (state->pausedAccumulator < step)
				return;

			state->pausedAccumulator = fmod(state->pausedAccumulator, (double)step);

			SetPhysicsOriginal(true);
			update(dyn, edx, dt, flag, reduced);
			RunVisual(dyn, kind, dt);
			SetPhysicsOriginal(false);
			return;
		}

		// Settle/reset update
		update(dyn, edx, dt, flag, reduced);
		RunVisual(dyn, kind, dt);
		return;
	}

	if (physicsRate <= 0)
	{
		update(dyn, edx, dt, flag, reduced);
		return;
	}

	BYTE* motion = MotionOf(dyn);
	if (!motion || !MotionClassOf(motion))
	{
		update(dyn, edx, dt, flag, reduced);
		RunVisual(dyn, kind, dt);
		return;
	}

	// Start with an update when the vehicle was not updated last frame
	if (!continuous)
		state->accumulator = step;
	else
		state->accumulator += dt;

	int steps = 0;
	while (state->accumulator >= step - 1e-6 && steps < 4)
	{
		state->accumulator -= step;
		steps++;
	}

	if (steps == 4)
		state->accumulator = 0.0;

	if (steps > 0)
	{
		SetPhysicsOriginal(true);

		recordMotion	= motion;
		recordDt		= dt / steps;
		recordState		= state;

		for (int i = 0; i < steps; i++)
		{
			state->forces.clear();
			update(dyn, edx, step, flag, reduced);
		}

		recordMotion	= nullptr;
		recordState		= nullptr;

		SetPhysicsOriginal(false);
	}
	else
	{
		Replay(state, motion, dt);
	}

	RunVisual(dyn, kind, dt);
}

static void __fastcall Wrap_UpdateCar(void* dyn, void* edx, float dt, int flag, int reduced)
{
	Wrap_Update(dyn, edx, dt, flag, reduced, KIND_CAR);
}

static void __fastcall Wrap_UpdateBike(void* dyn, void* edx, float dt, int flag, int reduced)
{
	Wrap_Update(dyn, edx, dt, flag, reduced, KIND_BIKE);
}

//
// Locating things at runtime
//

//
// Readable memory of the main module. Section names differ between builds of
// the game, so the RTTI data is searched in every readable region.
//
struct Region
{
	BYTE* start;
	BYTE* end;
};

static Region regions[64];
static size_t numRegions = 0;

static void CollectRegions()
{
	if (numRegions)
		return;

	BYTE* base = (BYTE*)GetModuleHandle(NULL);
	IMAGE_NT_HEADERS* nt = (IMAGE_NT_HEADERS*)(base + ((IMAGE_DOS_HEADER*)base)->e_lfanew);
	BYTE* end = base + nt->OptionalHeader.SizeOfImage;

	const DWORD readable = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
		PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;

	for (BYTE* ptr = base; ptr < end && numRegions < 64; )
	{
		MEMORY_BASIC_INFORMATION info;
		if (!VirtualQuery(ptr, &info, sizeof(info)))
			break;

		BYTE* regionEnd = (BYTE*)info.BaseAddress + info.RegionSize;
		if (regionEnd > end)
			regionEnd = end;

		if (info.State == MEM_COMMIT && (info.Protect & readable) && !(info.Protect & PAGE_GUARD))
		{
			// Merge with the previous region if contiguous
			if (numRegions && regions[numRegions - 1].end == ptr)
				regions[numRegions - 1].end = regionEnd;
			else
				regions[numRegions++] = { ptr, regionEnd };
		}

		ptr = regionEnd;
	}
}

static bool IsReadable(const void* p, size_t length)
{
	for (size_t r = 0; r < numRegions; r++)
	{
		if ((const BYTE*)p >= regions[r].start && (const BYTE*)p + length <= regions[r].end)
			return true;
	}
	return false;
}

static BYTE* FindDword(DWORD value, BYTE* from = nullptr)
{
	for (size_t r = 0; r < numRegions; r++)
	{
		BYTE* start = regions[r].start;
		if (from)
		{
			if (from >= regions[r].end)
				continue;
			if (from > start)
				start = from;
		}

		for (BYTE* p = start; p + 4 <= regions[r].end; p += 4)
		{
			if (*(DWORD*)p == value)
				return p;
		}
	}

	return nullptr;
}

// Finds the primary vtable of a class from its RTTI type descriptor name
static void** FindVTable(const char* rttiName)
{
	CollectRegions();

	size_t nameLength = strlen(rttiName) + 1;

	for (size_t r = 0; r < numRegions; r++)
	{
		for (BYTE* p = regions[r].start; p + nameLength <= regions[r].end; p++)
		{
			p = (BYTE*)memchr(p, rttiName[0], regions[r].end - p);
			if (!p || p + nameLength > regions[r].end)
				break;

			if (memcmp(p, rttiName, nameLength) != 0)
				continue;

			DWORD typeDescriptor = (DWORD)(p - 8);

			// Complete object locator: signature 0, offset 0, ..., type descriptor at +12
			for (BYTE* ref = FindDword(typeDescriptor); ref; ref = FindDword(typeDescriptor, ref + 4))
			{
				DWORD* col = (DWORD*)(ref - 12);
				if (!IsReadable(col, 16) || col[0] != 0 || col[1] != 0)
					continue;

				BYTE* vtableRef = FindDword((DWORD)col);
				if (vtableRef)
					return (void**)(vtableRef + 4);
			}
		}
	}

	return nullptr;
}

//
// Slot 9 (visual update) call sites inside the vehicle updates. The call is
// replaced with "pop reg; nop", which drops the pushed dt argument.
//
//   fld [ebp+8]; mov eax, [edi]; mov edx, [eax+0x24]; push ecx; mov ecx, edi;
//   fstp [esp]; call edx
//
static const int patCarVisualCall[] =
{
	0xD9, 0x45, 0x08, 0x8B, 0x07, 0x8B, 0x50, 0x24,
	0x51, 0x8B, 0xCF, 0xD9, 0x1C, 0x24, 0xFF, 0xD2
};

//
//   mov eax, [edx+0x24]; push ecx; fstp [esp]; mov byte [edi+0x1C], 1;
//   movss [edi+0x6F8], xmm0; call eax; mov ecx, [$physics]; cmp byte [ecx+4], 0
//
static const int patBikeVisualCall[] =
{
	0x8B, 0x42, 0x24, 0x51, 0xD9, 0x1C, 0x24, 0xC6,
	0x47, 0x1C, 0x01, 0xF3, 0x0F, 0x11, 0x87, 0xF8,
	0x06, 0x00, 0x00, 0xFF, 0xD0, 0x8B, 0x0D, -1,
	-1, -1, -1, 0x80, 0x79, 0x04, 0x00
};

static bool Fail(const wchar_t* what)
{
	HandleError(L"[V-PATCH] Fixed-rate vehicle physics disabled", what);
	return false;
}

void ApplyPatch_PhysicsRate()
{
	// Locate everything first, so that nothing is patched if anything is missing
	for (int k = 0; k < NUM_KINDS; k++)
	{
		classes[k].vtable = FindVTable(classes[k].rttiName);
		if (!classes[k].vtable) { Fail(L"Could not find the vehicle dynamics classes."); return; }
	}

	for (size_t i = 0; i < numMotions; i++)
	{
		motions[i].vtable = FindVTable(motions[i].rttiName);
		if (!motions[i].vtable) { Fail(L"Could not find the Havok motion classes."); return; }
	}

	BYTE* carVisual = nullptr;
	BYTE* bikeVisual = nullptr;

	if (physicsRate > 0)
	{
		carVisual = FindUniqueCode(patCarVisualCall, sizeof(patCarVisualCall) / sizeof(int));
		bikeVisual = FindUniqueCode(patBikeVisualCall, sizeof(patBikeVisualCall) / sizeof(int));
		if (!carVisual || !bikeVisual) { Fail(L"Could not find the vehicle visual update calls."); return; }

		physicsEnabledFlag = *(BYTE**)(bikeVisual + 23);
	}

	// Vehicle updates (slot 4)
	for (int k = 0; k < NUM_KINDS; k++)
	{
		classes[k].update = (UpdateFn)classes[k].vtable[4];

		void* wrapper = (k == KIND_CAR) ? (void*)&Wrap_UpdateCar : (void*)&Wrap_UpdateBike;
		if (!MemWrite(&classes[k].vtable[4], &wrapper, sizeof(wrapper))) return;
	}

	if (physicsRate <= 0)
		return;

	// Visual update calls
	BYTE popEdx[] = { 0x5A, 0x90 };
	BYTE popEax[] = { 0x58, 0x90 };
	if (!MemWrite(carVisual + 14, popEdx, sizeof(popEdx)))	return;
	classes[KIND_CAR].visualStubbed = true;
	if (!MemWrite(bikeVisual + 19, popEax, sizeof(popEax)))	return;
	classes[KIND_BIKE].visualStubbed = true;

	// Havok motion force functions
	for (size_t i = 0; i < numMotions; i++)
	{
		MotionClass& m = motions[i];
		m.applyPointForce	= (PointForceFn)m.vtable[0x54 / 4];
		m.applyForce		= (ForceFn)m.vtable[0x58 / 4];
		m.applyTorque		= (ForceFn)m.vtable[0x5C / 4];

		void* hooks[3] = { (void*)&Hook_ApplyPointForce, (void*)&Hook_ApplyForce, (void*)&Hook_ApplyTorque };
		if (!MemWrite(&m.vtable[0x54 / 4], hooks, sizeof(hooks))) return;
	}
}

void PhysicsRate_NewFrame(double newFrameTime)
{
	frameTime = newFrameTime;
	frameCounter++;
}
