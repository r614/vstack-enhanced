// ----------------------------------------------------------------------------
// fix_physics.cpp
//
// Part of vstack-enhanced, a fork of V-Patch (c) 2021-2024 Vaana
// ----------------------------------------------------------------------------

#include "fix_physics.h"

#include "patching.h"
#include <cassert>
#include <cmath>
#include <wchar.h>

//
// The vehicle code was tuned for a fixed 30 FPS. Most of it scales its
// forces with the frame time, but some of the state is still advanced by a
// fixed amount each frame: low-pass filters ("x += (target - x) * k"),
// decays ("x *= r") and countdowns ("x -= s"). At higher frame rates these
// run more often, so the car reacts faster than it should.
//
// Every one of these reads its constant from memory, so the fix redirects
// the instruction's memory operand to a value of our own, which is
// recomputed every frame from the frame time:
//
//   m = frametime / (1/30)
//
//   LERP (fraction k)        k' = 1 - (1 - k)^m
//   LERP_DIV (divisor d)     d' = 1 / (1 - (1 - 1/d)^m)
//   DECAY (retention r)      r' = r^m
//   STEP (amount s)          s' = s * m
//   INVERSE (factor c)       c' = c / m      (cancels a stray "* dt * 30")
//   SQUARE (factor c)        c' = c * m^2
//   DECAY_COMP (factor c)    c' = c * r / r^m  (r = aux; see the panel springs)
//
// A few sites need a code change instead (see codePatches below).
//
// At 30 FPS (m = 1) every value is identical to the original constant.
//

#define ANY -1

enum ScaleKind
{
	SCALE_LERP,
	SCALE_LERP_DIV,
	SCALE_DECAY,
	SCALE_STEP,
	SCALE_INVERSE,
	SCALE_SQUARE,
	SCALE_DECAY_COMP
};

struct FrameConstant
{
	const wchar_t*	name;
	const int*		pattern;		// Bytes to match, ANY for wildcards
	size_t			patternLength;
	size_t			operandOffset;	// Offset of the absolute address operand inside the pattern
	bool			isDouble;		// qword (true) or dword (false) operand
	double			original;		// Value the operand must point to
	ScaleKind		kind;
	double			aux;			// Extra parameter for some kinds

	double			valueDouble;	// Replacement storage
	float			valueFloat;
	bool			applied;
};

#define PATTERN(p) p, sizeof(p) / sizeof(p[0])

//
// CAR: collision/impact factor countdown (CarDynamicsInstance update)
//
// [this+0x72C] is decremented by 0.1 every frame and clamped to [0, 1],
// i.e. it lasts 10 frames (1/3 s at 30 FPS).
//
//   movss xmm1, [edi+0x72C]
//   xorps xmm2, xmm2
//   comiss xmm1, xmm2
//   jbe +0x17
//   cvtps2pd xmm0, xmm1
//   subsd xmm0, qword ptr [$0.1]
//
static const int patCarImpactCountdown[] =
{
	0xF3, 0x0F, 0x10, 0x8F, 0x2C, 0x07, 0x00, 0x00,
	0x0F, 0x57, 0xD2,
	0x0F, 0x2F, 0xCA,
	0x76, 0x17,
	0x0F, 0x5A, 0xC1,
	0xF2, 0x0F, 0x5C, 0x05, ANY, ANY, ANY, ANY
};

//
// CAR: slope force along the forward axis (FUN_00e5d280)
//
// The force is multiplied by dt and then by 30 before being handed to
// Havok's applyForce(dt), which multiplies by dt again. The impulse thus
// scales with dt^2 (the same bug as the braking force fixed in fix_fps.cpp).
// Scaling the 30 by 1/m turns "dt * 30" into 1.
//
//   movss xmm0, [ebp+8]        ; dt
//   movaps xmm1, [esp+0x10]
//   lea edx, [esp+0x10]
//   shufps xmm0, xmm0, 0
//   push edx
//   mulps xmm0, xmm1
//   call $store
//   movss xmm0, dword ptr [$30.0]
//
static const int patCarSlopeForce[] =
{
	0xF3, 0x0F, 0x10, 0x45, 0x08,
	0x0F, 0x28, 0x4C, 0x24, 0x10,
	0x8D, 0x54, 0x24, 0x10,
	0x0F, 0xC6, 0xC0, 0x00,
	0x52,
	0x0F, 0x59, 0xC1,
	0xE8, ANY, ANY, ANY, ANY,
	0xF3, 0x0F, 0x10, 0x05, ANY, ANY, ANY, ANY
};

//
// CAR: steering self-centering (CarDynamicsInstance slot 15, FUN_00e52390)
//
// With no steering input the steering value [this+0x24] returns towards
// zero by a fixed step every frame, so it re-centers faster at higher
// frame rates:
//
//   step = |speed| / 30000 + 0.0015
//
// Both terms are scaled by m.
//
//   divsd xmm3, qword ptr [$30000.0]
//   movss xmm5, dword ptr [$0.004]
//
static const int patCarSteerCenterSpeed[] =
{
	0xF2, 0x0F, 0x5E, 0x1D, ANY, ANY, ANY, ANY,
	0xF3, 0x0F, 0x10, 0x2D, ANY, ANY, ANY, ANY
};

//
//   addsd xmm3, qword ptr [$0.0015]
//   cvtsd2ss xmm3, xmm3
//
static const int patCarSteerCenterBase[] =
{
	0xF2, 0x0F, 0x58, 0x1D, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x5A, 0xDB
};

//
// CAR: wheelspin steering shake phase (FUN_00e5db70)
//
// While the drive wheels spin, each wheel's phase [wheel+0x384] advances by
// a fixed 3.2345 per frame. Wheel 0's phase drives a sin() shake that is
// added to the steering angle, so the shake gets faster with the frame rate.
//
//   movsd xmm0, qword ptr [$3.2345]
//   movss xmm1, dword ptr [$0.3]
//   lea eax, [edi+0x384]
//
static const int patCarWheelspinPhase[] =
{
	0xF2, 0x0F, 0x10, 0x05, ANY, ANY, ANY, ANY,
	0xF3, 0x0F, 0x10, 0x0D, ANY, ANY, ANY, ANY,
	0x8D, 0x87, 0x84, 0x03, 0x00, 0x00
};

//
// CAR: skid intensity [this+0x74C] (FUN_00e60340)
//
// The value is smoothed towards a target whose additive terms are
// multiplied by dt * 30 (dt * xmm5), so the value it settles at shrinks
// with the frame rate. xmm5 (30.0) is replaced by 30/m so that dt * xmm5
// stays 1, and the 0.2 smoothing factor (which is also multiplied by that
// same 1) becomes a proper per-frame fraction.
//
//   movsd xmm5, qword ptr [$30.0]
//   cvtps2pd xmm0, xmm0
//   mulsd xmm0, qword ptr [...]
//
static const int patCarSkidTargetScale[] =
{
	0xF2, 0x0F, 0x10, 0x2D, ANY, ANY, ANY, ANY,
	0x0F, 0x5A, 0xC0,
	0xF2, 0x0F, 0x59, 0x05, ANY, ANY, ANY, ANY
};

//
//   mulsd xmm1, qword ptr [$0.2]
//   mulsd xmm1, xmm0
//
static const int patCarSkidSmoothing[] =
{
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x59, 0xC8
};

//
// CAR: frame-to-frame acceleration [this+0x7C0] (base slot 2, FUN_00e5fe00)
//
//   [0x7C0] = (previous velocity - velocity) * dt / 30
//
// The velocity change over a frame already scales with dt, so this scales
// with dt^2. It feeds the tyre grip (through [0x758]), the at-rest check and
// the cosmetic panel springs below. Scaling the 30 by m^2 keeps it at its
// 30 FPS value.
//

static const int patCarAccel[] =
{
	0xF2, 0x0F, 0x5E, 0x05, ANY, ANY, ANY, ANY,
	0x0F, 0x28, 0x4C, 0x24, 0x10
};

//
// COSMETIC: wheels and body panels of driven cars (slot 9, FUN_00e57eb0)
//
// Wheel spin-down while airborne, and the spring/damper animations of loose
// hood, trunk and bumpers on damaged cars. Their velocities are damped by a
// fixed factor and kicked by fixed amounts every frame.
//
// The trunk spring loads 0.8 into a register that is used both for the
// stiffness (with dt * 30) and the per-frame damping, so the damping becomes
// 0.8^m and the 30 is compensated to 24 / 0.8^m to keep the stiffness.
//

static const int patWheelAirSpinA[] =
{
	0xF3, 0x0F, 0x10, 0x80, 0xA8, 0x00, 0x00, 0x00,
	0x0F, 0x5A, 0xC0, 0xF2, 0x0F, 0x59, 0x05, ANY,
	ANY, ANY, ANY
};

static const int patWheelAirSpinB[] =
{
	0x8B, 0x74, 0x24, 0x14, 0x0F, 0x5A, 0xC0, 0xF2,
	0x0F, 0x59, 0x05, ANY, ANY, ANY, ANY
};

static const int patHoodDamping[] =
{
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xD9, 0xF3, 0x0F, 0x11, 0x9B,
	0xE8, 0x06, 0x00, 0x00
};

static const int patTrunkDamping[] =
{
	0xF2, 0x0F, 0x10, 0x0D, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x59, 0xD1, 0x0F, 0x5A, 0xF5
};

static const int patTrunkStiffness[] =
{
	0xF2, 0x0F, 0x59, 0x15, ANY, ANY, ANY, ANY,
	0xF3, 0x0F, 0x10, 0xB3, 0x18, 0x07, 0x00, 0x00
};

static const int patTrunkKick[] =
{
	0xF2, 0x0F, 0x59, 0x15, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x5C, 0xCA, 0x66, 0x0F, 0x5A, 0xC9,
	0xF3, 0x0F, 0x11, 0x8B, 0x18, 0x07, 0x00, 0x00
};

static const int patBumperADamping[] =
{
	0x66, 0x0F, 0x5A, 0xCF, 0xF3, 0x0F, 0x5A, 0xC9,
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY
};

static const int patBumperAKick[] =
{
	0xF2, 0x0F, 0x59, 0xF7, 0xF2, 0x0F, 0x59, 0x35,
	ANY, ANY, ANY, ANY, 0x0F, 0x5A, 0xC9
};

static const int patBumperBDamping[] =
{
	0x66, 0x0F, 0x5A, 0xCB, 0xF3, 0x0F, 0x5A, 0xC9,
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xC9
};

static const int patBumperBKick[] =
{
	0xF3, 0x0F, 0x10, 0x93, 0xEC, 0x06, 0x00, 0x00,
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY
};

static const int patBumperBRoll[] =
{
	0xF2, 0x0F, 0x58, 0x0D, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xC9, 0xF3, 0x0F, 0x11, 0x8B,
	0xEC, 0x06, 0x00, 0x00
};

//
// COSMETIC: body panels of non-simulated cars (slot 10, FUN_00e5b060)
//
// A copy of the panel springs above, run every frame from slot 5 for cars
// that are not being physically simulated. Unlike slot 9, the kicks here are
// not multiplied by dt * 30.
//

static const int patPoseHoodDamping[] =
{
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xD1
};

static const int patPoseHoodKick[] =
{
	0x0F, 0x5A, 0xD2, 0xF2, 0x0F, 0x59, 0xCA, 0xF2,
	0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY, 0xF2,
	0x0F, 0x5C, 0xC1
};

static const int patPoseHoodBias[] =
{
	0xF2, 0x0F, 0x5C, 0x05, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xC0, 0xF3, 0x0F, 0x11, 0x83,
	0xE8, 0x06, 0x00, 0x00
};

static const int patPoseHoodRoll[] =
{
	0xF2, 0x0F, 0x58, 0x05, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xC0, 0xF3, 0x0F, 0x11, 0x83,
	0xE8, 0x06, 0x00, 0x00
};

static const int patPoseTrunkDamping[] =
{
	0xF2, 0x0F, 0x10, 0x0D, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x59, 0xD1, 0x0F, 0x5A, 0xEF
};

static const int patPoseTrunkStiffness[] =
{
	0xF2, 0x0F, 0x59, 0x15, ANY, ANY, ANY, ANY,
	0xF3, 0x0F, 0x10, 0xAB, 0x18, 0x07, 0x00, 0x00
};

static const int patPoseTrunkKick[] =
{
	0xF3, 0x0F, 0x10, 0x8B, 0x18, 0x07, 0x00, 0x00,
	0x0F, 0x5A, 0xD2, 0xF2, 0x0F, 0x59, 0x15, ANY,
	ANY, ANY, ANY
};

static const int patPoseBumperADamping[] =
{
	0x66, 0x0F, 0x5A, 0xCD, 0xF3, 0x0F, 0x5A, 0xC9,
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xC9
};

static const int patPoseBumperAKick[] =
{
	0xF2, 0x0F, 0x59, 0x2D, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x5C, 0xCD
};

static const int patPoseBumperBDamping[] =
{
	0xF2, 0x0F, 0x5A, 0xCB, 0xF3, 0x0F, 0x5A, 0xC9,
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY
};

static const int patPoseBumperBKick[] =
{
	0xF3, 0x0F, 0x10, 0x8B, 0xEC, 0x06, 0x00, 0x00,
	0xF2, 0x0F, 0x59, 0x05, ANY, ANY, ANY, ANY
};

static const int patPoseBumperBRoll[] =
{
	0xF2, 0x0F, 0x58, 0x05, ANY, ANY, ANY, ANY,
	0x66, 0x0F, 0x5A, 0xC0, 0xF3, 0x0F, 0x11, 0x83,
	0xEC, 0x06, 0x00, 0x00
};

//
// COSMETIC: body sway of non-simulated cars (slot 5, FUN_00e5c7d0)
//
// Visual steering lean [0x50] (low-pass, divisor 3) and a body roll/pitch
// spring ([0x74]/[0x7C] positions, [0x78]/[0x80] velocities) advanced as a
// per-frame discrete oscillator: v -= |p| * p / 25; v *= 0.95; p += 3 * v,
// kicked by the acceleration [0x7C0]. Scaling the position step and the
// spring by m (and the kicks by m) keeps its frequency and amplitude.
//

static const int patSwayLean[] =
{
	0xF2, 0x0F, 0x5E, 0x0D, ANY, ANY, ANY, ANY,
	0x0F, 0x5A, 0xC0, 0xF2, 0x0F, 0x58, 0xC8, 0x66,
	0x0F, 0x5A, 0xC1, 0x0F, 0x5A, 0xC8
};

static const int patSwayRestA[] =
{
	0xF2, 0x0F, 0x10, 0x0D, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x59, 0xD1, 0xF2, 0x0F, 0x5A, 0xD2,
	0xF3, 0x0F, 0x11, 0x56, 0x74
};

static const int patSwayRestB[] =
{
	0xF2, 0x0F, 0x10, 0x05, ANY, ANY, ANY, ANY,
	0xF2, 0x0F, 0x59, 0xC8, 0xF2, 0x0F, 0x5A, 0xC9,
	0xF3, 0x0F, 0x11, 0x4E, 0x74
};

static const int patSwaySpring[] =
{
	0xF2, 0x0F, 0x10, 0x35, ANY, ANY, ANY, ANY,
	0x0F, 0x5A, 0xC9, 0xF2, 0x0F, 0x5E, 0xC6
};

static const int patSwayDamping[] =
{
	0xF2, 0x0F, 0x10, 0x15, ANY, ANY, ANY, ANY,
	0x0F, 0x28, 0xE8
};

static const int patSwayStep[] =
{
	0xF2, 0x0F, 0x10, 0x0D, ANY, ANY, ANY, ANY,
	0x0F, 0x5A, 0xD2, 0x0F, 0x5A, 0xC0
};

static const int patSwayKickPitch[] =
{
	0xF2, 0x0F, 0x59, 0x0D, ANY, ANY, ANY, ANY,
	0x0F, 0x5A, 0xC0, 0xF2, 0x0F, 0x5C, 0xC1, 0x66,
	0x0F, 0x6F, 0x4C, 0x24, 0x10
};

static const int patSwayKickRoll[] =
{
	0xF2, 0x0F, 0x59, 0x05, ANY, ANY, ANY, ANY,
	0x0F, 0x5A, 0xC9, 0xF2, 0x0F, 0x58, 0xC1, 0x66,
	0x0F, 0x5A, 0xD8
};

static FrameConstant constants[] =
{
	{ L"Car impact countdown",	PATTERN(patCarImpactCountdown),	23, true,	0.1,	SCALE_STEP },
	{ L"Car slope force",		PATTERN(patCarSlopeForce),		31, false,	30.0,	SCALE_INVERSE },
	{ L"Steering centering (speed)",	PATTERN(patCarSteerCenterSpeed),	4, true,	30000.0,	SCALE_INVERSE },
	{ L"Steering centering (base)",		PATTERN(patCarSteerCenterBase),		4, true,	0.0015,		SCALE_STEP },
	{ L"Wheelspin shake phase",			PATTERN(patCarWheelspinPhase),		4, true,	3.2345142364501953, SCALE_STEP },
	{ L"Skid intensity target",			PATTERN(patCarSkidTargetScale),		4, true,	30.0,		SCALE_INVERSE },
	{ L"Skid intensity smoothing",		PATTERN(patCarSkidSmoothing),		4, true,	0.2,		SCALE_LERP },
	{ L"Car acceleration estimate",	PATTERN(patCarAccel),	4, true,	30.0,	SCALE_SQUARE },
	{ L"Wheel air spin-down A",	PATTERN(patWheelAirSpinA),	15, true,	0.949999988079071,	SCALE_DECAY },
	{ L"Wheel air spin-down B",	PATTERN(patWheelAirSpinB),	11, true,	0.550000011920929,	SCALE_DECAY },
	{ L"Hood damping",	PATTERN(patHoodDamping),	4, true,	0.8999999761581421,	SCALE_DECAY },
	{ L"Trunk damping",	PATTERN(patTrunkDamping),	4, true,	0.800000011920929,	SCALE_DECAY },
	{ L"Trunk stiffness",	PATTERN(patTrunkStiffness),	4, true,	30.0,	SCALE_DECAY_COMP, 0.800000011920929 },
	{ L"Trunk kick",	PATTERN(patTrunkKick),	4, true,	0.20000000298023224,	SCALE_STEP },
	{ L"Bumper A damping",	PATTERN(patBumperADamping),	12, true,	0.9399999976158142,	SCALE_DECAY },
	{ L"Bumper A kick",	PATTERN(patBumperAKick),	8, true,	0.125,	SCALE_STEP },
	{ L"Bumper B damping",	PATTERN(patBumperBDamping),	12, true,	0.8500000238418579,	SCALE_DECAY },
	{ L"Bumper B kick",	PATTERN(patBumperBKick),	12, true,	0.3499999940395355,	SCALE_STEP },
	{ L"Bumper B rollover kick",	PATTERN(patBumperBRoll),	4, true,	0.5,	SCALE_STEP },
	{ L"Pose hood damping",	PATTERN(patPoseHoodDamping),	4, true,	0.8999999761581421,	SCALE_DECAY },
	{ L"Pose hood kick",	PATTERN(patPoseHoodKick),	11, true,	0.15000000596046448,	SCALE_STEP },
	{ L"Pose hood bias",	PATTERN(patPoseHoodBias),	4, true,	0.07999999821186066,	SCALE_STEP },
	{ L"Pose hood rollover kick",	PATTERN(patPoseHoodRoll),	4, true,	0.5,	SCALE_STEP },
	{ L"Pose trunk damping",	PATTERN(patPoseTrunkDamping),	4, true,	0.800000011920929,	SCALE_DECAY },
	{ L"Pose trunk stiffness",	PATTERN(patPoseTrunkStiffness),	4, true,	30.0,	SCALE_DECAY_COMP, 0.800000011920929 },
	{ L"Pose trunk kick",	PATTERN(patPoseTrunkKick),	15, true,	0.20000000298023224,	SCALE_STEP },
	{ L"Pose bumper A damping",	PATTERN(patPoseBumperADamping),	12, true,	0.9399999976158142,	SCALE_DECAY },
	{ L"Pose bumper A kick",	PATTERN(patPoseBumperAKick),	4, true,	0.125,	SCALE_STEP },
	{ L"Pose bumper B damping",	PATTERN(patPoseBumperBDamping),	12, true,	0.8500000238418579,	SCALE_DECAY },
	{ L"Pose bumper B kick",	PATTERN(patPoseBumperBKick),	12, true,	0.3499999940395355,	SCALE_STEP },
	{ L"Pose bumper B rollover kick",	PATTERN(patPoseBumperBRoll),	4, true,	0.5,	SCALE_STEP },
	{ L"Sway lean smoothing",	PATTERN(patSwayLean),	4, true,	3.0,	SCALE_LERP_DIV },
	{ L"Sway rest decay A",	PATTERN(patSwayRestA),	4, true,	0.8999999761581421,	SCALE_DECAY },
	{ L"Sway rest decay B",	PATTERN(patSwayRestB),	4, true,	0.8999999761581421,	SCALE_DECAY },
	{ L"Sway spring",	PATTERN(patSwaySpring),	4, true,	25.0,	SCALE_INVERSE },
	{ L"Sway damping",	PATTERN(patSwayDamping),	4, true,	0.949999988079071,	SCALE_DECAY },
	{ L"Sway position step",	PATTERN(patSwayStep),	4, true,	3.0,	SCALE_STEP },
	{ L"Sway pitch kick",	PATTERN(patSwayKickPitch),	4, true,	2.0,	SCALE_STEP },
	{ L"Sway roll kick",	PATTERN(patSwayKickRoll),	4, true,	0.5,	SCALE_STEP },
};

static const size_t numConstants = sizeof(constants) / sizeof(constants[0]);

struct CodePatch
{
	const wchar_t*	name;
	const int*		pattern;
	size_t			patternLength;
	size_t			offset;			// Where the bytes are replaced
	size_t			length;			// How many bytes are replaced with NOPs
};

//
// CAR: roll damping couple (FUN_00e54e30)
//
// A pair of opposite forces damps the body's rotation. The force is divided
// by m = dt * 30 before being applied with applyForce(dt), so the impulse
// per frame is constant and the damping gets stronger the more frames there
// are per second (2x at 60 FPS). Removing the division makes the impulse
// scale with dt; at 30 FPS the result is identical.
//
//   shufps xmm1, xmm1, 0
//   divps xmm0, xmm1           ; / 2
//   movss xmm1, [esp+0x50]     ; m
//   shufps xmm1, xmm1, 0
//   divps xmm0, xmm1           ; / m   <- removed
//
static const int patCarRollDamping[] =
{
	0x0F, 0xC6, 0xC9, 0x00,
	0x0F, 0x5E, 0xC1,
	0xF3, 0x0F, 0x10, 0x4C, 0x24, 0x50,
	0x0F, 0xC6, 0xC9, 0x00,
	0x0F, 0x5E, 0xC1
};

static const CodePatch codePatches[] =
{
	{ L"Car roll damping", PATTERN(patCarRollDamping), 17, 3 },
};

static const size_t numCodePatches = sizeof(codePatches) / sizeof(codePatches[0]);

static bool GetTextSection(BYTE** start, BYTE** end)
{
	BYTE* base = (BYTE*)GetModuleHandle(NULL);
	IMAGE_DOS_HEADER* dos = (IMAGE_DOS_HEADER*)base;
	IMAGE_NT_HEADERS* nt = (IMAGE_NT_HEADERS*)(base + dos->e_lfanew);
	IMAGE_SECTION_HEADER* section = IMAGE_FIRST_SECTION(nt);

	for (WORD i = 0; i < nt->FileHeader.NumberOfSections; i++, section++)
	{
		if (memcmp(section->Name, ".text", 6) == 0)
		{
			*start = base + section->VirtualAddress;
			*end = *start + section->Misc.VirtualSize;
			return true;
		}
	}

	return false;
}

static BYTE* FindUnique(const int* pattern, size_t length, BYTE* start, BYTE* end)
{
	BYTE* found = nullptr;

	for (BYTE* p = start; p + length <= end; p++)
	{
		size_t i = 0;
		while (i < length && (pattern[i] == ANY || p[i] == (BYTE)pattern[i]))
			i++;

		if (i == length)
		{
			if (found)
				return nullptr; // Not unique
			found = p;
		}
	}

	return found;
}

static double ScaleValue(const FrameConstant& c, double multiplier)
{
	switch (c.kind)
	{
	case SCALE_LERP:		return 1.0 - pow(1.0 - c.original, multiplier);
	case SCALE_LERP_DIV:	return 1.0 / (1.0 - pow(1.0 - 1.0 / c.original, multiplier));
	case SCALE_DECAY:		return pow(c.original, multiplier);
	case SCALE_STEP:		return c.original * multiplier;
	case SCALE_INVERSE:		return c.original / multiplier;
	case SCALE_SQUARE:		return c.original * multiplier * multiplier;
	case SCALE_DECAY_COMP:	return c.original * c.aux / pow(c.aux, multiplier);
	}

	return c.original;
}

void ApplyPatch_Physics()
{
	BYTE* textStart;
	BYTE* textEnd;
	if (!GetTextSection(&textStart, &textEnd))
	{
		HandleError(L"[V-PATCH] Physics fix failed!", L"Could not locate the .text section.");
		return;
	}

	const size_t messageLength = 1024;
	wchar_t message[messageLength] = L"The following physics fixes could not be applied:\n";
	bool anyFailed = false;

	for (size_t i = 0; i < numConstants; i++)
	{
		FrameConstant& c = constants[i];
		c.valueDouble	= c.original;
		c.valueFloat	= (float)c.original;
		c.applied		= false;

		BYTE* match = FindUnique(c.pattern, c.patternLength, textStart, textEnd);
		if (match)
		{
			BYTE* operand = match + c.operandOffset;
			void* constAddr = *(void**)operand;

			double current = c.isDouble ? *(double*)constAddr : (double)*(float*)constAddr;

			// Constants are often floats widened to doubles (0.1 -> 0.100000001...)
			if (fabs(current - c.original) <= 1e-6 * fabs(c.original))
			{
				c.original		= current;
				c.valueDouble	= current;
				c.valueFloat	= (float)current;

				void* replacement = c.isDouble ? (void*)&c.valueDouble : (void*)&c.valueFloat;
				c.applied = MemWrite(operand, &replacement, sizeof(replacement));
			}
		}

		if (!c.applied)
		{
			anyFailed = true;
			size_t length = wcslen(message);
			wcscat_s(&message[length], messageLength - length, L"\n    ");
			length = wcslen(message);
			wcscat_s(&message[length], messageLength - length, c.name);
		}
	}

	for (size_t i = 0; i < numCodePatches; i++)
	{
		const CodePatch& p = codePatches[i];

		BYTE* match = FindUnique(p.pattern, p.patternLength, textStart, textEnd);
		if (!match || !MemWriteNop(match + p.offset, p.length))
		{
			anyFailed = true;
			size_t length = wcslen(message);
			wcscat_s(&message[length], messageLength - length, L"\n    ");
			length = wcslen(message);
			wcscat_s(&message[length], messageLength - length, p.name);
		}
	}

	if (anyFailed)
	{
		MessageBoxW(NULL, message, L"[V-PATCH] Physics fix", MB_OK);
	}
}

void UpdatePhysicsConstants(double multiplier)
{
	for (size_t i = 0; i < numConstants; i++)
	{
		FrameConstant& c = constants[i];
		if (!c.applied)
			continue;

		double value = ScaleValue(c, multiplier);
		c.valueDouble	= value;
		c.valueFloat	= (float)value;
	}
}
