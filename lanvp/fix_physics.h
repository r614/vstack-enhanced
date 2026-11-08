// ----------------------------------------------------------------------------
// fix_physics.h
//
// Part of vstack-enhanced, a fork of V-Patch (c) 2021-2024 Vaana
// ----------------------------------------------------------------------------

#pragma once

void ApplyPatch_Physics();
void UpdatePhysicsConstants(double multiplier);
void SetPhysicsOriginal(bool original);

// Fixed-rate vehicle physics (fix_physics_rate.cpp)
extern int physicsRate;
void ApplyPatch_PhysicsRate();
void PhysicsRate_NewFrame(double frameTime);
