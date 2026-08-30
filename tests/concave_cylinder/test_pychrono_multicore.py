"""Smoke test for the PyChrono multicore NSC system and capped cylinder bed."""

import math

import pychrono.core as chrono
import pychrono.multicore as multicore


system = multicore.ChSystemMulticoreNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_MULTICORE)
system.SetNumThreads(4)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionEnvelope(2e-6)
system.SetSolverMaxIterationsNormal(50)
system.SetSolverMaxIterationsSliding(50)
system.SetContactRecoverySpeed(0.2)

material = chrono.ChContactMaterialNSC()
material.SetFriction(0.35)
material.SetRestitution(0.05)

bed = chrono.ChBody()
bed.SetFixed(True)
bed.EnableCollision(True)
bed.AddCollisionShape(chrono.ChCollisionShapeCylindricalShell(
    material, 0.006, 0.030, True, False))
system.AddBody(bed)

radius = 0.00048
height = 0.00090
mass = 1200 * math.pi * radius**2 * height
particle = chrono.ChBody()
particle.SetMass(mass)
particle.SetInertiaXX(chrono.ChVector3d(
    mass * (3 * radius**2 + height**2) / 12,
    mass * (3 * radius**2 + height**2) / 12,
    0.5 * mass * radius**2))
particle.SetPos(chrono.ChVector3d(0, 0, -0.0125))
particle.EnableCollision(True)
particle.AddCollisionShape(
    chrono.ChCollisionShapeCylinder(material, radius, height))
system.AddBody(particle)

for _ in range(2500):
    system.DoStepDynamics(2e-5)

position = particle.GetPos()
assert all(math.isfinite(value)
           for value in (position.x, position.y, position.z))
assert position.z >= -0.0168, position
assert system.GetNumContacts() > 0
assert system.GetNumThreads() == 4

print("PyChrono multicore analytical cylinder-shell test: PASS")
