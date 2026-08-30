# Multicore capped cylindrical shell

This directory contains a correctness test and benchmark for cylindrical
particles falling inside a cylindrical bed.  The bed can be represented by:

- `ChCollisionShapeCylindricalShell(material, radius, height, true, false)` for
  the new analytical side wall plus closed bottom and open top; or
- a 96-segment concave triangle mesh, for comparison with the previous method.

The capped-shell narrowphase is implemented for cylinder-versus-cylindrical-
shell pairs in `ChCollisionSystemMulticore`.  It is dispatched by the existing
OpenMP pair loop, so different particle/container pairs are processed in
parallel.  Use the `PRIMS` or `HYBRID` narrowphase algorithm; `MPR` alone cannot
represent a concave container.  Other particle shapes against a capped shell
are not part of this implementation.

When both the Multicore and Python modules are enabled, the new
`pychrono.multicore` module exposes `ChSystemMulticoreNSC`.  It interoperates
with bodies, materials, and collision shapes from `pychrono.core` and provides
Python setters for thread count, collision envelope, NSC iteration limits,
solver tolerance and mode, and penetration recovery speed.  Run
`test_pychrono_multicore.py` with the build-tree `pychrono` package to verify a
true multicore NSC solve using the analytical capped cylinder bed.

The benchmark uses millimetre-scale geometry: a 6 mm radius, 30 mm high bed and
0.48 mm radius cylindrical particles.  It verifies that a single falling
particle and a multiple-particle fill remain finite and contained.

## Measured results

Measurements on the development machine used 1,500 steps of 20 microseconds.
Lower elapsed time is better.

| Particles | Threads | Analytical | 96-segment mesh | Mesh / analytical |
|---:|---:|---:|---:|---:|
| 120 | 1 | 1.737499 s | 3.381998 s | 1.946x |
| 120 | 8 | 1.465911 s | 2.964744 s | 2.022x |
| 600 | 1 | 6.996769 s | 8.998814 s | 1.286x |
| 600 | 8 | 4.311993 s | 5.562122 s | 1.290x |

All benchmark runs reported finite, contained particles.  The 600-particle
measurements above use the installed openSUSE RPMs.  The analytical case
improved from 6.996769 seconds with one thread to 4.311993 seconds with eight
threads (1.62x).  Scaling is limited by solver work and serial overhead as well
as collision detection, so the result is not expected to scale linearly with
the thread count.

## Focused development build

For collision development, configure only Chrono core, Multicore, and tests.
Leave Vehicle, Parsers, Cascade, Modal, Peridynamics, Postprocess, Synchrono,
and PyChrono disabled unless the change specifically needs them.  The full RPM
build deliberately enables those package features and therefore takes much
longer.
