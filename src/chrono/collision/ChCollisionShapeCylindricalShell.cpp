// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShapeCylindricalShell.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeCylindricalShell)
CH_UPCASTING(ChCollisionShapeCylindricalShell, ChCollisionShape)

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell() : ChCollisionShape(Type::CYLSHELL) {}

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell(std::shared_ptr<ChContactMaterial> material,
                                                                   double radius,
                                                                   double height)
    : ChCollisionShape(Type::CYLSHELL, material) {
    gcylinder.r = radius;
    gcylinder.h = height;
}

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell(std::shared_ptr<ChContactMaterial> material,
                                                                   double radius,
                                                                   double height,
                                                                   bool closed_bottom,
                                                                   bool closed_top)
    : ChCollisionShape(Type::CYLSHELL, material), closed_bottom(closed_bottom), closed_top(closed_top) {
    gcylinder.r = radius;
    gcylinder.h = height;
}

ChCollisionShapeCylindricalShell::ChCollisionShapeCylindricalShell(std::shared_ptr<ChContactMaterial> material,
                                                                   const ChCylinder& cyl)
    : ChCollisionShape(Type::CYLSHELL, material), gcylinder(cyl) {}

void ChCollisionShapeCylindricalShell::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCollisionShapeCylindricalShell>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gcylinder);
    archive_out << CHNVP(closed_bottom);
    archive_out << CHNVP(closed_top);
}

void ChCollisionShapeCylindricalShell::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCollisionShapeCylindricalShell>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gcylinder);
    archive_in >> CHNVP(closed_bottom);
    archive_in >> CHNVP(closed_top);
}

}  // end namespace chrono
