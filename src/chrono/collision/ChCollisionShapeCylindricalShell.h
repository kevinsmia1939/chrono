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

#ifndef CH_COLLISION_SHAPE_CYLINDRICAL_SHELL_H
#define CH_COLLISION_SHAPE_CYLINDRICAL_SHELL_H

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/geometry/ChCylinder.h"

namespace chrono {

/// @addtogroup chrono_collision
/// @{

/// Collision cylindrical shell shape.
/// When added to a collision model, the cylinder is defined with its axis along the Z direction of the shape frame.
class ChApi ChCollisionShapeCylindricalShell : public ChCollisionShape {
  public:
    ChCollisionShapeCylindricalShell();
    ChCollisionShapeCylindricalShell(std::shared_ptr<ChContactMaterial> material, double radius, double height);
    /// Construct a cylindrical container surface with optional flat end caps.
    /// The bottom cap is at -height/2 and the top cap at +height/2 in the shape frame.
    ChCollisionShapeCylindricalShell(std::shared_ptr<ChContactMaterial> material,
                                     double radius,
                                     double height,
                                     bool closed_bottom,
                                     bool closed_top);
    ChCollisionShapeCylindricalShell(std::shared_ptr<ChContactMaterial> material, const ChCylinder& cyl);

    ~ChCollisionShapeCylindricalShell() {}

    // Access the cylinder geometry.
    ChCylinder& GetGeometry() { return gcylinder; }

    /// Get the cylinder radius.
    double GetRadius() const { return gcylinder.GetRadius(); }

    /// Get the cylinder height.
    double GetHeight() const { return gcylinder.GetHeight(); }

    /// Return true if the shell has a flat cap at -height/2.
    bool IsBottomClosed() const { return closed_bottom; }

    /// Return true if the shell has a flat cap at +height/2.
    bool IsTopClosed() const { return closed_top; }

    /// Enable or disable the flat cap at -height/2.
    void SetBottomClosed(bool closed) { closed_bottom = closed; }

    /// Enable or disable the flat cap at +height/2.
    void SetTopClosed(bool closed) { closed_top = closed; }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gcylinder.GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChCylinder gcylinder;
    bool closed_bottom = false;
    bool closed_top = false;
};

/// @} chrono_collision

}  // end namespace chrono

#endif
