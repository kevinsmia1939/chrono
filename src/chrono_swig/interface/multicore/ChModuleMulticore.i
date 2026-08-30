// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Python wrapper for Chrono::Multicore systems.
// =============================================================================

%feature("autodoc", "1");
%feature("flatnested", "1");

%include "exception.i"
%exception {
    try {
        $action
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    }
}

%include "std_string.i"
%include "std_vector.i"
%include <std_shared_ptr.i>
%include "../chrono_cast.i"

%{
#define SWIG_FILE_WITH_INIT
#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono_multicore/ChSettings.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
using namespace chrono;
%}

#define CH_MULTICORE_API
#define ChApi
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define CH_DEPRECATED(msg)

%ignore CH_ENUM_MAPPER_BEGIN;
%ignore CH_ENUM_VAL;
%ignore CH_ENUM_MAPPER_END;
%ignore CH_CLASS_VERSION;

// Core owns the wrappers and SWIG runtime type information for the common
// Chrono classes used through the ChSystemMulticore inheritance hierarchy.
%import(module = "pychrono.core") "chrono_swig/interface/core/ChClassFactory.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChSystem.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBody.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChContactContainer.i"

%shared_ptr(chrono::ChSystemMulticore)
%shared_ptr(chrono::ChSystemMulticoreNSC)

// These expose internal multicore vector/data types which do not yet have
// public PyChrono typemaps. Keep the useful system-level API below clean.
%ignore chrono::ChSystemMulticore::data_manager;
%ignore chrono::ChSystemMulticore::GetSettings;
%ignore chrono::ChSystemMulticore::GetBodyContactForce;
%ignore chrono::ChSystemMulticore::GetBodyContactTorque;
%ignore chrono::ChSystemMulticore::CalculateConstraintViolation;
// Use the inherited pychrono.core ChSystem wrapper for this enum argument so
// callers pass pychrono.core.ChCollisionSystem.Type without a duplicate type.
%ignore chrono::ChSystemMulticore::SetCollisionSystemType;
%ignore chrono::ChSystemMulticoreNSC::GetBodyContactForce;
%ignore chrono::ChSystemMulticoreNSC::GetBodyContactTorque;
%ignore chrono::ChSystemMulticoreNSC::Add3DOFContainer;
%ignore chrono::ChSystemMulticoreSMC;

// Expose the multicore enums used to select the NSC solver and friction mode.
%include "../../../chrono_multicore/ChMulticoreDefines.h"

// Provide a stable Python API for the settings needed by rigid-body/DEM users
// without exposing Chrono's internal real3/vec3/custom_vector implementation.
%extend chrono::ChSystemMulticore {
    int GetNumThreads() const {
        return $self->current_threads;
    }

    void SetCollisionEnvelope(double value) {
        $self->GetSettings()->collision.collision_envelope = value;
    }

    double GetCollisionEnvelope() {
        return $self->GetSettings()->collision.collision_envelope;
    }

    void SetSolverMaxIterations(unsigned int value) {
        $self->GetSettings()->solver.max_iteration = value;
    }

    unsigned int GetSolverMaxIterations() {
        return $self->GetSettings()->solver.max_iteration;
    }

    void SetSolverMaxIterationsNormal(unsigned int value) {
        $self->GetSettings()->solver.max_iteration_normal = value;
    }

    void SetSolverMaxIterationsSliding(unsigned int value) {
        $self->GetSettings()->solver.max_iteration_sliding = value;
    }

    void SetSolverMaxIterationsSpinning(unsigned int value) {
        $self->GetSettings()->solver.max_iteration_spinning = value;
    }

    void SetSolverMaxIterationsBilateral(unsigned int value) {
        $self->GetSettings()->solver.max_iteration_bilateral = value;
    }

    void SetSolverTolerance(double value) {
        $self->GetSettings()->solver.tolerance = value;
    }

    double GetSolverTolerance() {
        return $self->GetSettings()->solver.tolerance;
    }

    void SetContactRecoverySpeed(double value) {
        $self->GetSettings()->solver.contact_recovery_speed = value;
    }

    double GetContactRecoverySpeed() {
        return $self->GetSettings()->solver.contact_recovery_speed;
    }

    void SetSolverMode(chrono::SolverMode mode) {
        $self->GetSettings()->solver.solver_mode = mode;
    }

    chrono::SolverMode GetSolverMode() {
        return $self->GetSettings()->solver.solver_mode;
    }
}

%include "../../../chrono_multicore/physics/ChSystemMulticore.h"
