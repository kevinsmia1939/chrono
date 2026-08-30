// Correctness and performance comparison for a cylinder bed represented by either
// ChCollisionShapeCylindricalShell with a flat bottom or a triangle mesh.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/collision/ChCollisionShapeCylindricalShell.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

using namespace chrono;

namespace {

constexpr double kBedRadius = 0.006;
constexpr double kBedHeight = 0.030;
constexpr double kParticleRadius = 0.00048;
constexpr double kParticleHeight = 0.00090;
constexpr double kStep = 2e-5;

enum class BedType { ANALYTICAL, MESH };

struct Result {
    double elapsed;
    double final_z_min;
    double final_z_max;
    double max_radius;
    unsigned int contacts;
    bool finite;
    bool contained;
};

std::shared_ptr<ChTriangleMeshConnected> MakeBedMesh(int segments) {
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    const double z0 = -kBedHeight / 2;
    const double z1 = +kBedHeight / 2;
    const ChVector3d bottom_center(0, 0, z0);

    for (int i = 0; i < segments; i++) {
        const double a0 = 2 * CH_PI * i / segments;
        const double a1 = 2 * CH_PI * (i + 1) / segments;
        const ChVector3d p00(kBedRadius * std::cos(a0), kBedRadius * std::sin(a0), z0);
        const ChVector3d p01(kBedRadius * std::cos(a1), kBedRadius * std::sin(a1), z0);
        const ChVector3d p10(kBedRadius * std::cos(a0), kBedRadius * std::sin(a0), z1);
        const ChVector3d p11(kBedRadius * std::cos(a1), kBedRadius * std::sin(a1), z1);

        // Inward-facing side and upward-facing bottom.
        mesh->AddTriangle(p00, p11, p10);
        mesh->AddTriangle(p00, p01, p11);
        mesh->AddTriangle(bottom_center, p01, p00);
    }
    return mesh;
}

std::vector<ChVector3d> ParticlePositions(int count) {
    std::vector<ChVector3d> positions;
    positions.reserve(count);
    const double spacing = 2.25 * kParticleRadius;
    const int across = 7;

    for (int layer = 0; static_cast<int>(positions.size()) < count; layer++) {
        for (int iy = -across; iy <= across && static_cast<int>(positions.size()) < count; iy++) {
            for (int ix = -across; ix <= across && static_cast<int>(positions.size()) < count; ix++) {
                const double x = spacing * ix + ((layer & 1) ? 0.45 * spacing : 0.0);
                const double y = spacing * iy;
                if (std::sqrt(x * x + y * y) + kParticleRadius > 0.88 * kBedRadius)
                    continue;
                const double z = -kBedHeight / 2 + 0.0025 + layer * 1.35 * kParticleHeight;
                positions.emplace_back(x, y, z);
            }
        }
    }
    return positions;
}

Result Run(BedType bed_type, int particle_count, int steps, int threads) {
    ChSystemMulticoreNSC system;
    if (!system.GetContactContainer()) {
        std::cerr << "multicore NSC system has no contact container\n";
        std::abort();
    }
    system.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    system.SetNumThreads(threads);
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    system.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system.GetSettings()->collision.collision_envelope = 2e-6;
    system.GetSettings()->collision.bins_per_axis = vec3(12, 12, 24);
    system.GetSettings()->solver.max_iteration_normal = 50;
    system.GetSettings()->solver.max_iteration_sliding = 50;
    system.GetSettings()->solver.max_iteration_spinning = 10;
    system.GetSettings()->solver.contact_recovery_speed = 0.2;

    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(0.35f);
    material->SetRestitution(0.05f);

    auto bed = chrono_types::make_shared<ChBody>();
    bed->SetFixed(true);
    bed->EnableCollision(true);
    bed->SetPos(ChVector3d(0, 0, 0));
    if (bed_type == BedType::ANALYTICAL) {
        auto shape = chrono_types::make_shared<ChCollisionShapeCylindricalShell>(
            material, kBedRadius, kBedHeight, true, false);
        bed->AddCollisionShape(shape);
    } else {
        auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, MakeBedMesh(96), true, false, 0);
        bed->AddCollisionShape(shape);
    }
    system.AddBody(bed);

    const double density = 1200;
    const double volume = CH_PI * kParticleRadius * kParticleRadius * kParticleHeight;
    const double mass = density * volume;
    const double ixy = mass * (3 * kParticleRadius * kParticleRadius + kParticleHeight * kParticleHeight) / 12;
    const double iz = 0.5 * mass * kParticleRadius * kParticleRadius;

    std::vector<std::shared_ptr<ChBody>> particles;
    for (const auto& position : ParticlePositions(particle_count)) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetMass(mass);
        body->SetInertiaXX(ChVector3d(ixy, ixy, iz));
        body->SetPos(position);
        body->SetRot(QuatFromAngleY(0.12 * std::sin(1000 * position.x())) *
                     QuatFromAngleX(0.08 * std::cos(1000 * position.y())));
        body->EnableCollision(true);
        body->AddCollisionShape(
            chrono_types::make_shared<ChCollisionShapeCylinder>(material, kParticleRadius, kParticleHeight));
        system.AddBody(body);
        particles.push_back(body);
    }

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; i++)
        system.DoStepDynamics(kStep);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    Result result{elapsed, 1e100, -1e100, 0, system.GetNumContacts(), true, true};
    for (const auto& body : particles) {
        const auto p = body->GetPos();
        const auto v = body->GetPosDt();
        result.finite &= std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
                         std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        result.final_z_min = std::min(result.final_z_min, p.z());
        result.final_z_max = std::max(result.final_z_max, p.z());
        result.max_radius = std::max(result.max_radius, std::sqrt(p.x() * p.x() + p.y() * p.y()));
        result.contained &= p.z() >= -kBedHeight / 2 - 2 * kParticleHeight;
        result.contained &= std::sqrt(p.x() * p.x() + p.y() * p.y()) <= kBedRadius + 2 * kParticleRadius;
    }
    return result;
}

void Print(const std::string& label, int particles, int steps, int threads, const Result& result) {
    std::cout << std::fixed << std::setprecision(6) << label << ", particles=" << particles << ", steps=" << steps
              << ", threads=" << threads << ", seconds=" << result.elapsed << ", z_min=" << result.final_z_min
              << ", z_max=" << result.final_z_max << ", max_radius=" << result.max_radius
              << ", contacts=" << result.contacts << ", finite=" << result.finite
              << ", contained=" << result.contained << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    const int particles = argc > 1 ? std::atoi(argv[1]) : 120;
    const int steps = argc > 2 ? std::atoi(argv[2]) : 1500;
    const int threads = argc > 3 ? std::atoi(argv[3]) : 8;

    // Required correctness checks: an actual single falling cylinder and a multiple-particle bed.
    const auto single = Run(BedType::ANALYTICAL, 1, 2500, threads);
    Print("analytical-single", 1, 2500, threads, single);
    if (!single.finite || !single.contained)
        return 2;

    const auto analytical = Run(BedType::ANALYTICAL, particles, steps, threads);
    Print("analytical-multiple", particles, steps, threads, analytical);
    if (!analytical.finite || !analytical.contained)
        return 3;

    const auto mesh = Run(BedType::MESH, particles, steps, threads);
    Print("mesh-multiple", particles, steps, threads, mesh);
    if (!mesh.finite || !mesh.contained)
        return 4;

    std::cout << "mesh_over_analytical=" << mesh.elapsed / analytical.elapsed << '\n';
    return 0;
}
