#ifndef DTE3607_UTILS_SCENARIO_FACTORIES_H
#define DTE3607_UTILS_SCENARIO_FACTORIES_H


#include "../bits/rigidbodies.h"
#include "../bits/scenario_fixtures.h"
#include "../concepts/scenario_fixture_concepts.h"

// stl
#include <concepts>
#include <random>


namespace dte3607::coldet::utils::scenario_factories {

  namespace detail {
    namespace fc_ns = concepts::scenario_fixtures::solver_dev::components;
    // Helpers
    auto const add_sphere
      = [](fc_ns::HasSpheres auto& solution_fixture, std::string const& name,
           types::Vector3 const& init_velocity, types::Vector3 const& translation = {0, 0, 0},
           types::Vector3::ElementType const& radius        = 1.0,
           types::Vector3::ElementType const& friction_coef = 0.0,
           rigidbodies::Sphere::States const& state         = rigidbodies::Sphere::States::Free,
           types::ValueType const&            mass          = 1.0) {
          auto sphere = std::make_unique<rigidbodies::Sphere>(init_velocity, radius, name);
          sphere->setMass(mass);
          sphere->spaceObjectFrame().translateParent(translation);
          sphere->state() = state;
          sphere->setFrictionCoef(friction_coef);
          solution_fixture.spheres().emplace_back(std::move(sphere));
          return solution_fixture.spheres().back().get();
        };

    auto const add_fixed_plane
      = [](fc_ns::HasFixedPlanes auto& solution_fixture, std::string const& name,
           types::Vector3 const& translation = {0, 0, 0}, types::Vector3 const& n = {0, 1, 0},
           types::Vector3::ElementType const& friction_coef = 0.0) {
          auto plane = std::make_unique<rigidbodies::FixedPlane>(n, name);
          plane->spaceObjectFrame().translateParent(translation);
          plane->setFrictionCoef(friction_coef);
          solution_fixture.fixedPlanes().emplace_back(std::move(plane));
          return solution_fixture.fixedPlanes().back().get();
        };
  }   // namespace detail


  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructComponentTestingScenario01() {

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Setup a sphere
    detail::add_sphere(*fixture, "A sphere 1", {0, 0, 0}, {-20, 5, 10}, 5);
    detail::add_sphere(*fixture, "A sphere 2", {0, 0, 0}, {20, 5, 10}, 5);
    detail::add_sphere(*fixture, "A sphere 3", {0, 0, 0}, {20, 5, -10}, 5);
    detail::add_sphere(*fixture, "A sphere 4", {0, 0, 0}, {-20, 5, -10}, 5);

    std::vector<rigidbodies::Sphere*> rot_offset_spheres;
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere 5", {0, 0, 0}, {-20, 15, 10}, 5));
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere 6", {0, 0, 0}, {20, 15, 10}, 5));
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere 7", {0, 0, 0}, {20, 15, -10}, 5));
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere 8", {0, 0, 0}, {-20, 15, -10}, 5));

    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere 9", {0, 0, 0}, {-20, 25, 10}, 5));
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere10", {0, 0, 0}, {20, 25, 10}, 5));
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere11", {0, 0, 0}, {20, 25, -10}, 5));
    rot_offset_spheres.push_back(detail::add_sphere(*fixture, "A sphere12", {0, 0, 0}, {-20, 25, -10}, 5));
    for (auto* sphere : rot_offset_spheres)
      sphere->spaceObjectFrame().rotateParent(std::numbers::pi * .5, {0, 1, 0});




    // Setup a fixed plane
    detail::add_fixed_plane(*fixture, "A plane 1", {40, 10, 25}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "A plane 2", {65, 10, 25}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "A plane 3", {65, 10, -25}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "A plane 4", {40, 10, -25}, {0, 1, 0});

    std::vector<rigidbodies::FixedPlane*> rot_offset_planes;
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane 5", {40, 40, 25}, {0, 1, 0}));
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane 6", {65, 40, 25}, {0, 1, 0}));
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane 7", {65, 40, -25}, {0, 1, 0}));
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane 8", {40, 40, -25}, {0, 1, 0}));

    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane 9", {40, 70, 25}, {0, 1, 0}));
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane10", {65, 70, 25}, {0, 1, 0}));
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane11", {65, 70, -25}, {0, 1, 0}));
    rot_offset_planes.push_back(detail::add_fixed_plane(*fixture, "A plane12", {40, 70, -25}, {0, 1, 0}));
    for (auto* plane : rot_offset_planes)
      plane->spaceObjectFrame().rotateParent(std::numbers::pi * .5, {0, 1, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFb> constructComponentTestingScenario02() {

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFb>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};


    detail::add_sphere(*fixture, "A sphere 1", {0, 0, 0}, {0, 20, 10}, 2);

    auto  bez  = std::make_unique<rigidbodies::FixedBezierSurface>("A first bezier surface");
    auto& bezs = bez->bezierSurface();

    using BzSPoint = typename std::decay_t<decltype(bezs)>::Point;

    bezs.m_C.resize(3, 3);
    bezs.m_C(0, 0) = BzSPoint{-10, 0, -10};
    bezs.m_C(1, 0) = BzSPoint{0, 0, -10};
    bezs.m_C(2, 0) = BzSPoint{10, 0, -10};
    bezs.m_C(0, 1) = BzSPoint{-10, 0, 0};
    bezs.m_C(1, 1) = BzSPoint{0, 65, 0};
    bezs.m_C(2, 1) = BzSPoint{10, 0, 0};
    bezs.m_C(0, 2) = BzSPoint{-10, 0, 10};
    bezs.m_C(1, 2) = BzSPoint{0, 0, 10};
    bezs.m_C(2, 2) = BzSPoint{10, 0, 10};

    bez->spaceObjectFrame().translateParent({0, 0, 0});
    bez->setFrictionCoef(0.0);
    fixture->fixedBezierSurfaces().emplace_back(std::move(bez));

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFlp> constructComponentTestingScenario03() {

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFlp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    detail::add_sphere(*fixture, "A sphere 1", {0, 0, 0}, {0, 20, 10}, 2);

    // Setup fixed limited planes
    auto plane = std::make_unique<rigidbodies::FixedLimitedPlane>(
      types::Point3{-15, 0, -15}, types::Vector3{30, 0, 0}, types::Vector3{0, 0, 30}, 1.0, "limited plane");
    plane->spaceObjectFrame().translateParent({0, 15, 0});
    plane->setFrictionCoef(0.0);
    fixture->fixedLimitedPlanes().emplace_back(std::move(plane));

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOs> constructOriginalStep0TestScenario() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOs>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};


    // Setup objects
    detail::add_sphere(*fixture, "uno", {5, 0, 0}, {0, 10, 0});
    detail::add_sphere(*fixture, "dos", {-5, 0, 0}, {0, 10, 0});
    detail::add_sphere(*fixture, "tres", {0, 5, 0}, {0, 10, 0});

    for (auto i = 0ul; i < 10; ++i) {
      detail::add_sphere(*fixture, std::string("filler ") + std::to_string(i), {0, -0.1, 0}, {0, 5, 0});
    }

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructOriginalStep1TestScenario() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Setup spheres
    auto const bmi = types::Vector3{-5, 15, -5};
    auto const bma = types::Vector3{5, 25, 5};

    auto const& [no_x, no_y, no_z] = std::tuple(2, 2, 2);
    //    auto const& [no_x, no_y, no_z] = std::tuple(10,10,10);
    //    auto const& [no_x, no_y, no_z] = std::tuple(20,20,20);
    //    auto const& [no_x, no_y, no_z] = std::tuple(50,50,50);

    auto       rf = types::Vector3::ElementType(0.9);   // Radius factor
    auto const dx = (bma[0] - bmi[0]) / types::Vector3::ElementType(no_x * rf);
    auto const dy = (bma[1] - bmi[1]) / types::Vector3::ElementType(no_y * rf);
    auto const dz = (bma[2] - bmi[2]) / types::Vector3::ElementType(no_z * rf);

    for (auto ix = 0; ix < no_x; ++ix) {
      for (auto iy = 0; iy < no_y; ++iy) {
        for (auto iz = 0; iz < no_z; ++iz) {
          auto const st = bmi;
          auto const d  = types::Vector3{ix * dx, iy * dy, iz * dz};
          auto const v  = types::Vector3{0, 0, 0};
          auto const r  = std::min(std::min(dx, dy), dz) / 4.0;
          auto const tr = st + d;

          detail::add_sphere(*fixture,
                             "Sphere " + std::to_string(ix) + ' ' + std::to_string(iy) + ' '
                               + std::to_string(iz) + ' ',
                             v, tr, r);
        }
      }
    }

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_uno", {0, -5, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructOriginalStep2TestScenario() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Setup spheres
    auto const bmi = types::Vector3{-5, 10, -5};
    auto const bma = types::Vector3{5, 15, 5};

    //    auto const& [no_x, no_y, no_z] = std::tuple(1,1,1);
    //    auto const& [no_x, no_y, no_z] = std::tuple(2,2,2);
    auto const& [no_x, no_y, no_z] = std::tuple(5, 5, 5);
    //    auto const& [no_x, no_y, no_z] = std::tuple(10, 10, 10);
    //    auto const& [no_x, no_y, no_z] = std::tuple(15,15,15);
    //    auto const& [no_x, no_y, no_z] = std::tuple(20,20,20);
    //    auto const& [no_x, no_y, no_z] = std::tuple(50,50,50);

    auto       rf = types::Vector3::ElementType(0.9);   // Radius factor
    auto const dx = (bma[0] - bmi[0]) / types::Vector3::ElementType(no_x * rf);
    auto const dy = (bma[1] - bmi[1]) / types::Vector3::ElementType(no_y * rf);
    auto const dz = (bma[2] - bmi[2]) / types::Vector3::ElementType(no_z * rf);

    for (auto ix = 0; ix < no_x; ++ix) {
      for (auto iy = 0; iy < no_y; ++iy) {
        for (auto iz = 0; iz < no_z; ++iz) {
          auto const st = bmi;
          auto const d  = types::Vector3{ix * dx, iy * dy, iz * dz};

          auto const v = types::Vector3{0, 50, 0};
          //          auto const v  = types::Vector3{0, 10000, 0};
          auto const r  = std::min(std::min(dx, dy), dz) / 2.0;
          auto const tr = st + d;

          detail::add_sphere(*fixture,
                             "Sphere " + std::to_string(ix) + ' ' + std::to_string(iy) + ' '
                               + std::to_string(iz) + ' ',
                             v, tr, r);
        }
      }
    }

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {0, 1, 0}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "plane_uno", {0, 25, 0}, {0, -1, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructOriginalStep3aTestScenario() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const v1 = types::Vector3{1, 0, 0};
    auto const v2 = types::Vector3{-1, 0, 0};
    auto const p1 = types::Vector3{-1, 10, 0};
    auto const p2 = types::Vector3{1, 10, 0};

    detail::add_sphere(*fixture, "S1", v1, p1, 0.5);
    detail::add_sphere(*fixture, "S2", v2, p2, 0.5);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {0, 0, 0}, {0, 1, 0});
    //    detail::add_fixed_plane(*fixture, "plane_uno", {0, 25, 0}, {0, -1,
    //    0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOs> constructScenario001() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOs>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};


    // Setup objects
    detail::add_sphere(*fixture, "uno", {10, 5, 0}, {-10, 5, -15});
    detail::add_sphere(*fixture, "dos", {10, 10, 0}, {-10, 5, 0});
    detail::add_sphere(*fixture, "tres", {10, 15, 0}, {-10, 5, 15});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario002() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};


    // Setup objects
    detail::add_sphere(*fixture, "uno", {10, 5, 0}, {-10, 5, -15});
    detail::add_sphere(*fixture, "dos", {10, 10, 0}, {-10, 5, 0});
    detail::add_sphere(*fixture, "tres", {10, 15, 0}, {-10, 5, 15});

    detail::add_fixed_plane(*fixture, "plane_dos", {0, 1, 0}, {0, 1, 0});
    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario003() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, 0, 0};

    // Set up spheres
    auto const p1 = types::Vector3{-10, 25, -10};
    auto const v1 = types::Vector3{10, 0, 0};

    auto const p2 = types::Vector3{-10, 15, 0};
    auto const v2 = types::Vector3{100, 0, 0};

    auto const p3 = types::Vector3{-10, 5, 10};
    auto const v3 = types::Vector3{10000, 0, 0};

    detail::add_sphere(*fixture, "S1", v1, p1, 1);
    detail::add_sphere(*fixture, "S2", v2, p2, 1);
    detail::add_sphere(*fixture, "S3", v3, p3, 1);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {-15, 15, 0}, {1, 0, 0});
    detail::add_fixed_plane(*fixture, "plane_uno", {15, 15, 0}, {-1, 0, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario004() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p1 = types::Vector3{-10, 5, 0};
    auto const v1 = types::Vector3{10, 10, 0};

    auto const p2 = types::Vector3{10, 5, 0};
    auto const v2 = types::Vector3{-10, 10, 0};

    detail::add_sphere(*fixture, "S1", v1, p1, 1.0);
    detail::add_sphere(*fixture, "S2", v2, p2, 1.0);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {0, 0, 0}, {0, 1, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario005() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p1 = types::Vector3{-10, 5, -2};
    auto const v1 = types::Vector3{10, 10, 0};

    auto const p2 = types::Vector3{10, 5, 2};
    auto const v2 = types::Vector3{-10, 10, 0};

    detail::add_sphere(*fixture, "S1", v1, p1, 1.0);
    detail::add_sphere(*fixture, "S2", v2, p2, 5.0);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {0, 0, 0}, {0, 1, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario006() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p1 = types::Vector3{-10, 15, 0};
    auto const v1 = types::Vector3{100, 0, 0};

    auto const p2 = types::Vector3{10, 15, 0};
    auto const v2 = types::Vector3{-100, 0, 0};

    detail::add_sphere(*fixture, "S1", v1, p1, 3);
    detail::add_sphere(*fixture, "S2", v2, p2, 3);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {-15, 15, 0}, {1, 0, 0});
    detail::add_fixed_plane(*fixture, "plane_uno", {15, 15, 0}, {-1, 0, 0});
    detail::add_fixed_plane(*fixture, "plane_tres", {0, 0, 0}, {0, 1, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario007() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p = types::Vector3{-10, 5, -10};
    auto const v = types::Vector3{0, 0, 0};

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x)
      for (auto y = 0; y < 3; ++y)
        for (auto z = 0; z < 3; ++z)
          sphere_data.emplace_back(v, p + types::Vector3{10. * x, 10. * y, 10. * z});

    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), 3);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario008() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p = types::Vector3{-9, 10, -8};
    auto const v = types::Vector3{0, 0, 0};

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x)
      for (auto y = 0; y < 2; ++y)
        for (auto z = 0; z < 3; ++z)
          sphere_data.emplace_back(v, p + types::Vector3{10. * x, 20. * y, 10. * z});

    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), 2);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "plane_uno", {-10, 5, 10}, {1, 1, -1});
    detail::add_fixed_plane(*fixture, "plane_dos", {-10, 5, -10}, {1, 1, 1});
    detail::add_fixed_plane(*fixture, "plane_tres", {10, 5, 10}, {-1, 1, -1});
    detail::add_fixed_plane(*fixture, "plane_cuatro", {10, 5, -10}, {-1, 1, 1});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario009() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{0, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {-10, 10, -10}, 1});
    sphere_data.push_back({v0, {0, 10, -10}, 2});
    sphere_data.push_back({v0, {10, 10, -10}, 3});
    sphere_data.push_back({v0, {-10, 10, 0}, 1});
    sphere_data.push_back({v0, {0, 10, 0}, 2});
    sphere_data.push_back({v0, {10, 10, 0}, 3});
    sphere_data.push_back({v0, {-10, 10, 10}, 1});
    sphere_data.push_back({v0, {0, 10, 10}, 2});
    sphere_data.push_back({v0, {10, 10, 10}, 3});
    // ADD INITIAL STATES AND FRICTION AND WEIGHT


    // add spheres
    for (auto const& sd : sphere_data)
      //      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd));
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0.5);

    // Setup fixed planes
    //    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0}, 0.5);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario010() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    //    fixture->forces().G = types::Vector3{0, -9.833, 0};
    fixture->forces().G = types::Vector3{0, -30.0, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({{1, 1, 0}, {-10, 3.001, -10}, 3});
    //    sphere_data.push_back({{1, 0.1, 0}, {-10, 3.001, -10}, 3});
    sphere_data.push_back({{1, 0, 0}, {-10, 3.001, 0}, 3});
    //    sphere_data.push_back({{1, 0, 0}, {-10, 3.001, 0}, 3});
    sphere_data.push_back({{1, -1, 0}, {-10, 3.001, 10}, 3});
    //    sphere_data.push_back({{1, -0.1, 0}, {-10, 3.001, 10}, 3});
    // ADD INITIAL STATES AND FRICTION AND WEIGHT


    // add spheres
    //    for (auto const& sd : sphere_data)
    //      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd));
    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0.6);

    // Setup fixed planes
    //    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0}, 0.6);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario011() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    //    fixture->forces().G = types::Vector3{0, -30, 0};
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    //    auto const p1 = types::Vector3{-10, 3, 0};
    auto const p1 = types::Vector3{-10, 16, 0};
    //    auto const p1 = types::Vector3{-10, 3.001, 0};
    //    auto const p1 = types::Vector3{-75, 0.501, 0};
    //    auto const v1 = types::Vector3{150, 0, 0};
    auto const v1 = types::Vector3{200, 0, 0};

    //    auto const p2 = types::Vector3{10, 3, 0};
    auto const p2 = types::Vector3{10, 16, 0};
    //    auto const p2 = types::Vector3{5, 3.001, 0};
    auto const v2 = types::Vector3{-200, 0, 0};
    //    auto const v2 = types::Vector3{0, 0, 0};

    //    detail::add_sphere(*fixture, "S1", v1, p1, 3);
    detail::add_sphere(*fixture, "S1", v1, p1, 3, 0.6);
    detail::add_sphere(*fixture, "S2", v2, p2, 3, 0.6);
    // ADD INITIAL STATES AND FRICTION AND WEIGHT

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_dos", {-15, 15, 0}, {1, 0, 0}, 0.6);
    detail::add_fixed_plane(*fixture, "plane_uno", {15, 15, 0}, {-1, 0, 0}, 0.6);
    detail::add_fixed_plane(*fixture, "plane_tres", {0, 0, 0}, {0, 1, 0}, 0.6);



    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFp> constructScenario012() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFp>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p = types::Vector3{-10, 5, -10};

    // Randomize zelection of normal
    auto const v_min = types::Vector3{-10, 5, -20};
    auto const v_max = types::Vector3{20, 10, 10};

    std::random_device                               r;
    std::default_random_engine                       g(r());
    std::uniform_real_distribution<types::ValueType> x_dist(v_min[0], v_max[0]);
    std::uniform_real_distribution<types::ValueType> y_dist(v_min[1], v_max[1]);
    std::uniform_real_distribution<types::ValueType> z_dist(v_min[2], v_max[2]);

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x) {
      for (auto y = 0; y < 3; ++y) {
        for (auto z = 0; z < 3; ++z) {

          auto const v = types::Vector3{x_dist(g), y_dist(g), z_dist(g)};
          sphere_data.emplace_back(v, p + types::Vector3{10. * x, 10. * y, 10. * z});
        }
      }
    }

    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), 1);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});
    detail::add_fixed_plane(*fixture, "plane_top", {0, 30, 0}, {0, -1, 0});

    detail::add_fixed_plane(*fixture, "plane_back", {0, 15, -30}, {0, 0, 1});
    detail::add_fixed_plane(*fixture, "plane_front", {0, 15, 30}, {0, 0, -1});

    detail::add_fixed_plane(*fixture, "plane_left", {-30, 15, 0}, {1, 0, 0});
    detail::add_fixed_plane(*fixture, "plane_right", {30, 15, 0}, {-1, 0, 0});

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario013() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{0, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {-10, 15, -10}, 1});
    sphere_data.push_back({v0, {0, 15, -10}, 2});
    sphere_data.push_back({v0, {10, 15, -10}, 3});
    sphere_data.push_back({v0, {-10, 15, 0}, 1});
    sphere_data.push_back({v0, {0, 15, 0}, 2});
    sphere_data.push_back({v0, {10, 15, 0}, 3});
    sphere_data.push_back({v0, {-10, 15, 10}, 1});
    sphere_data.push_back({v0, {0, 15, 10}, 2});
    sphere_data.push_back({v0, {10, 15, 10}, 3});
    // ADD INITIAL STATES AND FRICTION AND WEIGHT


    // add spheres
    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0.5);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {1, 1, 0}, 0.5);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario014() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{0, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {-10, 30, 10}, 1});
    sphere_data.push_back({v0, {-10, 30, 0}, 2});
    sphere_data.push_back({v0, {-10, 30, -10}, 3});
    // ADD INITIAL STATES AND FRICTION AND WEIGHT


    // add spheres
    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0.5);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_inclined", {0, 12, 0}, {1, 1, 0}, 0.5);

    detail::add_fixed_plane(*fixture, "plane_ground", {15, 0, 0}, {0, 1, 0}, 0.5);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario015() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{10, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {0, 1, 10}, 1});
    sphere_data.push_back({v0, {0, 2, 0}, 2});
    sphere_data.push_back({v0, {0, 3, -10}, 3});

    // Ground plane the sphere initially slides on
    auto* gp = detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});

    // add spheres
    for (auto const& sd : sphere_data) {
      auto* sp = detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0,
                                    rigidbodies::Sphere::States::Sliding);

      // default attachment
      fixture->sfpAttachments()[sp] = gp;
    }


    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane R", {20, 10, 0}, {-1, 1, 0});

    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane L", {-20, 10, 0}, {1, 1, 0});


    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario016() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{10, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {0, 1, 10}, 1});
    sphere_data.push_back({v0, {0, 2, 0}, 2});
    sphere_data.push_back({v0, {0, 3, -10}, 3});

    // Ground plane the sphere initially slides on
    auto* gp = detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0}, 0.1);

    // add spheres
    for (auto const& sd : sphere_data) {
      auto* sp = detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0.1,
                                    rigidbodies::Sphere::States::Sliding);

      // default attachment
      fixture->sfpAttachments()[sp] = gp;
    }


    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane R", {20, 10, 0}, {-1, 1, 0}, 0.2);

    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane L", {-20, 10, 0}, {1, 1, 0}, 0.5);


    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario017() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{10, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {0, 1, 10}, 1});
    sphere_data.push_back({v0, {0, 2, 0}, 2});
    sphere_data.push_back({v0, {0, 3, -10}, 3});

    // Ground plane the sphere initially slides on
    auto* gp = detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});

    // add spheres
    for (auto const& sd : sphere_data) {
      auto* sp = detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0,
                                    rigidbodies::Sphere::States::Sliding);

      // default attachment
      fixture->sfpAttachments()[sp] = gp;
    }


    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane R", {20, 10, 0}, {-1, -1, 0});


    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario018() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{9.833, 0, 0};

    // Set up spheres
    // -- velocity - translation - radius --
    auto const                                                                v0 = types::Vector3{30, 0, 0};
    std::vector<std::tuple<types::Vector3, types::Vector3, types::ValueType>> sphere_data;
    sphere_data.push_back({v0, {0, 1, 10}, 1});
    sphere_data.push_back({v0, {0, 2, 0}, 2});
    sphere_data.push_back({v0, {0, 3, -10}, 3});

    // Ground plane the sphere initially slides on
    auto* gp = detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0});

    // add spheres
    for (auto const& sd : sphere_data) {
      auto* sp = detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), std::get<2>(sd), 0,
                                    rigidbodies::Sphere::States::Sliding);

      // default attachment
      fixture->sfpAttachments()[sp] = gp;
    }


    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane R", {20, 10, 0}, {-1, 1, 0});

    // Inclined plane to bounce on
    detail::add_fixed_plane(*fixture, "bounce plane R2", {20, 20, 0}, {-1, -1, 0});


    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa> constructScenario019() {
    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Set up spheres
    auto const p = types::Vector3{-10, 5, -10};

    // Randomize zelection of normal
    auto const v_min = types::Vector3{-10, 5, -20};
    auto const v_max = types::Vector3{20, 10, 10};

    std::random_device                               r;
    std::default_random_engine                       g(r());
    std::uniform_real_distribution<types::ValueType> x_dist(v_min[0], v_max[0]);
    std::uniform_real_distribution<types::ValueType> y_dist(v_min[1], v_max[1]);
    std::uniform_real_distribution<types::ValueType> z_dist(v_min[2], v_max[2]);

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x) {
      for (auto y = 0; y < 3; ++y) {
        for (auto z = 0; z < 3; ++z) {

          auto const v = types::Vector3{x_dist(g), y_dist(g), z_dist(g)};
          sphere_data.emplace_back(v, p + types::Vector3{10. * x, 10. * y, 10. * z});
        }
      }
    }

    for (auto const& sd : sphere_data)
      detail::add_sphere(*fixture, "Sx", std::get<0>(sd), std::get<1>(sd), 2, 0.1);

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "plane_ground", {0, 0, 0}, {0, 1, 0}, 0.5);
    detail::add_fixed_plane(*fixture, "plane_top", {0, 30, 0}, {0, -1, 0}, 0.5);

    detail::add_fixed_plane(*fixture, "plane_back", {0, 15, -30}, {0, 0, 1}, 0.5);
    detail::add_fixed_plane(*fixture, "plane_front", {0, 15, 30}, {0, 0, -1}, 0.5);

    detail::add_fixed_plane(*fixture, "plane_left", {-30, 15, 0}, {1, 0, 0}, 0.5);
    detail::add_fixed_plane(*fixture, "plane_right", {30, 15, 0}, {-1, 0, 0}, 0.5);

    return fixture;
  }

  /* --------------------------------------------------------------------------------- */
  /* ------------------------------- PROJECT SCENARIOS ------------------------------- */
  /* --------------------------------------------------------------------------------- */

  // ----- Scenario template -----
  //  //  inline std::unique_ptr<scenario_fixtures::/*fixture type*/> construct/*function name*/() {
  //  // add_sphere(fixture, name, v0, p0, r, µ, state, m);
  //  // add_fixed_plane(fixture, name, p0, n, µ);

  //  // Construct scenario
  //  auto fixture = std::make_unique<scenario_fixtures::/*fixture type*/>();

  //  // Setup global system
  //  fixture->forces().G = types::Vector3{0, -9.833, 0};

  //  // Objects
  //  detail::add_sphere(*fixture, "sphere", {0, 0, 0}, {0, 10, 0}, 3.0, 0.5,
  //  rigidbodies::Sphere::States::Free,
  //                     1.0);
  //  detail::add_fixed_plane(*fixture, "ground", {0, 0, 0}, {0, 1, 0}, 0.5);

  //  return fixture;
  //  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonVerticalFallFlatGround01() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Objects
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {0, 10, 0}, 3.0, 0.5, rigidbodies::Sphere::States::Free,
                       1.0);
    detail::add_fixed_plane(*fixture, "Floor", {0, 0, 0}, {0, 1, 0}, 0.5);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonRollSlopedGround01() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.81, 0};

    // Objects
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {-16, 13, 0}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_fixed_plane(*fixture, "Floor", {0, 0, 0}, {0.5, 1, 0}, 0.45);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonRollSlopedGround02() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.81, 0};

    // Objects
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {16, 13, 0}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_fixed_plane(*fixture, "Floor", {0, 0, 0}, {-0.5, 1, 0}, 0.45);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonRollSlopedGround03() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.81, 0};

    // Objects
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {0, 13, -16}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_fixed_plane(*fixture, "Floor", {0, 0, 0}, {0, 1, 0.5}, 0.45);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonRollSlopedGround04() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.81, 0};

    // Objects
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {0, 13, 16}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_fixed_plane(*fixture, "Floor", {0, 0, 0}, {0, 1, -0.5}, 0.45);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonRollSlopedGround05() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.81, 0};

    // Objects
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {-46, 13, 0}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {46, 13, 0}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {0, 13, -46}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);
    detail::add_sphere(*fixture, "S", {0, 0, 0}, {0, 13, 46}, 1.5, 0.45, rigidbodies::Sphere::States::Free,
                       200.0);

    detail::add_fixed_plane(*fixture, "Funnel +x", {-30, 0, 0}, {0.5, 1, 0}, 0.6);
    detail::add_fixed_plane(*fixture, "Funnel -x", {30, 0, 0}, {-0.5, 1, 0}, 0.6);
    detail::add_fixed_plane(*fixture, "Funnel +z", {0, 0, -30}, {0, 1, 0.5}, 0.6);
    detail::add_fixed_plane(*fixture, "Funnel -z", {0, 0, 30}, {0, 1, -0.5}, 0.6);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds>
  constructGaltonRollingCollisionFlatGround01() {
    // add_sphere(fixture, name, v0, p0, r, µ, state, m);
    // add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -15, 0};

    // Set up spheres
    auto const p1 = types::Vector3{-75, 3.001, 0};
    auto const v1 = types::Vector3{300, 0, 0};

    auto const p2 = types::Vector3{5, 3.001, 0};
    auto const v2 = types::Vector3{0, 0, 0};

    detail::add_sphere(*fixture, "S1", v1, p1, 3, 0.5, rigidbodies::Sphere::States::Free, 0.8);
    detail::add_sphere(*fixture, "S2", v2, p2, 3, 0.5, rigidbodies::Sphere::States::Free, 1.0);
    // ADD INITIAL STATES AND FRICTION AND WEIGHT

    // Setup fixed planes
    detail::add_fixed_plane(*fixture, "floor", {0, 0, 0}, {0, 1, 0}, 0.5);

    return fixture;
  }

  inline std::unique_ptr<scenario_fixtures::FixtureOsFpPsfp_sa_ds> constructGaltonRollingCollisionAir01() {
    //     add_sphere(fixture, name, v0, p0, r, µ, state, m);
    //     add_fixed_plane(fixture, name, p0, n, µ);

    // Construct scenario
    auto fixture = std::make_unique<scenario_fixtures::FixtureOsFpPsfp_sa_ds>();

    // Setup global system
    fixture->forces().G = types::Vector3{0, -9.833, 0};

    // Objects
    detail::add_sphere(*fixture, "S1", {50, 3, 50}, {4, 10, 4}, 3.0, 0.6, rigidbodies::Sphere::States::Free,
                       1.5);
    detail::add_sphere(*fixture, "S2", {-50, 3, -50}, {13.5, 10, 21.5}, 3.0, 0.6,
                       rigidbodies::Sphere::States::Free, 1.5);
    detail::add_fixed_plane(*fixture, "ground", {0, 0, 0}, {0, 1, 0}, 0.5);

    return fixture;
  }

}   // namespace dte3607::coldet::utils::scenario_factories



#endif   // DTE3607_UTILS_SCENARIO_FACTORIES_H
