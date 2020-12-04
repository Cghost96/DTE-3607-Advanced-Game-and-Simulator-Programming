#include <coldet/solver_development/step2.h>
#include <coldet/bits/scenario_fixtures.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>


// Safely enable namespaces
using namespace dte3607::coldet;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///


struct SolverDevStep2_Fixture001 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();

    // Set external forces
    m_fixture->forces().G = types::Vector3{0, 0, 0};

    // make plane
    auto plane1 = std::make_unique<FixedPlane>(types::Vector3{-1, 0, 0});
    plane1->spaceObjectFrame().translateParent(types::Vector3{10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane1));

    auto plane2 = std::make_unique<FixedPlane>(types::Vector3{1, 0, 0});
    plane2->spaceObjectFrame().translateParent(types::Vector3{-10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane2));

    // make sphere
    auto sphere = std::make_unique<Sphere>(types::Vector3{100, 0, 0}, 1.0);
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 0, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));
  }
  void TearDown() final { m_fixture.release(); }
};


TEST_F(SolverDevStep2_Fixture001, Test001)
{
  solver_dev::step2::solve(*m_fixture, 1s);

  for (auto& sphere : m_fixture->spheres()) {
    EXPECT_TRUE(sphere->point()[0] > -10);
    EXPECT_TRUE(sphere->point()[0] < 10);
  }
}

struct SolverDevStep2_Fixture002 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{0, 0, 0};


    // make plane
    auto plane_yz_p = std::make_unique<FixedPlane>(types::Vector3{-1, 0, 0});
    plane_yz_p->spaceObjectFrame().translateParent(types::Vector3{10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_yz_p));

    auto plane_yz_m = std::make_unique<FixedPlane>(types::Vector3{1, 0, 0});
    plane_yz_m->spaceObjectFrame().translateParent(types::Vector3{-10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_yz_m));

    auto plane_xy_p = std::make_unique<FixedPlane>(types::Vector3{0, 0, -1});
    plane_xy_p->spaceObjectFrame().translateParent(types::Vector3{0, 0, 10});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_xy_p));

    auto plane_xy_m = std::make_unique<FixedPlane>(types::Vector3{0, 0, 1});
    plane_xy_m->spaceObjectFrame().translateParent(types::Vector3{0, 0, -10});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_xy_m));


    // make sphere
    auto sphere = std::make_unique<Sphere>(types::Vector3{100, 0, 100}, 1.0);
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 0, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));
  }
  void TearDown() final { m_fixture.release(); }
};


TEST_F(SolverDevStep2_Fixture002, Test001)
{
  solver_dev::step2::solve(*m_fixture, 1s);

  // Expect to be inbetween the planes
  for( auto const& sphere : m_fixture->spheres() ) {
    for( auto const& plane : m_fixture->fixedPlanes() ) {
      auto const& pn = plane->normal();
      auto const& d = sphere->point() - plane->point();
      auto const pnd = blaze::inner(pn,d) - sphere->radius();
      EXPECT_TRUE(pnd > 0);
    }
  }
}



struct SolverDevStep2_Fixture003 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{0, 0, 0};


    // make plane
    auto plane_yz_p = std::make_unique<FixedPlane>(types::Vector3{-1, 0, 0});
    plane_yz_p->spaceObjectFrame().translateParent(types::Vector3{10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_yz_p));

    auto plane_yz_m = std::make_unique<FixedPlane>(types::Vector3{1, 0, 0});
    plane_yz_m->spaceObjectFrame().translateParent(types::Vector3{-10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_yz_m));

    auto plane_xy_p = std::make_unique<FixedPlane>(types::Vector3{0, 0, -1});
    plane_xy_p->spaceObjectFrame().translateParent(types::Vector3{0, 0, 10});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_xy_p));

    auto plane_xy_m = std::make_unique<FixedPlane>(types::Vector3{0, 0, 1});
    plane_xy_m->spaceObjectFrame().translateParent(types::Vector3{0, 0, -10});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_xy_m));

    auto plane_xz_p = std::make_unique<FixedPlane>(types::Vector3{0, -1, 0});
    plane_xz_p->spaceObjectFrame().translateParent(types::Vector3{0, 10, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_xz_p));

    auto plane_xz_m = std::make_unique<FixedPlane>(types::Vector3{0, 1,   0});
    plane_xz_m->spaceObjectFrame().translateParent(types::Vector3{0, -10, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane_xz_m));


    // Set up spheres
    auto const p = types::Vector3{-2, -2, -2};

    // Randomize zelection of normal
    auto const v_min = types::Vector3{-10, 5, -20};
    auto const v_max = types::Vector3{ 20, 10, 10};

    std::random_device                     r;
    std::default_random_engine             g(r());
    std::uniform_real_distribution<types::ValueType> x_dist(v_min[0], v_max[0]);
    std::uniform_real_distribution<types::ValueType> y_dist(v_min[1], v_max[1]);
    std::uniform_real_distribution<types::ValueType> z_dist(v_min[2], v_max[2]);

    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;
    sphere_data.reserve(3 * 3 * 3);
    for (auto x = 0; x < 3; ++x) {
      for (auto y = 0; y < 3; ++y) {
        for (auto z = 0; z < 3; ++z) {

          auto const v = types::Vector3{x_dist(g), y_dist(g), z_dist(g)};
          sphere_data.emplace_back(
            v, p + types::Vector3{2. * x, 2. * y, 2. * z});
        }
      }
    }

    for (auto const& sd : sphere_data) {
      auto sphere = std::make_unique<Sphere>(std::get<0>(sd), 1.0);
      sphere->spaceObjectFrame().translateParent(std::get<1>(sd));
      m_fixture->spheres().emplace_back(std::move(sphere));
    }
  }
  void TearDown() final { m_fixture.release(); }
};


TEST_F(SolverDevStep2_Fixture003, Test001)
{
  solver_dev::step2::solve(*m_fixture, 1s);

  // Expect to be inbetween the planes
  for( auto const& sphere : m_fixture->spheres() ) {
    for( auto const& plane : m_fixture->fixedPlanes() ) {
      auto const& pn = plane->normal();
      auto const& d = sphere->point() - plane->point();
      auto const pnd = blaze::inner(pn,d) - sphere->radius();
      EXPECT_TRUE(pnd > 0);
    }
  }
}
