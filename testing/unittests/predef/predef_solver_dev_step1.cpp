#include <coldet/solver_development/step1.h>
#include <coldet/bits/scenario_fixtures.h>
#include <coldet/bits/rigidbodies.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>
#include <memory>


// Safely enable namespaces
using namespace dte3607::coldet;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///


struct SolverDevStep1_Fixture001 : ::testing::Test {

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
    auto plane = std::make_unique<FixedPlane>(types::Vector3{-1, 0, 0});
    plane->spaceObjectFrame().translateParent(types::Vector3{10, 0, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane));

    // make sphere
    auto sphere = std::make_unique<Sphere>(types::Vector3{100, 0, 0}, 1.0);
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 0, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));
  }
  void TearDown() final { m_fixture.release(); }
};



TEST_F(SolverDevStep1_Fixture001, Test001)
{
  solver_dev::step1::solve(*m_fixture, 1s);

  for (auto& sphere : m_fixture->spheres())
    EXPECT_TRUE(sphere->point()[0] < 10);
}





struct SolverDevStep1_Fixture002 : ::testing::Test {

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

    // make sphere
    auto sphere = std::make_unique<Sphere>(types::Vector3{100, 0, 0}, 1.0);
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 0, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));
  }
  void TearDown() final { m_fixture.release(); }
};


TEST_F(SolverDevStep1_Fixture002, Test001)
{
  solver_dev::step1::solve(*m_fixture, 1s);

  // Expect to be inbetween the planes
  for( auto const& sphere : m_fixture->spheres() ) {
    for( auto const& plane : m_fixture->fixedPlanes() ) {
      auto const pn = blaze::evaluate(plane->normal());
      auto const sp = blaze::evaluate(sphere->point());
      auto const sr = sphere->radius();
      auto const pp = blaze::evaluate(plane->point());
      auto const d  = blaze::evaluate(sp - (pp + blaze::normalize(pn)*sr));
      EXPECT_GE(blaze::inner(pn,d),0);
    }
  }
//  EXPECT_NEAR(m_fixture->spheres()[0]->point()[0], -0.8, 1e-7);
}



