#include <coldet/solver_development/step3b.h>
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


struct SolverDevStep3b_Fixture001 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;
  Sphere*                      m_the_sphere;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{0, -9.82, 0};

    // make plane
    auto plane = std::make_unique<FixedPlane>(types::Vector3{0, 1, 0});
    plane->setFrictionCoef(FixedPlane::frictionCoefMax);
    m_fixture->fixedPlanes().emplace_back(std::move(plane));

    // make sphere
    auto sphere     = std::make_unique<Sphere>(types::Vector3{0, 0, 0}, 1.0);
    m_the_sphere    = sphere.get();
    sphere->state() = Sphere::States::Free;
    sphere->setFrictionCoef(Sphere::frictionCoefMax);
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 1.001, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));
  }
  void TearDown() final { m_fixture.release(); }
};

TEST_F(SolverDevStep3b_Fixture001, Test001)
{
  solver_dev::step3b::solve(*m_fixture, 10s);
  EXPECT_TRUE(m_the_sphere->state() == Sphere::States::Resting);
}



struct SolverDevStep3b_Fixture002 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;
  Sphere*                      m_the_sphere;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{10, -9.82, 0};

    // make plane
    auto plane = std::make_unique<FixedPlane>(types::Vector3{0, 1, 0});
    plane->setFrictionCoef(FixedPlane::frictionCoefMax);
    m_fixture->fixedPlanes().emplace_back(std::move(plane));

    // make sphere
    auto sphere     = std::make_unique<Sphere>(types::Vector3{0, 0, 0}, 1.0);
    m_the_sphere    = sphere.get();
    sphere->state() = Sphere::States::Free;
    sphere->setFrictionCoef(Sphere::frictionCoefMax);
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 1.001, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));
  }
  void TearDown() final { m_fixture.release(); }
};



TEST_F(SolverDevStep3b_Fixture002, Test001)
{
  solver_dev::step3b::solve(*m_fixture, 10s);
  EXPECT_TRUE(m_the_sphere->state() == Sphere::States::Sliding);
}





struct SolverDevStep3b_Fixture003 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;
  Sphere*                      m_the_sphere;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{0, -9.82, 0};

    // make plane
    auto plane = std::make_unique<FixedPlane>(types::Vector3{0, 1, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane));

    // make sphere
    auto sphere     = std::make_unique<Sphere>(types::Vector3{0, 0, 0}, 1.0);
    m_the_sphere    = sphere.get();
    sphere->state() = Sphere::States::Resting;
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 1.000, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));

    // make sphere 2
    auto sphere2     = std::make_unique<Sphere>(types::Vector3{0, -5, 0}, 0.2);
    sphere2->state() = Sphere::States::Free;
    sphere2->setMass(10.0);
    sphere2->setFrictionCoef(Sphere::frictionCoefMax);
    sphere2->spaceObjectFrame().translateParent(types::Vector3{-.5, 2.000, 0});
    m_fixture->spheres().emplace_back(std::move(sphere2));
  }
  void TearDown() final { m_fixture.release(); }
};



TEST_F(SolverDevStep3b_Fixture003, Test001)
{
  solver_dev::step3b::solve(*m_fixture, 10s);
  EXPECT_TRUE(m_the_sphere->state() == Sphere::States::Free);
}



struct SolverDevStep3b_Fixture004 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;
  Sphere*                      m_the_sphere;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{0, 0, 0};

    // make plane
    auto plane = std::make_unique<FixedPlane>(types::Vector3{0, 1, 0});
    m_fixture->fixedPlanes().emplace_back(std::move(plane));

    // make sphere
    auto sphere     = std::make_unique<Sphere>(types::Vector3{0, 0, 0}, 1.0);
    m_the_sphere    = sphere.get();
    sphere->state() = Sphere::States::Resting;
    sphere->spaceObjectFrame().translateParent(types::Vector3{0, 1.000, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));

    // make sphere 2
    auto sphere2     = std::make_unique<Sphere>(types::Vector3{5, 0, 0}, 0.2);
    sphere2->state() = Sphere::States::Free;
    sphere2->setMass(10.0);
    sphere2->setFrictionCoef(Sphere::frictionCoefMax);
    sphere2->spaceObjectFrame().translateParent(types::Vector3{-1.5, .5, 0});
    m_fixture->spheres().emplace_back(std::move(sphere2));
  }
  void TearDown() final { m_fixture.release(); }
};



TEST_F(SolverDevStep3b_Fixture004, Test001)
{
  solver_dev::step3b::solve(*m_fixture, 2s);
  EXPECT_TRUE(m_the_sphere->state() == Sphere::States::Free);
}








struct SolverDevStep3b_Fixture005 : ::testing::Test {

  using FixedPlane = rigidbodies::FixedPlane;
  using Sphere     = rigidbodies::Sphere;

  using TestFixture = scenario_fixtures::FixtureOsFp;
  std::unique_ptr<TestFixture> m_fixture;
  Sphere*                      m_the_sphere;


  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture = std::make_unique<TestFixture>();


    // Set external forces
    m_fixture->forces().G = types::Vector3{0, 0, 0};

    // make plane
    auto plane = std::make_unique<FixedPlane>(types::Vector3{0, 1, 0});
    plane->setFrictionCoef(FixedPlane::frictionCoefMax);
    m_fixture->fixedPlanes().emplace_back(std::move(plane));

    // make sphere
    auto sphere     = std::make_unique<Sphere>(types::Vector3{10, 0, 0}, 1);
    m_the_sphere    = sphere.get();
    sphere->state() = Sphere::States::Sliding;
    sphere->spaceObjectFrame().translateParent(types::Vector3{-5, 1, 0});
    m_fixture->spheres().emplace_back(std::move(sphere));

    // make sphere 2
    auto sphere2     = std::make_unique<Sphere>(types::Vector3{0, 0, 0}, .2);
    sphere2->state() = Sphere::States::Resting;
    sphere2->setMass(10000.0);
    sphere2->setFrictionCoef(Sphere::frictionCoefMax);
    sphere2->spaceObjectFrame().translateParent(types::Vector3{0, .2, 0});
    m_fixture->spheres().emplace_back(std::move(sphere2));
  }
  void TearDown() final { m_fixture.release(); }
};



TEST_F(SolverDevStep3b_Fixture005, Test001)
{
  solver_dev::step3b::solve(*m_fixture, 2s);
  EXPECT_TRUE(m_the_sphere->state() == Sphere::States::Free);
}

