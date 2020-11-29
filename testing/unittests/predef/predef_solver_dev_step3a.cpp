#include <coldet/solver_development/step3a.h>
#include <coldet/bits/scenario_fixtures.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///


struct SolverDevStep3a_TestFixture : ::testing::Test {

  using ::testing::Test::Test;
  ~SolverDevStep3a_TestFixture() override {}

  void SetUp() final
  {
//    m_scenario_fixture = ...;
  }
  void TearDown() final {}

  dte3607::coldet::scenario_fixtures::FixtureOsFp m_scenario_fixture;
};



TEST_F(SolverDevStep3a_TestFixture, Test001)
{
  dte3607::coldet::solver_dev::step3a::solve(m_scenario_fixture, 16ms);

  EXPECT_TRUE(false);
}
