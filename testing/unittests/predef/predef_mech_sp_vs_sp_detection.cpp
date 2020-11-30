#include <coldet/mechanics/sphere_vs_sphere_detection.h>
#include <coldet/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <chrono>
using namespace std::chrono_literals;

/////////////////
/// \brief TEST
///
///

using namespace dte3607::coldet;

TEST(Mechanics_Sphere_vs_Sphere_Detection, Basic_SpheresAtT0)
{
  types::HighResolutionTP const& t_0       = types::HighResolutionClock::now();
  types::HighResolutionTP const& sphere_tc = t_0;
  types::HighResolutionTP const& other_tc  = t_0;

  // Sphere
  types::Point3 const    sphere_p{-5.0, 0.0, 0.0};
  types::ValueType const sphere_r{1.0};
  types::Point3 const    sphere_v{10.0, 0.0, 0.0};

  // Other
  types::Point3 const    other_p{5.0, 0.0, 0.0};
  types::ValueType const other_r{1.0};
  types::Point3 const    other_v{-10.0, 0.0, 0.0};

  // No gravity
  types::Vector3 const external_forces{0, 0, 0};

  // Sim 0ms
  auto const res_0ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 0ms);

  EXPECT_FALSE(res_0ms);

  // Sim 200ms
  auto const res_200ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 200ms);

  EXPECT_FALSE(res_200ms);

  // Sim 399ms
  auto const res_399ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 399ms);

  EXPECT_FALSE(res_399ms);

  // Sim 400ms
  auto const res_400ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 400ms);

  EXPECT_TRUE(res_400ms);

  // Sim 1s
  auto const res_1s = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 1s);

  EXPECT_TRUE(res_1s);
}

TEST(Mechanics_Sphere_vs_Sphere_Detection, Basic_SpheresAtMedio_T0_ColDT)
{
  types::HighResolutionTP const& t_0       = types::HighResolutionClock::now();
  types::HighResolutionTP const& sphere_tc = t_0 + 200ms;
  types::HighResolutionTP const& other_tc  = t_0 + 200ms;

  // Sphere
  types::Point3 const    sphere_p{-5.0, 0.0, 0.0};
  types::ValueType const sphere_r{1.0};
  types::Point3 const    sphere_v{10.0, 0.0, 0.0};

  // Other
  types::Point3 const    other_p{5.0, 0.0, 0.0};
  types::ValueType const other_r{1.0};
  types::Point3 const    other_v{-10.0, 0.0, 0.0};

  // No gravity
  types::Vector3 const external_forces{0, 0, 0};

  // Sim 0ms
  auto const res_0ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 0ms);

  EXPECT_FALSE(res_0ms);

  // Sim 16ms
  auto const res_16ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 16ms);

  EXPECT_FALSE(res_16ms);

  // Sim 599ms
  auto const res_599ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 599ms);

  EXPECT_FALSE(res_599ms);

  // Sim 600ms
  auto const res_600ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 600ms);

  EXPECT_TRUE(res_600ms);

  // Sim 1s
  auto const res_1s = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 1s);

  EXPECT_TRUE(res_1s);
}

TEST(Mechanics_Sphere_vs_Sphere_Detection, Basic_SpheresAtPast_ColDT)
{
  types::HighResolutionTP const& t_0       = types::HighResolutionClock::now();
  types::HighResolutionTP const& sphere_tc = t_0 + 1s;
  types::HighResolutionTP const& other_tc  = t_0 + 1s;

  // Sphere
  types::Point3 const    sphere_p{-5.0, 0.0, 0.0};
  types::ValueType const sphere_r{1.0};
  types::Point3 const    sphere_v{10.0, 0.0, 0.0};

  // Other
  types::Point3 const    other_p{5.0, 0.0, 0.0};
  types::ValueType const other_r{1.0};
  types::Point3 const    other_v{-10.0, 0.0, 0.0};

  // No gravity
  types::Vector3 const external_forces{0, 0, 0};

  // Sim 0ms
  auto const res_0ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 0ms);

  EXPECT_FALSE(res_0ms);

  // Sim 16ms
  auto const res_16ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 16ms);

  EXPECT_FALSE(res_16ms);

  // Sim 599ms
  auto const res_599ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 599ms);

  EXPECT_FALSE(res_599ms);

  // Sim 600ms
  auto const res_600ms = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 600ms);

  EXPECT_FALSE(res_600ms);

  // Sim 1s
  auto const res_1s = mechanics::detectCollisionSphereSphere(
    sphere_tc, sphere_p, sphere_r, sphere_v, other_tc, other_p, other_r,
    other_v, external_forces, t_0, 1s);

  EXPECT_FALSE(res_1s);
}
