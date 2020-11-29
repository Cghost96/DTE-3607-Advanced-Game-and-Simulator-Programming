#include <coldet/mechanics/sphere_vs_fixed_plane_detection.h>
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
///

using namespace dte3607::coldet;

TEST(Mechanics_Sphere_vs_FixedPlane_CollisionDetection, Basic_SphereAtT0)
{
  types::HighResolutionTP const& t_0       = types::HighResolutionClock::now();
  types::HighResolutionTP const& sphere_tc = t_0;

  // The sphere
  types::Point3 const  sphere_p{0.0, 10.0, 0};
  types::ValueType               sphere_r{1.0};
  types::Vector3 const sphere_v{10.0, 0, 0};

  // The plane
  types::Point3 const  fplane_q{10., 0, 0};
  types::Vector3 const fplane_n{-1, 0, 0};

  // No gravity
  types::Vector3 const external_forces{0, 0, 0};


  // Sim 16 [ms]
  auto const res_16ms = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 16ms);
  EXPECT_FALSE(res_16ms);

  // Sim 1 [s]
  auto const res_1s = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 1s);
  EXPECT_TRUE(res_1s);
  if(res_1s) {
    auto const res_1s_value = res_1s.value();
    EXPECT_DOUBLE_EQ(res_1s_value, 0.9);
  }

  // Sim 1 [ms]
  auto const res_2s = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 2s);
  EXPECT_TRUE(res_2s);
}

TEST(Mechanics_Sphere_vs_FixedPlane_CollisionDetection, Basic_SphereAtMedio_T0_ColDT)
{
  types::HighResolutionTP const& t_0       = types::HighResolutionClock::now();
  types::HighResolutionTP const& sphere_tc = t_0 + 500ms;

  // The sphere
  types::Point3 const  sphere_p{0.0, 10.0, 0};
  types::ValueType               sphere_r{1.0};
  types::Vector3 const sphere_v{10.0, 0, 0};

  // The plane
  types::Point3 const  fplane_q{10., 0, 0};
  types::Vector3 const fplane_n{-1, 0, 0};

  // No gravity
  types::Vector3 const external_forces{0, 0, 0};


  // Sim 200 [ms]
  auto const res_200ms = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 200ms);
  EXPECT_FALSE(res_200ms);

  // Sim 1 [s]
  auto const res_1s = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 1s);
  EXPECT_FALSE(res_1s);

  // Sim 1 [ms]
  auto const res_2s = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 2s);
  EXPECT_TRUE(res_2s);
}

TEST(Mechanics_Sphere_vs_FixedPlane_CollisionDetection, Basic_SphereAtPast_ColDT)
{
  types::HighResolutionTP const& t_0       = types::HighResolutionClock::now();
  types::HighResolutionTP const& sphere_tc = t_0 + 2s;

  // The sphere
  types::Point3 const  sphere_p{0.0, 10.0, 0};
  types::ValueType               sphere_r{1.0};
  types::Vector3 const sphere_v{10.0, 0, 0};

  // The plane
  types::Point3 const  fplane_q{10., 0, 0};
  types::Vector3 const fplane_n{-1, 0, 0};

  // No gravity
  types::Vector3 const external_forces{0, 0, 0};


  // Sim 200 [ms]
  auto const res_200ms = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 200ms);

  // Sim 1 [s]
  auto const res_1s = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 1s);

  // Sim 1 [ms]
  auto const res_2s = mechanics::detectCollisionSphereFixedPlane(
    sphere_tc, sphere_p, sphere_r, sphere_v, fplane_q, fplane_n,
    external_forces, t_0, 2s);

  EXPECT_FALSE(res_200ms);
  EXPECT_FALSE(res_1s);
  EXPECT_FALSE(res_2s);
}


TEST(Mechanics_Sphere_vs_FixedPlane_CollisionDetection, Basic_SphereTouching)
{
  EXPECT_TRUE(false);
}
