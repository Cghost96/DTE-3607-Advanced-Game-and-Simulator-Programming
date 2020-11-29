#include <coldet/mechanics/sphere_vs_fixed_plane_response.h>

#include <coldet/bits/types.h>

// gtest
#include <gtest/gtest.h>   // googletest header file


/////////////////
/// \brief TEST
///
///

using namespace dte3607::coldet;

TEST(Mechanics_Sphere_vs_FixedPlane_Response, Test001)
{

  // Sphere velocity
  types::Vector3 const sphere_v{10.0, 0, 0};

  // The plane's normal
  types::Vector3 const fplane_n{-1, 0, 0};

  auto const response
    = mechanics::computeImpactResponseSphereFixedPlane(sphere_v, fplane_n);

  auto gold_res        = types::Vector3{-10.0, 0, 0};
  auto diff_res        = gold_res - response;
  auto length_res_diff = blaze::evaluate(blaze::length(diff_res));

  EXPECT_NEAR(length_res_diff, 0, 1e-5);

//  EXPECT_TRUE(false);
}
