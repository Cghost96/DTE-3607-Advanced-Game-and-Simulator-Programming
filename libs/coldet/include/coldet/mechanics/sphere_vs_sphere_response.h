#ifndef DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
#define DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H

#include "../bits/types.h"
#include "../../../3rdparty/gmlib2/include/gmlib/core/gm2_blaze.h"
#include <blaze/Math.h>
#include <cmath>

namespace dte3607::coldet::mechanics {

inline
//template <concepts::Vector Vector_T, std::floating_point Scalar_T>
std::pair<types::Vector3, types::Vector3>
computeImpactResponseSphereSphere(
    [[maybe_unused]] types::Point3 const&  s1_p,
    [[maybe_unused]] types::Vector3 const& s1_v,
    [[maybe_unused]] types::ValueType      s1_mass,
    [[maybe_unused]] types::Point3 const&  s2_p,
    [[maybe_unused]] types::Vector3 const& s2_v,
    [[maybe_unused]] types::ValueType      s2_mass) {
    auto const p_0 = s1_p;
    auto const p_1 = s2_p;
    auto const v_0 = s1_v;
    auto const v_1 = s2_v;
    auto const m_0 = s1_mass;
    auto const m_1 = s2_mass;
    auto const d   = blaze::evaluate(blaze::normalize(p_1 - p_0)); // Force blaze to assign a type
    auto const liv_d = gm::algorithms::linearIndependentVector(d);
    auto const n = blaze::normalize(blaze::cross(liv_d, d));
    auto const inner_v_0_d = blaze::inner(v_0, d);
    auto const inner_v_1_d = blaze::inner(v_1, d);
    auto const inner_v_0_n = blaze::inner(v_0, n);
    auto const inner_v_1_n = blaze::inner(v_1, n);
    auto const v_prime_0_d = (((m_0 - m_1) / (m_0 + m_1)) * inner_v_0_d)
                             + (((2 * m_1) / (m_0 + m_1)) * inner_v_1_d);
    auto const v_prime_1_d = (((m_1 - m_0) / (m_0 + m_1)) * inner_v_1_d)
                             + (((2 * m_0) / (m_0 + m_1)) * inner_v_0_d);
    auto const v_prime_0 = inner_v_0_n * n + v_prime_0_d * d;
    auto const v_prime_1 = inner_v_1_n * n + v_prime_1_d * d;

    return {v_prime_0, v_prime_1};
}


}   // namespace dte3607::coldet::mechanics


#endif // DTE3607_COLDET_MECHANICS_SPHERE_VS_SPHERE_RESPONSE_H
