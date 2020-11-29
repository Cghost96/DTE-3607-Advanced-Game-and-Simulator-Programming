#ifndef DTE3607_CONCEPTS_RIGIDBODY_CONCEPTS_H
#define DTE3607_CONCEPTS_RIGIDBODY_CONCEPTS_H

#include "../bits/types.h"

// stl
#include <concepts>

// clang-format off

namespace dte3607::coldet::concepts
{

  namespace linalg {

    template <typename Object_T>
    concept HasLinAlgTypes = requires
    {
      // Value Type (element type)
      std::floating_point<typename Object_T::ValueType>;

      // SpaceObjectFrame
      typename Object_T::SpaceObjectFrame;

      // Space types : homogenous coordinates
      typename Object_T::Point3H;
      typename Object_T::Vector3H;

      // Space types : vector space coordinates
      typename Object_T::Point3;
      typename Object_T::Vector3;
    };

    template <typename Object_T>
    concept HasLinAlgOperations = HasLinAlgTypes<Object_T> and
        requires(Object_T obj) {
          {obj.spaceObjectFrame()} ->
            std::convertible_to<typename Object_T::SpaceObjectFrame>; };

  }   // namespace linalg


  namespace mechanics {

    template <typename Object_T>
    concept Dynamic =
        linalg::HasLinAlgOperations<Object_T>
        and requires(Object_T obj, typename Object_T::Vector3 v) {
          { obj.velocity() } ->std::convertible_to<typename Object_T::Vector3>;
          { obj.addAcceleration(v) };
          { obj.setVelocity(v) };
        };

    template <typename Object_T>
    concept HasMass =
        requires(Object_T obj) {
          { obj.mass() } ->std::convertible_to<typename Object_T::ValueType>; };

    template <typename Object_T>
    concept Timeaware =
        requires(Object_T obj) { {obj.timepoint()}; {true}; };

    template <typename Object_T>
    concept HasFrictionCoef =
        requires(Object_T obj) { { obj.frictionCoef() }; };

    template <typename Object_T>
    concept HasStates =
        requires(Object_T obj) {
          {obj.state()} -> std::convertible_to<typename Object_T::States>;
        };

    template <typename Object_T>
    concept HasFreeState = requires { {Object_T::States::Free}; };

    template <typename Object_T>
    concept HasRestingState = requires { {Object_T::States::Resting}; };

    template <typename Object_T>
    concept HasSlidingState = requires { {Object_T::States::Sliding}; };

    template <typename Object_T>
    concept HasRollingState = requires { {Object_T::States::Rolling}; };

  }   // namespace mechanics


  namespace geometry
  {
    template <typename FixedPlane_T>
    concept IsPlane = requires(FixedPlane_T obj)
    {
      { obj.point() } ->std::convertible_to<typename FixedPlane_T::Point3>;
      { obj.normal() } ->std::convertible_to<typename FixedPlane_T::Vector3>;
    };

    template <typename Sphere_T>
    concept IsSphere = requires(Sphere_T& obj)
    {
      { obj.point() }  -> std::convertible_to<typename Sphere_T::Point3>;
      { obj.radius() } -> std::convertible_to<typename Sphere_T::ValueType>;
    };

  }   // namespace geometry

}   // namespace dte3607::coldet::concepts

// clang-format on

#endif   // DTE3607_CONCEPTS_RIGIDBODY_CONCEPTS_H
