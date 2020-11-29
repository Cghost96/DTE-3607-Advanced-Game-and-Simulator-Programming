#ifndef GMLIBQTINTEGRATION_PRIVATE_CPP_HELPERS_H
#define GMLIBQTINTEGRATION_PRIVATE_CPP_HELPERS_H

// gm
#include <gmlib/core/../platform.h>
#include <gmlib/core/gm2_blaze.h>

// qt
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <QVariant>

namespace gmqt
{

  namespace utils
  {



    enum class VecType { Point, Vector };



    // gmlib vector 3 -- qt vector3
    template <typename Type_T, size_t N>
      QVector3D
      toQVectorND(gm::VectorT<Type_T, N> const&
                    v) requires std::is_same_v<Type_T, float> and (N == 3)
    {
      return {v[0], v[1], v[2]};
    }

    template <typename Type_T, size_t N>
    QVector3D toQVectorND(gm::VectorT<Type_T, N> const& v) requires(N == 3)
    {
      return {float(v[0]), float(v[1]), float(v[2])};
    }



    // gmlib vector 4 -- qt vector4
    template <typename Type_T, size_t N>
      QVector4D
      toQVectorND(gm::VectorT<Type_T, N> const&
                    v) requires std::is_same_v<Type_T, float> and (N == 4)
    {
      return {v[0], v[1], v[2], v[3]};
    }

    template <typename Type_T, size_t N>
    QVector4D toQVectorND(gm::VectorT<Type_T, N> const& v) requires(N == 4)
    {
      return {float(v[0]), float(v[1]), float(v[2]), float(v[3])};
    }



    // gmlib vector 3 -- qt vector4
    template <VecType VecType_T, typename Type_T, size_t N>
      QVector4D
      toQVector4D(gm::VectorT<Type_T, N> const&
                    v) requires std::is_same_v<Type_T, float> and (N == 3)
      and (VecType_T == VecType::Point)
    {
      return {v[0], v[1], v[2], 1.0f};
    }

    template <VecType VecType_T, typename Type_T, size_t N>
      QVector4D
      toQVector4D(gm::VectorT<Type_T, N> const&
                    v) requires std::is_same_v<Type_T, float> and (N == 3)
      and (VecType_T == VecType::Vector)
    {
      return {v[0], v[1], v[2], 0.0f};
    }

    template <VecType VecType_T, typename Type_T, size_t N>
      QVector4D toQVector4D(gm::VectorT<Type_T, N> const& v) requires(N
                                                                          == 3)
      and (VecType_T == VecType::Point)
    {
      return {float(v[0]), float(v[1]), float(v[2]), 1.0f};
    }

    template <VecType VecType_T, typename Type_T, size_t N>
      QVector4D toQVector4D(gm::VectorT<Type_T, N> const& v) requires(N
                                                                          == 3)
      and (VecType_T == VecType::Vector)
    {
      return {float(v[0]), float(v[1]), float(v[2]), 0.0f};
    }



    // gmlib vector N > 3 -- qt vector3
    template <typename Type_T, size_t N>
      QVector3D
      toQVector3D(gm::VectorT<Type_T, N> const&
                    v) requires std::is_same_v<Type_T, float> and (N >= 4)
    {
      return {v[0], v[1], v[2]};
    }

    template <typename Type_T, size_t N>
    QVector3D toQVector3D(gm::VectorT<Type_T, N> const& v) requires(N >= 4)
    {
      return {float(v[0]), float(v[1]), float(v[2])};
    }








    // qt vector2 -- gmlib vector 2
    template <typename Type_T = double>
    gm::VectorT<Type_T, 2ul>
    toGMVectorT(QVector2D const& v) requires std::is_same_v<Type_T, float>
    {
      return {v.x(), v.y()};
    }

    template <typename Type_T = double>
    gm::VectorT<Type_T, 2ul> toGMVectorT(QVector2D const& v)
    {
      return {Type_T(v.x()), Type_T(v.y())};
    }







    // qt vector2 -- gmlib vector 3 (PointH)
    template <typename Type_T = double>
    gm::VectorT<Type_T, 3ul>
    toGMPointH3D(QVector2D const& v) requires std::is_same_v<Type_T, float>
    {
      return {v.x(), v.y(), 1.0f};
    }

    template <typename Type_T = double>
    gm::VectorT<Type_T, 3ul> toGMPointH3D(QVector2D const& v)
    {
      return {Type_T(v.x()), Type_T(v.y()), 1};
    }

    // qt vector2 -- gmlib vector 3 (VectorH)
    template <typename Type_T = double>
    gm::VectorT<Type_T, 3ul>
    toGMVectorH3D(QVector2D const& v) requires std::is_same_v<Type_T, float>
    {
      return {v.x(), v.y(), 0.0f};
    }

    template <typename Type_T = double>
    gm::VectorT<Type_T, 3ul> toGMVectorH3D(QVector2D const& v)
    {
      return {Type_T(v.x()), Type_T(v.y()),0};
    }









    // qt vector3 -- gmlib vector 3
    template <typename Type_T = double>
    gm::VectorT<Type_T, 3ul>
    toGMVectorT(QVector3D const& v) requires std::is_same_v<Type_T, float>
    {
      return {v.x(), v.y(), v.z()};
    }

    template <typename Type_T = double>
    gm::VectorT<Type_T, 3ul> toGMVectorT(QVector3D const& v)
    {
      return {Type_T(v.x()), Type_T(v.y()), Type_T(v.z())};
    }


    // qt vector3 -- gmlib vector 4 (PointH)
    template <typename Type_T = double>
    gm::VectorT<Type_T, 4ul>
    toGMPointH4D(QVector3D const& v) requires std::is_same_v<Type_T, float>
    {
      return {v.x(), v.y(), v.z(), 1.0f};
    }

    template <typename Type_T = double>
    gm::VectorT<Type_T, 4ul> toGMPointH4D(QVector3D const& v)
    {
      return {Type_T(v.x()), Type_T(v.y()), Type_T(v.z()), 1};
    }

    // qt vector3 -- gmlib vector 4 (VectorH)
    template <typename Type_T = double>
    gm::VectorT<Type_T, 4ul>
    toGMVectorH4D(QVector3D const& v) requires std::is_same_v<Type_T, float>
    {
      return {v.x(), v.y(), v.z(), 0.0f};
    }

    template <typename Type_T = double>
    gm::VectorT<Type_T, 4ul> toGMVectorH4D(QVector3D const& v)
    {
      return {Type_T(v.x()), Type_T(v.y()), Type_T(v.z()),0};
    }






    template <typename... Ts>
    QString toQString(Ts const& ... ts)
    {
      QString name("#gmlib_curve_geometry");

      auto name_gen =
        [&name]<typename LT, typename... LTs>(
            auto& name_gen_ref, LT const& first, LTs const&... rest)
      {

        if constexpr(std::is_same_v<LT,QString>)
          name.append("_" + first);
        else if constexpr (std::is_same_v<LT,float>)
          name.append(QString("_%1").arg(double(first)));
        else
          name.append(QString("_%1").arg(QVariant(first).toString()));

        if constexpr (sizeof...(LTs) > 0)
          return name_gen_ref(name_gen_ref,rest...);
        else return name;
      };

      return name_gen(name_gen,ts...);
    }


  }   // namespace utils
}   // namespace gmqt

#endif   // GMLIBQTINTEGRATION_PRIVATE_CPP_HELPERS_H
