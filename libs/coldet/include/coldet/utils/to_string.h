#ifndef DTE3607_UTILS_TO_STRING_H
#define DTE3607_UTILS_TO_STRING_H

namespace dte3607::coldet::utils
{

  template <typename ValueType_T, size_t Dim_T>
  std::string toString(gm::VectorT<ValueType_T, Dim_T> const& vec)
  {
    static_assert(Dim_T > 0, "Dim_T must be greater than zero");

    std::string s("[");
    for (auto i = 0ul; i < Dim_T - 1; ++i) s += std::to_string(vec[i]) + ", ";

    s += std::to_string(vec[Dim_T - 1]) + "]";

    return s;
  }

}   // namespace dte3607::coldet::utils

#endif // DTE3607_UTILS_TO_STRING_H
