#ifndef SRC_EIGEN_HASH_H
#define SRC_EIGEN_HASH_H

#include <functional>

#include <Eigen/Core>

namespace std
{
template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>>
{
  // https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      Scalar elem = *(matrix.data() + i);
      seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);  // NOLINT
    }
    return seed;
  }
};

template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Array<Scalar, Rows, Cols>>
{
  // https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
  size_t operator()(const Eigen::Array<Scalar, Rows, Cols>& matrix) const
  {
    size_t seed = 0;
    for (Eigen::Index i = 0; i < matrix.size(); ++i)
    {
      Scalar elem = *(matrix.data() + i);
      seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);  // NOLINT
    }
    return seed;
  }
};

template <typename Scalar, int Rows, int Cols>
struct equal_to<Eigen::Array<Scalar, Rows, Cols>>
{
  bool operator()(const Eigen::Array<Scalar, Rows, Cols>& lhs, const Eigen::Array<Scalar, Rows, Cols>& rhs) const
  {
    for (Eigen::Index i = 0; i < lhs.size(); ++i)
    {
      if (*(lhs.data() + i) != *(rhs.data() + i))
      {
        return false;
      }
    }
    return true;
  }
};

}  // namespace std

#endif  // SRC_EIGEN_HASH_H
