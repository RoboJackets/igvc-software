#ifndef SRC_PROBABILITY_UTILS_H
#define SRC_PROBABILITY_UTILS_H

namespace probability_utils
{
template <typename T>
T toLogOdds(T probability)
{
  return std::log(probability / (1 - probability));
}

template <typename T>
T fromLogOdds(T logodds)
{
  return 1.0 - (1.0 / (1 + std::exp(logodds)));
}
}  // namespace probability_utils

#endif  // SRC_PROBABILITY_UTILS_H
