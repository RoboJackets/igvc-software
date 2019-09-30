#ifndef SRC_FILTER_H
#define SRC_FILTER_H

#include <pointcloud_filter/bundle.h>

namespace pointcloud_filter
{
class Filter
{
public:
  virtual void filter(Bundle& bundle) = 0;
};
}  // namespace pointcloud_filter

#endif  // SRC_FILTER_H
