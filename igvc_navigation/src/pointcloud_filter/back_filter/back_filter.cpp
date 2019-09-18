#include <pointcloud_filter/back_filter/back_filter.h>

namespace pointcloud_filter
{
BackFilter::BackFilter(const BackFilterConfig& config) : config_{ config }
{
}

void BackFilter::filter(BackFilter::PointCloud& pointcloud) const
{
}
}  // namespace pointcloud_filter
