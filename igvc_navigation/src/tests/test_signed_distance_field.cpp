#include <gtest/gtest.h>

#include <igvc_navigation/signed_distance_field.h>

TEST(SignedDistanceField, simple)
{
  int rows = 3;
  int cols = 3;
  double start_x = 0;
  double start_y = 0;
  double resolution = 1.0;

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  path.poses.emplace_back(pose);

  signed_distance_field::SignedDistanceFieldOptions options{ rows, cols, start_x, start_y, resolution };

  cv::Mat traversal_costs(rows, cols, CV_32F, 1.0f);

  fast_sweep::FastSweep solver(rows, cols, resolution);

  std::unique_ptr<cv::Mat> solution = signed_distance_field::getSignedDistanceField(path, 0, 0, options, traversal_costs, solver);
  ASSERT_NE(solution.get(), nullptr);
  ASSERT_EQ(solution->cols, cols);
  ASSERT_EQ(solution->rows, rows);


  float in_between = (2+sqrt(2.0f))/2.0f;
  std::vector<float> expected = {
      in_between, 1.0, in_between,
      1.0, 0.0, 1.0,
      in_between, 1.0, in_between
  };
  for (int i = 0; i < expected.size(); i++) {
    EXPECT_FLOAT_EQ(expected[i], solution->at<float>(i));
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
