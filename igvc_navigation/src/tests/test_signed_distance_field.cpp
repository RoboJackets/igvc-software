#include <gtest/gtest.h>

#include <igvc_navigation/signed_distance_field.h>

class SignedDistanceFieldTest : public ::testing::Test {
protected:
  void SetUp(int rows, int cols, double start_x, double start_y, double resolution) {
    options = std::make_unique<signed_distance_field::SignedDistanceFieldOptions>(rows, cols, start_x, start_y, resolution);
    signed_distance_field = std::make_unique<signed_distance_field::SignedDistanceField>(*options);

    rows_ = rows;
    cols_ = cols;
    start_x_ = start_x;
    start_y_ = start_y;
    resolution_ = resolution;

  }

  std::unique_ptr<signed_distance_field::SignedDistanceFieldOptions> options;
  std::unique_ptr<signed_distance_field::SignedDistanceField> signed_distance_field;
  int rows_;
  int cols_;
  double start_x_;
  double start_y_;
  double resolution_;
};

TEST_F(SignedDistanceFieldTest, simple)
{
  SetUp(3, 3, 0.0, 0.0, 1.0);

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  path.poses.emplace_back(pose);

  cv::Mat traversal_costs(rows_, cols_, CV_32F, 1.0f);

  signed_distance_field->calculate(path, 0, 0, traversal_costs);
  std::unique_ptr<cv::Mat> solution = signed_distance_field->toMat();
  ASSERT_NE(signed_distance_field.get(), nullptr);
  ASSERT_EQ(solution->cols, cols_);
  ASSERT_EQ(solution->rows, rows_);


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

TEST_F(SignedDistanceFieldTest, line)
{
  SetUp(5, 5, 0.0, 0.0, 1.0);

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 2;
  pose.pose.position.y = 2;
  path.poses.emplace_back(pose);

  cv::Mat traversal_costs(rows_, cols_, CV_32F, 1.0f);

  signed_distance_field->calculate(path, 0, 1, traversal_costs);
  std::unique_ptr<cv::Mat> solution = signed_distance_field->toMat();
  ASSERT_NE(solution.get(), nullptr);
  ASSERT_EQ(solution->cols, cols_);
  ASSERT_EQ(solution->rows, rows_);

  // Check zeros are correct
  std::vector<std::pair<int, int>> zeros = {
      {2, 2}, {1, 3}, {0, 4}
  };

  for (const auto& [r, c] : zeros) {
    EXPECT_FLOAT_EQ(solution->at<float>(r, c), 0.0);
  }

  // Check that all numbers are below 10
  for (int i = 0; i < rows_; i++) {
    for (int j = 0; j < cols_; j++) {
      EXPECT_TRUE(solution->at<float>(j, i) < 10.0f);
    }
  }
}

TEST_F(SignedDistanceFieldTest, rectangle)
{
  SetUp(5, 5, 0.0, 0.0, 1.0);

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = -2;
  pose.pose.position.y = -2;
  path.poses.emplace_back(pose);
  pose.pose.position.x = -2;
  pose.pose.position.y = 2;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 2;
  pose.pose.position.y = 2;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 2;
  pose.pose.position.y = -2;
  path.poses.emplace_back(pose);
  pose.pose.position.x = -2;
  pose.pose.position.y = -2;
  path.poses.emplace_back(pose);

  cv::Mat traversal_costs(rows_, cols_, CV_32F, 1.0f);

  signed_distance_field->calculate(path, 0, 4, traversal_costs);
  std::unique_ptr<cv::Mat> solution = signed_distance_field->toMat();
  ASSERT_NE(solution.get(), nullptr);
  ASSERT_EQ(solution->cols, cols_);
  ASSERT_EQ(solution->rows, rows_);

  std::vector<float> expected = {
      0.0000e+00, 0.0000e+00,  0.0000e+00,  0.0000e+00,  0.0000e+00,
      0.0000e+00, 7.0711e-01,  9.6593e-01,  7.0711e-01,  0.0000e+00,
      0.0000e+00, 9.6593e-01,  1.6730e+00,  9.6593e-01,  0.0000e+00,
      0.0000e+00, 7.0711e-01,  9.6593e-01,  7.0711e-01,  0.0000e+00,
      0.0000e+00, 0.0000e+00,  0.0000e+00,  0.0000e+00,  0.0000e+00,
  };
  for (int i = 0; i < expected.size(); i++) {
    EXPECT_TRUE(std::fabs(expected[i] - solution->at<float>(i)) < 1e-3);
  }
}

TEST_F(SignedDistanceFieldTest, simpleResolution)
{
  SetUp(11, 11, 0.0, 0.0, 0.1);

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = -0.2;
  pose.pose.position.y = 0.3;
  path.poses.emplace_back(pose);

  cv::Mat traversal_costs(rows_, cols_, CV_32F, 1.0f);

  signed_distance_field->calculate(path, 0, 0, traversal_costs);
  std::unique_ptr<cv::Mat> solution = signed_distance_field->toMat();
  ASSERT_NE(signed_distance_field.get(), nullptr);
  ASSERT_EQ(solution->cols, cols_);
  ASSERT_EQ(solution->rows, rows_);

  // 0 is placed at the correct position
  EXPECT_FLOAT_EQ(solution->at<float>(2, 3), 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
