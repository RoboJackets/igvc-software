#include <math.h>

#include <gtest/gtest.h>

#include <igvc_navigation/fast_sweep.h>

TEST(FastSweepTest, constructor)
{
  int rows = 3;
  int cols = 3;
  float resolution = 1;

  fast_sweep::FastSweep solver(rows, cols, resolution);
  EXPECT_NO_THROW();
}

TEST(FastSweepTest, empty)
{
  int rows = 3;
  int cols = 3;
  float resolution = 1;

  fast_sweep::FastSweep solver(rows, cols, resolution);

  std::vector<float> costs(rows * cols, 1.0f);
  std::vector<float> solution = solver.solveEikonal({}, costs);
  EXPECT_NO_THROW();
  ASSERT_EQ(solution.size(), static_cast<size_t>(rows * cols));
  for (const auto& cell : solution) {
    EXPECT_FLOAT_EQ(cell, std::numeric_limits<float>::max());
  }
}

TEST(FastSweepTest, simple3x3)
{
  int rows = 3;
  int cols = 3;
  float resolution = 1;

  fast_sweep::FastSweep solver(rows, cols, resolution);

  std::vector<float> costs(rows * cols, 1.0f);
  std::vector<fast_sweep::Node> gammas{
      {1,1}
  };

  std::vector<float> solution = solver.solveEikonal(gammas, costs);
  EXPECT_NO_THROW();
  ASSERT_EQ(solution.size(), static_cast<size_t>(rows * cols));
  float in_between = (2+sqrt(2.0f))/2.0f;
  std::vector<float> expected = {
      in_between, 1.0, in_between,
      1.0, 0.0, 1.0,
      in_between, 1.0, in_between
  };
  for (int i = 0; i < solution.size(); i++) {
    EXPECT_FLOAT_EQ(expected[i], solution[i]);
  }
}

TEST(FastSweepTest, twoGamma3x3)
{
  int rows = 3;
  int cols = 3;
  float resolution = 1;

  fast_sweep::FastSweep solver(rows, cols, resolution);

  std::vector<float> costs(rows * cols, 1.0f);
  std::vector<fast_sweep::Node> gammas{
      {0,0}, {2,2}
  };

  std::vector<float> solution = solver.solveEikonal(gammas, costs);
  EXPECT_NO_THROW();
  ASSERT_EQ(solution.size(), static_cast<size_t>(rows * cols));
  float in_between = (2+sqrt(2.0f))/2.0f;
  std::vector<float> expected = {
      0.0, 1.0, in_between,
      1.0, in_between, 1.0,
      in_between, 1.0, 0.0
  };
  for (int i = 0; i < solution.size(); i++) {
    EXPECT_FLOAT_EQ(expected[i], solution[i]);
  }
}

TEST(FastSweepTest, simple11x11)
{
  int rows = 11;
  int cols = 11;
  float resolution = 1;

  fast_sweep::FastSweep solver(rows, cols, resolution);

  std::vector<float> costs(rows * cols, 1.0f);
  std::vector<fast_sweep::Node> gammas{
      {5,5}
  };

  std::vector<float> solution = solver.solveEikonal(gammas, costs);
  EXPECT_NO_THROW();
  ASSERT_EQ(solution.size(), static_cast<size_t>(rows * cols));
  // Symmetry
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      int idx1 = 11*i + j;
      int idx2 = 11*(10 - i) + j;
      int idx3 = 11*i + (10 - j);
      int idx4 = 11*(10 - i) + (10 - j);
      ASSERT_FLOAT_EQ(solution[idx1], solution[idx2]);
      ASSERT_FLOAT_EQ(solution[idx2], solution[idx3]);
      ASSERT_FLOAT_EQ(solution[idx3], solution[idx4]);
    }
  }
}

TEST(FastSweepTest, resolutionSimple3x3)
{
  int rows = 3;
  int cols = 3;
  float resolution = 0.5;

  fast_sweep::FastSweep solver(rows, cols, resolution);

  std::vector<float> costs(rows * cols, 1.0f);
  std::vector<fast_sweep::Node> gammas{
      {1,1}
  };

  std::vector<float> solution = solver.solveEikonal(gammas, costs);
  EXPECT_NO_THROW();
  ASSERT_EQ(solution.size(), static_cast<size_t>(rows * cols));
  float in_between = (1+sqrt(0.5f))/2.0f;
  std::vector<float> expected = {
      in_between, 0.5f, in_between,
      0.5f, 0.0, 0.5f,
      in_between, 0.5f, in_between
  };
  for (int i = 0; i < solution.size(); i++) {
    EXPECT_FLOAT_EQ(expected[i], solution[i]);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
