#include <gtest/gtest.h>

#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>

class TestAssertions : public testing::Test
{
public:
  TestAssertions() : handle_{} {};

protected:
  virtual void SetUp()
  {
    ros::start();
    handle_.deleteParam(parameter1);
    handle_.deleteParam(parameter2);
    handle_.deleteParam(parameter3);
  }

  virtual void TearDown()
  {
    if (ros::ok())
    {
      ros::shutdown();
    }
  }

  const std::string parameter1 = "first_param";
  const std::string parameter2 = "second_param";
  const std::string parameter3 = "third_param";
  ros::NodeHandle handle_;
};

//========== Basic Functionality ==========
TEST_F(TestAssertions, getParamGetsParam)
{
  {
    int param;
    int set_param = 1;
    handle_.setParam(parameter1, set_param);
    igvc::getParam(handle_, parameter1, param);
    EXPECT_EQ(param, set_param);
  }
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    handle_.setParam(parameter2, set_param);
    igvc::getParam(handle_, parameter2, param);
    EXPECT_STREQ(param.c_str(), set_param.c_str());
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 3.0, 1.0, 4.0, 1.0, 5.0 };
    handle_.setParam(parameter3, set_param);
    igvc::getParam(handle_, parameter3, param);
    EXPECT_EQ(param.size(), set_param.size());
    for (size_t i = 0; i < set_param.size(); i++)
    {
      EXPECT_DOUBLE_EQ(param[i], set_param[i]);
    }
  }
}

TEST_F(TestAssertions, paramGetsParam)
{
  {
    int param;
    int set_param = 1;
    handle_.setParam(parameter1, set_param);
    igvc::param(handle_, parameter1, param, 0);
    EXPECT_EQ(param, set_param);
  }
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    handle_.setParam(parameter2, set_param);
    igvc::param(handle_, parameter2, param, std::string{ "" });
    EXPECT_STREQ(param.c_str(), set_param.c_str());
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 3.0, 1.0, 4.0, 1.0, 5.0 };
    handle_.setParam(parameter3, set_param);
    igvc::param(handle_, parameter3, param, std::vector<double>{});
    EXPECT_EQ(param.size(), set_param.size());
    for (size_t i = 0; i < set_param.size(); i++)
    {
      EXPECT_DOUBLE_EQ(param[i], set_param[i]);
    }
  }
}

//========== Guarantees that parameter is set ==========
TEST_F(TestAssertions, getParamEnsuresParamIsSet)
{
  {
    int param;
    igvc::getParam(handle_, parameter1, param);
    EXPECT_TRUE(ros::isShuttingDown());
  }
}

TEST_F(TestAssertions, paramUsesDefaultValue)
{
  {
    int param;
    int set_param = 1;
    igvc::param(handle_, parameter1, param, set_param);
    EXPECT_EQ(param, set_param);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    igvc::param(handle_, parameter2, param, set_param);
    EXPECT_STREQ(param.c_str(), set_param.c_str());
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 3.0, 1.0, 4.0, 1.0, 5.0 };
    igvc::param(handle_, parameter3, param, set_param);
    EXPECT_EQ(param.size(), set_param.size());
    for (size_t i = 0; i < set_param.size(); i++)
    {
      EXPECT_DOUBLE_EQ(param[i], set_param[i]);
    }
    EXPECT_FALSE(ros::isShuttingDown());
  }
}

//========== Assertions ==========
TEST_F(TestAssertions, getParamCanAssertTrueUsingIGVCAssertion)
{
  {
    int param;
    int set_param = 1;
    handle_.setParam(parameter1, set_param);
    igvc::getParam(handle_, parameter1, param, igvc::Assertion::POSITIVE);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    int param;
    int set_param = -1;
    handle_.setParam(parameter1, set_param);
    igvc::getParam(handle_, parameter1, param, igvc::Assertion::NEGATIVE);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 3.0, 1.0, 4.0, 1.0, 5.0 };
    handle_.setParam(parameter3, set_param);
    igvc::getParam(handle_, parameter3, param, igvc::Assertion::POSITIVE);
    EXPECT_EQ(param.size(), set_param.size());
    for (size_t i = 0; i < set_param.size(); i++)
    {
      EXPECT_DOUBLE_EQ(param[i], set_param[i]);
    }
    EXPECT_FALSE(ros::isShuttingDown());
  }
}

TEST_F(TestAssertions, getParamCanAssertFalseUsingIGVCAssertionNegativeInt)
{
  int param;
  int set_param = 1;
  handle_.setParam(parameter1, set_param);
  igvc::getParam(handle_, parameter1, param, igvc::Assertion::NEGATIVE);
  EXPECT_TRUE(ros::isShuttingDown());
}
TEST_F(TestAssertions, getParamCanAssertFalseUsingIGVCAssertionPositiveInt)
{
  int param;
  int set_param = -1;
  handle_.setParam(parameter1, set_param);
  igvc::getParam(handle_, parameter1, param, igvc::Assertion::POSITIVE);
  EXPECT_TRUE(ros::isShuttingDown());
}
TEST_F(TestAssertions, getParamCanAssertFalseUsingIGVCAssertionNegativeVector)
{
  std::vector<double> param;
  std::vector<double> set_param{ -3.0, -1.0, -4.0, 1.0, -5.0 };
  handle_.setParam(parameter3, set_param);
  igvc::getParam(handle_, parameter3, param, igvc::Assertion::NEGATIVE);
  EXPECT_TRUE(ros::isShuttingDown());
}
TEST_F(TestAssertions, getParamCanAssertFalseUsingIGVCAssertionPostitiveVector)
{
  std::vector<double> param;
  std::vector<double> set_param{ 3.0, -1.0, 4.0, 1.0, 5.0 };
  handle_.setParam(parameter3, set_param);
  igvc::getParam(handle_, parameter3, param, igvc::Assertion::POSITIVE);
  EXPECT_TRUE(ros::isShuttingDown());
}
// ========== getParam lambdas ==========
TEST_F(TestAssertions, getParamCanAssertTrueUsingLambda)
{
  int param;
  int set_param = -1;
  handle_.setParam(parameter1, set_param);
  igvc::getParam(handle_, parameter1, param, [](const int& x) { return x < -0.5; });
  EXPECT_EQ(param, set_param);
  EXPECT_FALSE(ros::isShuttingDown());
}
TEST_F(TestAssertions, getParamCanAssertFalseUsingLambda)
{
  std::string param;
  std::string set_param = "The cake";
  handle_.setParam(parameter1, set_param);
  igvc::getParam(handle_, parameter1, param, [](const std::string& x) { return x == "is a lie"; });
  EXPECT_TRUE(ros::isShuttingDown());
}
TEST_F(TestAssertions, getParamCanAssertTrueUsingLambdaVector)
{
  std::vector<double> param;
  std::vector<double> set_param{ 0.3, -0.1, 0.4, 0.1, 0.5 };
  handle_.setParam(parameter3, set_param);
  igvc::getParam(handle_, parameter3, param, [](const double& x) { return -1 <= x && x <= 1; });
  EXPECT_EQ(param.size(), set_param.size());
  for (size_t i = 0; i < set_param.size(); i++)
  {
    EXPECT_DOUBLE_EQ(param[i], set_param[i]);
  }
  EXPECT_FALSE(ros::isShuttingDown());
}
TEST_F(TestAssertions, getParamCanAssertFalseUsingLambdaVector)
{
  std::vector<double> param;
  std::vector<double> set_param{ 0.3, -1.1, 0.4, 0.1, 0.5 };
  handle_.setParam(parameter3, set_param);
  igvc::getParam(handle_, parameter3, param, [](const double& x) { return -1 <= x && x <= 1; });
  EXPECT_TRUE(ros::isShuttingDown());
}

// ========== param ==========
TEST_F(TestAssertions, paramCanAssertTrueUsingIGVCAssertion)
{
  {
    int param;
    int set_param = 1;
    handle_.setParam(parameter1, set_param);
    igvc::param(handle_, parameter1, param, -1, igvc::Assertion::POSITIVE);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    int param;
    int set_param = -1;
    handle_.setParam(parameter1, set_param);
    igvc::param(handle_, parameter1, param, 1, igvc::Assertion::NEGATIVE);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 3.0, 1.0, 4.0, 1.0, 5.0 };
    handle_.setParam(parameter3, set_param);
    igvc::param(handle_, parameter3, param, std::vector<double>{}, igvc::Assertion::POSITIVE);
    EXPECT_EQ(param.size(), set_param.size());
    for (size_t i = 0; i < set_param.size(); i++)
    {
      EXPECT_DOUBLE_EQ(param[i], set_param[i]);
    }
    EXPECT_FALSE(ros::isShuttingDown());
  }
}

TEST_F(TestAssertions, paramCanAssertTrueUsingIGVCAssertionDefault)
{
  {
    int param;
    int set_param = 1;
    igvc::param(handle_, parameter1, param, set_param, igvc::Assertion::POSITIVE);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    int param;
    int set_param = -1;
    igvc::param(handle_, parameter1, param, set_param, igvc::Assertion::NEGATIVE);
    EXPECT_FALSE(ros::isShuttingDown());
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 3.0, 1.0, 4.0, 1.0, 5.0 };
    igvc::param(handle_, parameter3, param, set_param, igvc::Assertion::POSITIVE);
    EXPECT_EQ(param.size(), set_param.size());
    for (size_t i = 0; i < set_param.size(); i++)
    {
      EXPECT_DOUBLE_EQ(param[i], set_param[i]);
    }
    EXPECT_FALSE(ros::isShuttingDown());
  }
}

TEST_F(TestAssertions, paramCanAssertFalseUsingIGVCAssertionNegativeInt)
{
  int param;
  int set_param = 1;
  handle_.setParam(parameter1, set_param);
  igvc::param(handle_, parameter1, param, 2, igvc::Assertion::NEGATIVE);
  EXPECT_TRUE(ros::isShuttingDown());
}
TEST_F(TestAssertions, paramCanAssertFalseUsingIGVCAssertionNegativeDefault)
{
  double param;
  igvc::param(handle_, parameter1, param, 1.0, igvc::Assertion::NEGATIVE);
  EXPECT_TRUE(ros::isShuttingDown());
}

// =========== param lambdas ===========

TEST_F(TestAssertions, paramCanAssertTrueUsingLambda)
{
  double param;
  double set_param = 1.5;
  handle_.setParam(parameter1, set_param);
  igvc::param(handle_, parameter1, param, 3, [](const double& x) { return x <= 2; });
  EXPECT_DOUBLE_EQ(param, set_param);
  EXPECT_FALSE(ros::isShuttingDown());
}
TEST_F(TestAssertions, paramCanAssertFalseUsingLambda)
{
  double param;
  double set_param = 3;
  handle_.setParam(parameter1, set_param);
  igvc::param(handle_, parameter1, param, 4, [](const double& x) { return x <= 2; });
  EXPECT_TRUE(ros::isShuttingDown());
}
TEST_F(TestAssertions, paramCanAssertTrueUsingLambdaDefault)
{
  double param;
  igvc::param(handle_, parameter1, param, 0, [](const double& x) { return x <= 2; });
  EXPECT_DOUBLE_EQ(param, param);
  EXPECT_FALSE(ros::isShuttingDown());
}
TEST_F(TestAssertions, paramCanAssertTrueUsingLambdaDefaultVector)
{
  std::vector<double> param;
  std::vector<double> set_param{ 0.3, -0.1, 0.4, 0.1, 0.5 };
  igvc::param(handle_, parameter3, param, set_param, [](const double& x) { return -1 <= x && x <= 1; });
  EXPECT_EQ(param.size(), set_param.size());
  for (size_t i = 0; i < set_param.size(); i++)
  {
    EXPECT_DOUBLE_EQ(param[i], set_param[i]);
  }
  EXPECT_FALSE(ros::isShuttingDown());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_assertions");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
