#ifndef NODEUTILS_HPP
#define NODEUTILS_HPP

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <string.h>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

#include <ros/ros.h>

namespace igvc
{
enum class Assertion
{
  NONE,
  POSITIVE,
  NEGATIVE
};

template <class T>
inline std::string to_string(const std::vector<T>& v)
{
  std::stringstream ss;
  for(size_t i = 0; i < v.size(); ++i)
  {
    if(i != 0)
      ss << ",";
    ss << v[i];
  }
  return ss.str();
}

template <class T>
inline void warn_with_message(const std::string &node_namespace, const std::string &variable_name, const T &variable)
{
  ROS_WARN_STREAM("[" << node_namespace << "] Missing parameter " << variable_name
                      << ". Continuing with default values " << variable);
}

template <class T>
inline void warn_with_message(const std::string &node_namespace, const std::string &variable_name,
                              const std::vector<T> &variable)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] Missing parameter " << variable_name << ". Continuing with default values [";
  ss << to_string(variable) << "].";
  ROS_WARN_STREAM(ss.str());
}

template <class T>
inline void fail_with_message(const std::string &node_namespace, const std::string &variable_name, const T &variable,
                              const std::string &message)
{
  ROS_ERROR_STREAM("[" << node_namespace << "] " << variable_name << " (" << variable << ") should be " << message
                       << " Exiting...");
  ros::shutdown();
}

template <class T>
inline void fail_with_message(const std::string &node_namespace, const std::string &variable_name, const T &element,
                              const std::vector<T> &variable, const std::string &message)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] " << variable_name << " ([";
  ss << to_string(variable) << "]) includes element " << element << " which should be " << message << " Exiting...";
  ROS_WARN_STREAM(ss.str());
  ros::shutdown();
}

template <class T, typename... Ts>
inline void assert_with_default(const std::string &node_namespace, T &variable, bool condition, T &&default_value,
                                const std::string &message, Ts... ts)
{
  if (!condition)
  {
    ROS_WARN(message.c_str(), node_namespace.c_str(), ts...);
    variable = default_value;
  }
}

template <class T>
inline void assert_with_default(const std::string &node_namespace, std::vector<T> &variable,
                                std::function<bool(T)> lambda, std::vector<T> &&default_value,
                                const std::string &message, const std::string &condition_string)
{
  for (T element : variable)
  {
    if (!lambda(element))
    {
      ROS_WARN_STREAM(message << element << condition_string << " Setting to default value [" << to_string(default_value) << "].");
      variable = default_value;
    }
  }
}

template <class T>
inline void assert_positive_with_default(const std::string &node_namespace, T &variable, T &&default_value,
                                         const std::string &variable_name)
{
  std::ostringstream message;
  message << "[" << node_namespace << "] " << variable_name << " (currently " << variable
          << ") should be greater than 0. Setting to default value of " << default_value;
  assert_with_default(node_namespace, variable, variable > 0, std::forward<T>(default_value), message.str());
}

template <class T>
inline void assert_positive_with_default(const std::string &node_namespace, std::vector<T> &variable,
                                         std::vector<T> &&default_value, const std::string &variable_name)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] " << variable_name << " ([";
  ss << to_string(variable) << "]) includes element ";
  assert_with_default(node_namespace, variable, [](T x) { return x > 0; }, std::forward<std::vector<T>>(default_value), ss.str(),
                      " greater than 0.");
}

template <class T>
inline void assert_negative_with_default(const std::string &node_namespace, T &variable, T &&default_value,
                                         const std::string &variable_name)
{
  std::ostringstream message;
  message << "[" << node_namespace << "] " << variable_name << " (currently " << variable
          << ") should be less than 0. Setting to default value of " << default_value;
  assert_with_default(node_namespace, variable, variable < 0, std::forward<T>(default_value), message.str());
}

template <typename T>
inline void assert_negative_with_default(const std::string &node_namespace, std::vector<T> &variable,
                                         std::vector<T> &&default_value, const std::string &variable_name)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] " << variable_name << " ([";
  ss << to_string(variable) << "]) includes element ";
  assert_with_default(node_namespace, variable, [](T x) { return x < 0; }, std::forward<std::vector<T>>(default_value), ss.str(),
                      " less than 0.");
}

template <class T>
inline void check_assertion_with_default(Assertion assertion, const std::string &node_namespace, T &variable,
                                         T &&default_value, const std::string &variable_name)
{
  switch (assertion)
  {
    case Assertion::POSITIVE:
      assert_positive_with_default(node_namespace, variable, std::forward<T>(default_value), variable_name);
      break;
    case Assertion::NEGATIVE:
      assert_negative_with_default(node_namespace, variable, std::forward<T>(default_value), variable_name);
      break;
    case Assertion::NONE:
      break;
  }
}

template <class T>
inline void assert_positive(const std::string &node_namespace, const T &variable, const std::string &variable_name)
{
  if (!(variable > 0))
  {
    fail_with_message(node_namespace, variable_name, variable, "greater than 0.");
  }
}

template <class T>
inline void assert_positive(const std::string &node_namespace, const std::vector<T> &variable,
                            const std::string &variable_name)
{
  for (auto element : variable)
  {
    if (!(element > 0))
    {
      fail_with_message(node_namespace, variable_name, element, variable, "greater than 0.");
    }
  }
}

template <class T>
inline void assert_negative(const std::string &node_namespace, const T &variable, const std::string &variable_name)
{
  if (!(variable < 0))
  {
    fail_with_message(node_namespace, variable_name, variable, "less than 0.");
  }
}

template <class T>
inline void assert_negative(const std::string &node_namespace, const std::vector<T> &variable,
                            const std::string &variable_name)
{
  for (auto element : variable)
  {
    if (!(element < 0))
    {
      fail_with_message(node_namespace, variable_name, element, variable, "less than 0.");
    }
  }
}

template <class T>
inline void check_assertion(Assertion assertion, const std::string &node_namespace, T &variable,
                            const std::string &variable_name)
{
  switch (assertion)
  {
    case Assertion::POSITIVE:
      assert_positive(node_namespace, variable, variable_name);
      break;
    case Assertion::NEGATIVE:
      assert_negative(node_namespace, variable, variable_name);
      break;
    case Assertion::NONE:
      break;
  }
}

template <class T, class AssertionFunction>
inline void check_assertion(AssertionFunction assertion, const std::string &node_namespace, T &variable,
    const std::string &variable_name)
{
  if (!assertion(variable)) {
    fail_with_message(node_namespace, variable_name, variable, "able to pass the assertion.");
  }
}

template <class T>
void param(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val, T &&default_val,
           Assertion assertion = Assertion::NONE)
{
  if (!pNh.param(param_name, param_val, default_val))
  {
    warn_with_message(pNh.getNamespace(), param_name, default_val);
  }
  else
  {
    check_assertion_with_default(assertion, pNh.getNamespace(), param_val, std::forward<T>(default_val), param_name);
  }
}

template <class T>
void getParam(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val, Assertion assertion)
{
  if (!pNh.getParam(param_name, param_val))
  {
    ROS_ERROR_STREAM("[" << pNh.getNamespace() << "] Missing parameter " << param_name << ". Exiting...");
    ros::shutdown();
  }
  else
  {
    check_assertion(assertion, pNh.getNamespace(), param_val, param_name);
  }
}

template <class T, class AssertionFunction>
void getParam(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val, AssertionFunction assertion)
{
  if (!pNh.getParam(param_name, param_val))
  {
    ROS_ERROR_STREAM("[" << pNh.getNamespace() << "] Missing parameter " << param_name << ". Exiting...");
    ros::shutdown();
  }
  else
  {
    check_assertion(assertion, pNh.getNamespace(), param_val, param_name);
  }
}

template <class T>
void getParam(const ros::NodeHandle &pNh, const std::string &param_name, T &param_val)
{
  if (!pNh.getParam(param_name, param_val))
  {
    ROS_ERROR_STREAM("[" << pNh.getNamespace() << "] Missing parameter " << param_name << ". Exiting...");
    ros::shutdown();
  }
}

/**
make_unique method for when using versions of c++ < 14
*/
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/**
Calculates euclidian distance between two points

@tparam T the data type of the input points to calculate the euclidian distance for
@param[in] x1 x value of first point
@param[in] y1 y value of first point
@param[in] x2 x value of second point
@param[in] y2 y value of second point
@return the euclidian distance between both points
*/
template <typename T>
inline T get_distance(T x1, T y1, T x2, T y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

/**
Calculates euclidian distance between two points

@tparam T the data type of the input points to calculate the euclidian distance for
@param[in] p1 the <x,y> coords of the first point
@param[in] p2 the <x,y> coords of the second point
@return the euclidian distance between both points
*/
inline double get_distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return igvc::get_distance(p1.x, p1.y, p2.x, p2.y);
}

/**
Calculates euclidian distance between two points, taking tuples for each
(x,y) point as arguments

@tparam T the data type contained within each input tuple
@param[in] p1 the first point
@param[in] p2 the second point
@return the euclidian distance between both points
*/
template <typename T>
inline T get_distance(const std::tuple<T, T> &p1, const std::tuple<T, T> &p2)
{
  return igvc::get_distance(std::get<0>(p1), std::get<1>(p1), std::get<0>(p2), std::get<1>(p2));
}

/**
symmetric round up
Bias: away from zero

@tparam T the data type to round up
@param[in] the value to round up
@return the value rounded away from zero
*/
template <typename T>
T ceil0(const T &value)
{
  return (value < 0.0) ? std::floor(value) : std::ceil(value);
}

/**
Adjust angle to lie within the polar range [-PI, PI]
*/
inline void fit_to_polar(double &angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
}

/**
Computes the egocentric polar angle of vec2 wrt vec1 in 2D, that is:
  - clockwise: negative
  - counter-clockwise: positive

source: https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors

@param[in] angle the double variable to assign the computed angle to
@param[in] vec2 the vector the angle is computed with respect to
@param[in] vec1 the reference vector
*/
inline void compute_angle(double &angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
{
  double dot = vec2[0] * vec1[0] + vec2[1] * vec1[1];  // dot product - proportional to cos
  double det = vec2[0] * vec1[1] - vec2[1] * vec1[0];  // determinant - proportional to sin

  angle = atan2(det, dot);
}

}  // namespace igvc
#endif
