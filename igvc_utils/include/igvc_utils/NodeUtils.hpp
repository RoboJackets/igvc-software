#ifndef NODEUTILS_HPP
#define NODEUTILS_HPP

#include <type_traits>

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
inline std::string toString(const std::vector<T>& v);

template <bool B, typename T = void>
using disable_if = std::enable_if<!B, T>;

template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <typename T>
struct is_vector : public std::false_type
{
};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : public std::true_type
{
};

template <class T>
inline void warnWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& variable,
                            const std::string& message);
template <class T>
inline void warnWithMessage(const std::string& node_namespace, const std::string& variable_name,
                            const std::vector<T>& variable, const std::string& message);

/**
 * Prints an error to the console with a given message, then does ros::shutdown.
 * @tparam T
 * @param node_namespace
 * @param variable_name
 * @param variable
 * @param message
 */
template <class T>
inline void failWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& variable,
                            const std::string& message);
template <class T>
inline void failWithMessage(const std::string& node_namespace, const std::string& variable_name,
                            const std::vector<T>& variable, const std::string& message);
template <class T>
inline void failWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& element,
                            const std::vector<T>& variable, const std::string& message);

/**
 * Asserts that the condition passed in is true. If it isn't true, it prints a message to the console, and sets the
 * variable passed in to the default value
 * @tparam T
 * @tparam Ts
 * @param node_namespace
 * @param variable
 * @param condition
 * @param default_value
 * @param message
 * @param ts
 */
template <class T, class U, typename... Ts>
inline void assertWithDefault(const std::string& node_namespace, T& variable, bool condition, U&& default_value,
                              const std::string& message, Ts... ts);
template <class T, class U>
inline void assertWithDefault(const std::string& node_namespace, std::vector<T>& variable,
                              std::function<bool(T&&)> lambda, std::vector<U>&& default_value,
                              const std::string& message, const std::string& condition_string);

template <class T, class U>
inline void assertPositiveWithDefault(const std::string& node_namespace, T& variable, U&& default_value,
                                      const std::string& variable_name);
template <class T, class U>
inline void assertPositiveWithDefault(const std::string& node_namespace, std::vector<T>& variable,
                                      std::vector<U>&& default_value, const std::string& variable_name);

template <class T, class U>
inline void assertNegativeWithDefault(const std::string& node_namespace, T& variable, U&& default_value,
                                      const std::string& variable_name);
template <class T, class U>
inline void assertNegativeWithDefault(const std::string& node_namespace, std::vector<T>& variable,
                                      std::vector<U>&& default_value, const std::string& variable_name);

template <class T, class U, class AssertionFunction, typename disable_if<is_vector<T>::value, T>::type* = nullptr>
inline void checkAssertionWithDefault(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                                      U&& default_value, const std::string& variable_name,
                                      const std::string& error_message = "doesn't pass the assertion");

template <class T, class U, class AssertionFunction, typename std::enable_if<is_vector<T>::value, T>::type* = nullptr>
inline void checkAssertionWithDefault(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                                      U&& default_value, const std::string& variable_name,
                                      const std::string& error_message = "doesn't pass the assertion");

template <class T, class U>
inline void checkAssertionWithDefault(Assertion assertion, const std::string& node_namespace, T& variable,
                                      U&& default_value, const std::string& variable_name);

template <class T>
inline void assertPositive(const std::string& node_namespace, const T& variable, const std::string& variable_name);
template <class T>
inline void assertPositive(const std::string& node_namespace, const std::vector<T>& variable,
                           const std::string& variable_name);

template <class T>
inline void assertNegative(const std::string& node_namespace, const T& variable, const std::string& variable_name);
template <class T>
inline void assertNegative(const std::string& node_namespace, const std::vector<T>& variable,
                           const std::string& variable_name);

template <class T, class AssertionFunction>
inline void checkAssertionVector(AssertionFunction assertion, const std::string& node_namespace,
                                 std::vector<T>& variable, const std::string& variable_name,
                                 const std::string& error_message);

template <class T, class AssertionFunction, typename disable_if<is_vector<T>::value, T>::type* = nullptr>
inline void checkAssertion(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                           const std::string& variable_name,
                           const std::string& error_message = "should pass the assertion.");

template <class T, class AssertionFunction, typename std::enable_if<is_vector<T>::value, T>::type* = nullptr>
inline void checkAssertion(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                           const std::string& variable_name,
                           const std::string& error_message = "should pass the assertion.");
template <class T>
inline void checkAssertion(Assertion assertion, const std::string& node_namespace, T& variable,
                           const std::string& variable_name);

/**
 * Wrapper function for NodeHandle::param that shows a warning if the parameter isn't found and the default value is
 * used, and allows for assertions.
 * @tparam T
 * @tparam AssertionFunction
 * @param pNh
 * @param param_name
 * @param param_val
 * @param assertion
 */
template <class T, class U, class AssertionFunction>
void param(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, U&& default_val,
           AssertionFunction assertion);

template <class T, class U>
void param(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, U&& default_val,
           Assertion assertion = Assertion::NONE);

template <class T, class U, class AssertionFunction>
void paramHelper(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, U&& default_val,
                 AssertionFunction assertion);

/**
 * Wrapper function for NodeHandle::getParam that shows an error message if the parameter isn't found and allows for
 * assertions.
 * @tparam T
 * @tparam AssertionFunction
 * @param pNh
 * @param param_name
 * @param param_val
 * @param assertion
 */
template <class T>
void getParam(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, Assertion assertion);

/**
 * Wrapper function for NodeHandle::getParam that shows an error message if the parameter isn't found and allows for
 * assertions.
 * @tparam T
 * @tparam AssertionFunction
 * @param pNh
 * @param param_name
 * @param param_val
 * @param assertion a custom lambda function used for assertions
 */
template <class T, class AssertionFunction>
void getParam(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, AssertionFunction assertion);

template <class T>
void getParam(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val);

template <class T, class AssertionFunction>
void getParamHelper(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val,
                    AssertionFunction assertion);

/**
make_unique method for when using versions of c++ < 14
*/
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
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
inline double get_distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
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
inline T get_distance(const std::tuple<T, T>& p1, const std::tuple<T, T>& p2)
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
T ceil0(const T& value)
{
  return (value < 0.0) ? std::floor(value) : std::ceil(value);
}

/**
Adjust angle to lie within the polar range [-PI, PI]
*/
inline void fit_to_polar(double& angle)
{
  angle = std::fmod(angle, 2 * M_PI);
  if (angle > M_PI)
  {
    angle -= 2 * M_PI;
  }
  else if (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }
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
inline void compute_angle(double& angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
{
  double dot = vec2[0] * vec1[0] + vec2[1] * vec1[1];  // dot product - proportional to cos
  double det = vec2[0] * vec1[1] - vec2[1] * vec1[0];  // determinant - proportional to sin

  angle = atan2(det, dot);
}

}  // namespace igvc

template <class T, class U>
void igvc::param(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, U&& default_val,
                 Assertion assertion)
{
  paramHelper(pNh, param_name, param_val, std::forward<T>(default_val), assertion);
}

template <class T, class U, class AssertionFunction>
void igvc::param(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, U&& default_val,
                 AssertionFunction assertion)
{
  paramHelper(pNh, param_name, param_val, std::forward<T>(default_val), assertion);
}

template <class T, class U, class AssertionFunction>
void igvc::paramHelper(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, U&& default_val,
                       AssertionFunction assertion)
{
  if (!pNh.param(param_name, param_val, default_val))
  {
    warnWithMessage(pNh.getNamespace(), param_name, default_val, " is not set");
    checkAssertion(assertion, pNh.getNamespace(), param_val, param_name);
  }
  else
  {
    checkAssertionWithDefault(assertion, pNh.getNamespace(), param_val, std::forward<T>(default_val), param_name);
  }
}

template <class T>
void igvc::getParam(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val, Assertion assertion)
{
  getParamHelper(pNh, param_name, param_val, assertion);
}

template <class T, class AssertionFunction>
void igvc::getParam(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val,
                    AssertionFunction assertion)
{
  getParamHelper(pNh, param_name, param_val, assertion);
}

template <class T, class AssertionFunction>
void igvc::getParamHelper(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val,
                          AssertionFunction assertion)
{
  if (!pNh.getParam(param_name, param_val))
  {
    ROS_ERROR_STREAM("[" << pNh.getNamespace() << "] Missing parameter " << param_name << ". Exiting...");
    ros::shutdown();
  }
  else
  {
    checkAssertion(assertion, pNh.getNamespace(), param_val, param_name);
  }
}

template <class T>
void igvc::getParam(const ros::NodeHandle& pNh, const std::string& param_name, T& param_val)
{
  if (!pNh.getParam(param_name, param_val))
  {
    ROS_ERROR_STREAM("[" << pNh.getNamespace() << "] Missing parameter " << param_name << ". Exiting...");
    ros::shutdown();
  }
}

// Assertion method implementations
template <class T, class U, class AssertionFunction, typename std::enable_if<igvc::is_vector<T>::value, T>::type*>
inline void igvc::checkAssertionWithDefault(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                                            U&& default_value, const std::string& variable_name,
                                            const std::string& error_message)
{
  bool found_nonmatching = false;
  for (const auto& element : variable)
  {
    if (!assertion(element))
    {
      warnWithMessage(node_namespace, variable_name, default_value, error_message);
      found_nonmatching = true;
      break;
    }
  }
  if (found_nonmatching)
  {
    variable = default_value;
    checkAssertionVector(assertion, node_namespace, variable, variable_name, error_message);
  }
}

template <class T, class U, class AssertionFunction, typename igvc::disable_if<igvc::is_vector<T>::value, T>::type*>
inline void igvc::checkAssertionWithDefault(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                                            U&& default_value, const std::string& variable_name,
                                            const std::string& error_message)
{
  if (!assertion(variable))
  {
    warnWithMessage(node_namespace, variable_name, default_value, error_message);
    if (!assertion(default_value))
    {
      failWithMessage(node_namespace, "default value for " + variable_name, variable, error_message);
    }
    variable = default_value;
  }
}
template <class T, class U>
inline void igvc::checkAssertionWithDefault(Assertion assertion, const std::string& node_namespace, T& variable,
                                            U&& default_value, const std::string& variable_name)
{
  switch (assertion)
  {
    case Assertion::POSITIVE:
      assertPositiveWithDefault(node_namespace, variable, std::forward<T>(default_value), variable_name);
      assertPositive(node_namespace, variable, "default value for " + variable_name);
      break;
    case Assertion::NEGATIVE:
      assertNegativeWithDefault(node_namespace, variable, std::forward<T>(default_value), variable_name);
      assertNegative(node_namespace, variable, "default value for " + variable_name);
      break;
    case Assertion::NONE:
      break;
  }
}

template <>
inline void igvc::checkAssertionWithDefault(Assertion assertion, const std::string& node_namespace,
                                            std::string& variable, std::string&& default_value,
                                            const std::string& variable_name)
{
  if (assertion != Assertion::NONE)
  {
    ROS_WARN_STREAM("igvc::Assertion cannot be used for std::string");
  }
}

template <class T>
inline void igvc::checkAssertion(Assertion assertion, const std::string& node_namespace, T& variable,
                                 const std::string& variable_name)
{
  switch (assertion)
  {
    case Assertion::POSITIVE:
      assertPositive(node_namespace, variable, variable_name);
      break;
    case Assertion::NEGATIVE:
      assertNegative(node_namespace, variable, variable_name);
      break;
    case Assertion::NONE:
      break;
  }
}

template <>
inline void igvc::checkAssertion(Assertion assertion, const std::string& node_namespace, std::string& variable,
                                 const std::string& variable_name)
{
  if (assertion != Assertion::NONE)
  {
    ROS_WARN_STREAM("igvc::Assertion cannot be used for std::string");
  }
}

template <class T, class AssertionFunction, typename igvc::disable_if<igvc::is_vector<T>::value, T>::type*>
inline void igvc::checkAssertion(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                                 const std::string& variable_name, const std::string& error_message)
{
  if (!assertion(variable))
  {
    failWithMessage(node_namespace, variable_name, variable, error_message);
  }
}

template <class T, class AssertionFunction, typename std::enable_if<igvc::is_vector<T>::value, T>::type*>
inline void igvc::checkAssertion(AssertionFunction assertion, const std::string& node_namespace, T& variable,
                                 const std::string& variable_name, const std::string& error_message)
{
  checkAssertionVector(assertion, node_namespace, variable, variable_name, error_message);
}

template <class T, class AssertionFunction>
inline void igvc::checkAssertionVector(AssertionFunction assertion, const std::string& node_namespace,
                                       std::vector<T>& variable, const std::string& variable_name,
                                       const std::string& error_message)
{
  for (const T& element : variable)
  {
    if (!assertion(element))
    {
      failWithMessage(node_namespace, variable_name, element, variable, error_message);
    }
  }
}

template <class T, class U>
inline void igvc::assertWithDefault(const std::string& node_namespace, std::vector<T>& variable,
                                    std::function<bool(T&&)> lambda, std::vector<U>&& default_value,
                                    const std::string& message, const std::string& condition_string)
{
  for (const T& element : variable)
  {
    if (!lambda(element))
    {
      ROS_WARN_STREAM(message << element << condition_string << " Setting to default value [" << toString(default_value)
                              << "].");
      variable = default_value;
    }
  }
}

template <class T>
inline void igvc::warnWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                  const T& variable, const std::string& message)
{
  ROS_WARN_STREAM("[" << node_namespace << "] " << variable_name << message << ". Continuing with default values "
                      << variable);
}

template <class T>
inline void igvc::warnWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                  const std::vector<T>& variable, const std::string& message)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "]" << variable_name << message << ". Continuing with default values [";
  ss << toString(variable) << "].";
  ROS_WARN_STREAM(ss.str());
}

template <class T>
inline void igvc::failWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                  const T& variable, const std::string& message)
{
  ROS_ERROR_STREAM("[" << node_namespace << "] " << variable_name << " (" << variable << ") should be " << message
                       << " Exiting...");
  ros::shutdown();
}

template <class T>
inline void igvc::failWithMessage(const std::string& node_namespace, const std::string& variable_name,
                                  const std::vector<T>& variable, const std::string& message)
{
  ROS_ERROR_STREAM("[" << node_namespace << "] " << variable_name << " (" << toString(variable) << ") should be "
                       << message << " Exiting...");
  ros::shutdown();
}

template <class T>
inline void igvc::failWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& element,
                                  const std::vector<T>& variable, const std::string& message)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] " << variable_name << " ([";
  ss << toString(variable) << "]) includes element " << element << " which should be " << message << " Exiting...";
  ROS_WARN_STREAM(ss.str());
  ros::shutdown();
}

template <class T, class U, typename... Ts>
inline void igvc::assertWithDefault(const std::string& node_namespace, T& variable, bool condition, U&& default_value,
                                    const std::string& message, Ts... ts)
{
  if (!condition)
  {
    ROS_WARN(message.c_str(), node_namespace.c_str(), ts...);
    variable = default_value;
  }
}

template <class T, class U>
inline void igvc::assertPositiveWithDefault(const std::string& node_namespace, T& variable, U&& default_value,
                                            const std::string& variable_name)
{
  std::ostringstream message;
  message << "[" << node_namespace << "] " << variable_name << " (currently " << variable
          << ") should be greater than 0. Setting to default value of " << default_value;
  assertWithDefault(node_namespace, variable, variable > 0, std::forward<T>(default_value), message.str());
}

template <class T, class U>
inline void igvc::assertPositiveWithDefault(const std::string& node_namespace, std::vector<T>& variable,
                                            std::vector<U>&& default_value, const std::string& variable_name)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] " << variable_name << " ([";
  ss << toString(variable) << "]) includes element ";
  assertWithDefault(node_namespace, variable, [](T x) { return x > 0; }, std::forward<std::vector<T>>(default_value),
                    ss.str(), " greater than 0.");
}

template <class T, class U>
inline void igvc::assertNegativeWithDefault(const std::string& node_namespace, T& variable, U&& default_value,
                                            const std::string& variable_name)
{
  std::ostringstream message;
  message << "[" << node_namespace << "] " << variable_name << " (currently " << variable
          << ") should be less than 0. Setting to default value of " << default_value;
  assertWithDefault(node_namespace, variable, variable < 0, std::forward<T>(default_value), message.str());
}

template <class T, class U>
inline void igvc::assertNegativeWithDefault(const std::string& node_namespace, std::vector<T>& variable,
                                            std::vector<U>&& default_value, const std::string& variable_name)
{
  std::ostringstream ss;
  ss << "[" << node_namespace << "] " << variable_name << " ([";
  ss << toString(variable) << "]) includes element ";
  assertWithDefault(node_namespace, variable, [](T x) { return x < 0; }, std::forward<std::vector<T>>(default_value),
                    ss.str(), " less than 0.");
}

template <class T>
inline void igvc::assertPositive(const std::string& node_namespace, const T& variable, const std::string& variable_name)
{
  if (!(variable > 0))
  {
    failWithMessage(node_namespace, variable_name, variable, "greater than 0.");
  }
}

template <class T>
inline void igvc::assertPositive(const std::string& node_namespace, const std::vector<T>& variable,
                                 const std::string& variable_name)
{
  for (const T& element : variable)
  {
    if (!(element > 0))
    {
      failWithMessage(node_namespace, variable_name, element, variable, "greater than 0.");
    }
  }
}

template <class T>
inline void igvc::assertNegative(const std::string& node_namespace, const std::vector<T>& variable,
                                 const std::string& variable_name)
{
  for (const T& element : variable)
  {
    if (!(element < 0))
    {
      failWithMessage(node_namespace, variable_name, element, variable, "less than 0.");
    }
  }
}

template <class T>
inline void igvc::assertNegative(const std::string& node_namespace, const T& variable, const std::string& variable_name)
{
  if (!(variable < 0))
  {
    failWithMessage(node_namespace, variable_name, variable, "less than 0.");
  }
}

template <class T>
inline std::string igvc::toString(const std::vector<T>& v)
{
  std::stringstream ss;
  for (size_t i = 0; i < v.size(); ++i)
  {
    if (i != 0)
      ss << ",";
    ss << v[i];
  }
  return ss.str();
}
#endif
