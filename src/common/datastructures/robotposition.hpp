#ifndef ROBOTPOSITION_HPP
#define ROBOTPOSITION_HPP

/**
 * @brief Represents the position of the robot.
 */
class RobotPosition
{
public:

    /**
     * @brief The horizontal component of the robot's positon
     */
    double X;

    /**
     * @brief The vertical component of the robot's position. Positive Y points North
     */
    double Y;

    /**
     * @brief The direction the robot is facing. (Degrees East of North, ie - clockwise)
     */
    double Heading;
};

#endif // ROBOTPOSITION_HPP
