#ifndef PLAYERERROR_H
#define PLAYERERROR_H

#include <string>
#include <iostream>

namespace PlayerCc
{

/** @brief The C++ exception class
 *
 * When @em libplayerc++ receives an error from @em libplayerc
 * it throws a PlayerError exception.
 */
class PlayerError
{
  private:

    // a string describing the error
    std::string mStr;
    // a string describing the location of the error in the source
    std::string mFun;
    // error code returned by playerc
    int mCode;

  public:
    /// the error string
    std::string GetErrorStr() const { return(mStr); };
    /// the function that threw the error
    std::string GetErrorFun() const { return(mFun); };
    /// a numerical error code
    int GetErrorCode() const { return(mCode); };

    /// default constructor
    PlayerError(const std::string aFun="",
                const std::string aStr="",
                const int aCode=-1);
    /// default destructor
    ~PlayerError();
};

}

namespace std
{
std::ostream& operator << (std::ostream& os, const PlayerCc::PlayerError& e);
}

#endif
