#ifndef STRINGUTILS_HPP_INCLUDED
#define STRINGUTILS_HPP_INCLUDED

#include <string>
#include <vector>

extern std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

extern std::vector<std::string> split(const std::string &s, char delim);

#endif // STRINGUTILS_HPP_INCLUDED
