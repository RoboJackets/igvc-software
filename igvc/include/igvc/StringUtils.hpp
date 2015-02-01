#pragma once
#include <vector>
#include <sstream>

std::vector<std::string> split(const std::string &s, const char &delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
