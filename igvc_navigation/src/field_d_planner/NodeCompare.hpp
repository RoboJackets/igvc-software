/**
ItemCompate is a custom comparator (java name) that is passed as a type
parameter upon the construction of a std::priority_queue
*/

#include <tuple>
#include "Node.h"

class NodeCompare
{
public:
    bool operator() (const Item& lhs, const Item& rhs) const
    {

        if (std::get<0>(lhs.value) > std::get<0>(rhs.value))
        {
            return true;
        }
        else if (std::get<0>(lhs.value) == std::get<0>(rhs.value))
        {
            return (std::get<1>(lhs.value) >= std::get<1>(rhs.value));
        }
        else
        {
            return false;
        }
    }
};
