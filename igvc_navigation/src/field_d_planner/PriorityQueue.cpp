#include "PriorityQueue.h"

bool PriorityQueue::remove(std::string s)
{
    for (auto it = this->c.begin(); it != this->c.end(); it ++ )
    {
        if ((it->key).compare(s) == 0)
        {
            this->c.erase(it);
            return true;
        }
    }

    return false;
}

std::string PriorityQueue::toString()
{
    std::stringstream ss;
    for (auto it = this->c.begin(); it != this->c.end(); it ++ )
    {
        ss << it->key << " [ f: "<< std::get<0>(it->value) << ", g: " << std::get<1>(it->value) << "]\n";
    }

    return ss.str();
}

bool PriorityQueue::update(Item i)
{
    bool success = this->remove(i.key);
    if (success)
        this->push(i);

    return success;
}

bool PriorityQueue::contains(std::string key)
{
    for (auto it = this->c.begin(); it != this->c.end(); it ++ )
    {
        if ((it->key).compare(key) == 0)
        {
            return true;
        }
    }

    return false;

}
