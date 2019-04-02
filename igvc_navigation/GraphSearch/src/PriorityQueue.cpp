#include "PriorityQueue.h"

PriorityQueue::PriorityQueue() : pq_(compFunctor)
{
}

bool PriorityQueue::contains(Node n)
{
  return this->find(n) != pq_.end();
}

void PriorityQueue::insert(Node n, Key k)
{
  pq_.insert(std::make_pair(n, k));
}

void PriorityQueue::clear()
{
  pq_.clear();
}

bool PriorityQueue::remove(Node n)
{
  auto it = this->find(n);

  if (it == pq_.end())  // no such item exists
    return false;

  pq_.erase(it);
  return true;
}

void PriorityQueue::pop()
{
  if (this->size() <= 0)
    return;

  pq_.erase(pq_.begin());
}

Key PriorityQueue::topKey()
{
  std::pair<Node, Key> e = *(pq_.begin());
  return e.second;
}

Node PriorityQueue::topNode()
{
  std::pair<Node, Key> e = *(pq_.begin());
  return e.first;
}

int PriorityQueue::size()
{
  return pq_.size();
}

bool PriorityQueue::empty()
{
  return pq_.empty();
}

std::string PriorityQueue::str() const
{
  std::stringstream ss;
  ss << "{";
  for (auto it = pq_.begin(); it != pq_.end(); it++)
  {
    ss << "<" << it->first << ", " << it->second << ">";

    // add commas and new lines responsibly
    if ((it != pq_.end()) && (std::next(it) != pq_.end()))
      ss << ",\n";
  }
  ss << "}";

  return ss.str();
}

std::set<std::pair<Node, Key>>::iterator PriorityQueue::find(Node n)
{
  std::set<std::pair<Node, Key>>::iterator it = std::find_if(
      pq_.begin(), pq_.end(), [n](std::pair<Node, Key> const& e) { return e.first.getIndex() == n.getIndex(); });

  return it;
}

std::ostream& operator<<(std::ostream& stream, const PriorityQueue& pq)
{
  stream << pq.str();
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const Key& k)
{
  stream << "[" << k.f1 << ", " << k.f2 << "]";
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const std::pair<Node, Key>& e)
{
  stream << "<" << e.first << ", " << e.second << ">";
  return stream;
}
