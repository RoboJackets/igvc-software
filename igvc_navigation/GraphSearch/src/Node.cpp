#include "Node.h"

Node::Node(bool valid)
{
  this->valid = valid;
}

Node::Node(int x, int y)
{
  this->x = x;
  this->y = y;
  this->ind = std::make_tuple(x, y);
}
Node::Node(std::tuple<int, int> ind) : Node(std::get<0>(ind), std::get<1>(ind))
{
}

Node::~Node()
{
}

void Node::setIndex(int x, int y)
{
  this->x = x;
  this->y = y;
  this->ind = std::make_tuple(this->x, this->y);
}

void Node::setIndex(std::tuple<int, int> ind)
{
  this->setIndex(std::get<0>(ind), std::get<1>(ind));
}

std::tuple<int, int> Node::getIndex() const
{
  return this->ind;
}

void Node::setBptr(std::tuple<int, int> bptr)
{
  this->bptr = bptr;
}

std::tuple<int, int> Node::getBptr() const
{
  return this->bptr;
}

bool Node::operator==(const Node& other) const
{
  return this->getIndex() == other.getIndex();
}

bool Node::operator!=(const Node& other) const
{
  return !(*this == other);
}

Node& Node::operator=(const Node& node)
{
  // do the copy
  std::tie(this->x, this->y) = node.getIndex();
  this->ind = node.getIndex();
  this->bptr = node.getBptr();

  // return the existing object so we can chain this operator
  return *this;
}

std::ostream& operator<<(std::ostream& stream, const Node& n)
{
  int x, y;
  std::tie(x, y) = n.getIndex();
  stream << "[" << x << ", " << y << "]";
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const std::unordered_set<Node>& uset)
{
  stream << "{";
  for (auto const& i : uset)
  {
    stream << i << ", ";
  }
  stream << "}";

  return stream;
}
