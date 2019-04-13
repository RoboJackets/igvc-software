#include <igvc_navigation/fast_sweep.h>
#include <limits>

using namespace fast_sweep;

FastSweep::FastSweep(int rows, int cols)
  : rows_{ rows }, cols_{ cols }, grid_(rows * cols, std::numeric_limits<float>::max())
{
}

const std::vector<float> FastSweep::solveEikonal(const std::vector<Node>& gamma_points)
{
  // 1. Setup grid
  // 1.1: Clear grid
  resetGrid();

  // 1.2: Set gamma points to 0
  setupGamma(gamma_points);

  // Get bands
  std::vector<Sweep> sweeps = getSweeps();

  // Iterate over bands
  doSweeps(sweeps);

  // Return grid_
  return std::vector<float>{};
}

void FastSweep::resetGrid()
{
  std::fill(grid_.begin(), grid_.end(), std::numeric_limits<float>::max());
}

void FastSweep::setupGamma(const std::vector<Node>& gamma_points)
{
  for (const auto& gamma : gamma_points)
  {
    getCell({ gamma.x, gamma.y }) = 0.0f;
  }
}

float& FastSweep::getCell(const Node& node)
{
  return grid_[getIndex(node)];
}

const float FastSweep::getCell(const Node& node) const
{
  return grid_[getIndex(node)];
}

float FastSweep::getEikonal(const Node& node) const
{
  float left = std::numeric_limits<float>::max();
  float right = std::numeric_limits<float>::max();
  float up = std::numeric_limits<float>::max();
  float down = std::numeric_limits<float>::max();

  if (node.left().isValid(rows_, cols_))
  {
    left = getCell(node.left());
  }
  if (node.right().isValid(rows_, cols_))
  {
    right = getCell(node.right());
  }
  if (node.up().isValid(rows_, cols_))
  {
    up = getCell(node.up());
  }
  if (node.right().isValid(rows_, cols_))
  {
    down = getCell(node.up());
  }

  float x_min = std::min(left, right);
  float y_min = std::min(up, down);
  float x_bar = std::min(x_min, y_min) + resolution_ * getCost(node);

  return x_bar;
}

const float FastSweep::getCost(const Node& node) const
{
  return costs_[getIndex(node)];
}

int FastSweep::getIndex(const Node& node) const
{
  return node.y * rows_ + node.x;
}

void FastSweep::doSweeps(const std::vector<fast_sweep::Sweep>& sweeps)
{
  for (auto sweep : sweeps)
  {
    for (const auto& node : sweep)
    {
      float x_bar = getEikonal(node);
      getCell(node) = std::min(x_bar, getCell(node));
    }
  }
}

std::vector<Sweep> FastSweep::getSweeps() const
{
  std::vector<Sweep> sweeps;
  sweeps.reserve(4 * (rows_ + cols_ - 1));  // w + h - 1 for each, four directions in total

  // (1,1) /. Start at (0, 0)
  // 0 / / / /
  // 1 / / / /
  // 2 3 4 5 6
  for (int i = 0; i < rows_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), i + 1);
    sweeps.emplace_back(Sweep{ { 0, i }, iterations, 1, 1 });
  }
  for (int i = 1; i < cols_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), cols_ - i);
    sweeps.emplace_back(Sweep{ { i, rows_ - 1 }, iterations, -1, -1 });
  }

  // (-1,-1) /. Start at (0, 0)
  // 0 1 2 3 4
  // / / / / 5
  // / / / / 6
  for (int i = 0; i < cols_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), i + 1);
    sweeps.emplace_back(Sweep{ { i, 0 }, iterations, -1, -1 });
  }
  for (int i = 1; i < rows_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), rows_ - i);
    sweeps.emplace_back(Sweep{ { i, cols_ - 1 }, iterations, -1, -1 });
  }

  // (1,-1) \. Start at (cols, 0)
  // 4 3 2 1 0.
  // 5 \ \ \ \.
  // 6 \ \ \ \.
  for (int i = 0; i < cols_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), i + 1);
    sweeps.emplace_back(Sweep{ { 0, cols_ - 1 - i }, iterations, 1, -1 });
  }
  for (int i = 1; i < rows_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), rows_ - i);
    sweeps.emplace_back(Sweep{ { i, 0 }, iterations, 1, -1 });
  }

  // (-1,1) \. Start at (0, rows)
  // \ \ \ \ 6
  // \ \ \ \ 5
  // 0 1 2 3 4
  for (int i = 0; i < cols_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), i + 1);
    sweeps.emplace_back(Sweep{ { i, rows_ - 1 }, iterations, -1, 1 });
  }
  for (int i = 1; i < rows_; i++)
  {
    int iterations = std::min(std::min(rows_, cols_), rows_ - i);
    sweeps.emplace_back(Sweep{ { rows_ - 1 - i, cols_ - 1 }, iterations, -1, 1 });
  }
  return sweeps;
}

Node Node::left() const
{
  return { x - 1, y };
}
Node Node::right() const
{
  return { x + 1, y };
}
Node Node::up() const
{
  return { x, y - 1 };
}
Node Node::down() const
{
  return { x, y + 1 };
}

void Node::move(int dx, int dy)
{
  x += dx;
  y += dy;
}

bool Node::isValid(int rows, int cols) const
{
  return x >= 0 && x < rows && y >= 0 && y < cols;
}

bool Node::operator==(const fast_sweep::Node& rhs) const
{
  return x == rhs.x && y == rhs.y;
}

bool Node::operator!=(const fast_sweep::Node& rhs) const
{
  return x != rhs.x || y != rhs.y;
}

SweepIterator Sweep::begin()
{
  return SweepIterator(*this);
}

SweepIterator Sweep::end()
{
  return {};
}

SweepIterator::SweepIterator(Sweep sweep) : sweep_{ sweep }
{
}

SweepIterator::SweepIterator() : sweep_{ { 0, 0 }, 0, 0, 0 }
{
}

const Node& SweepIterator::operator*() const
{
  return sweep_.start_node;
}

const Node& SweepIterator::operator->() const
{
  return sweep_.start_node;
}

SweepIterator& SweepIterator::operator++()
{
  sweep_.start_node.move(sweep_.dx, sweep_.dy);
  sweep_.iterations--;
  return *this;
}

const SweepIterator SweepIterator::operator++(int)
{
  SweepIterator old{ *this };
  sweep_.start_node.move(sweep_.dx, sweep_.dy);
  sweep_.iterations--;
  return old;
}

bool SweepIterator::operator==(const SweepIterator& rhs) const
{
  return sweep_.start_node == rhs.sweep_.start_node && sweep_.dx == rhs.sweep_.dx && sweep_.dy == rhs.sweep_.dy;
}

bool SweepIterator::operator!=(const SweepIterator& rhs) const
{
  return sweep_.iterations == rhs.sweep_.iterations;
}
