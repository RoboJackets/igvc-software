#ifndef SRC_FAST_SWEEP_H
#define SRC_FAST_SWEEP_H

#include <iostream>
#include <vector>

namespace fast_sweep
{
struct Node
{
  int x;
  int y;

public:
  Node left() const;
  Node right() const;
  Node up() const;
  Node down() const;

  void move(int dx, int dy);
  bool isValid(int rows, int cols) const;

  bool operator==(const Node& rhs) const;
  bool operator!=(const Node& rhs) const;

  friend std::ostream& operator<<(std::ostream& os, const Node& node);
};

struct SweepIterator;
struct Sweep
{
  Node start_node;
  int iterations;
  int dx;
  int dy;

public:
  SweepIterator begin();
  SweepIterator end();
  friend std::ostream& operator<<(std::ostream& os, const Sweep& sweep);
};

struct SweepIterator
{
  using iterator_category = std::forward_iterator_tag;
  using value_type = Node;
  using difference_type = void;
  using pointer = void;
  using reference = void;

  explicit SweepIterator(Sweep sweep);

public:
  explicit SweepIterator();

  // Forward iterator requirements
  const Node& operator*() const;
  const Node& operator->() const;
  SweepIterator& operator++();          // Pre
  const SweepIterator operator++(int);  // Post

  bool operator==(const SweepIterator& rhs) const;
  bool operator!=(const SweepIterator& rhs) const;

private:
  Sweep sweep_;
};

class FastSweep
{
public:
  FastSweep(int rows, int cols, float resolution);

  /**
   * Solve the eikonal equation using the given gamma (source) points used for the solver
   * @param gamma_points vector of Nodes that represent a gamma point
   */
  const std::vector<float> solveEikonal(const std::vector<Node>& gamma_points, const std::vector<float>& costs);
  void printGrid() const;

private:
  void resetGrid();
  void setupGamma(const std::vector<Node>& gamma_points);
  float& getCell(const Node& node);
  const float getCell(const Node& node) const;
  const float getCost(const Node& node) const;
  int getIndex(const Node& node) const;
  float getEikonal(const Node& node) const;

  std::vector<Sweep> getSweeps() const;
  void doSweeps(const std::vector<Sweep>& sweeps);

  int rows_;
  int cols_;
  float resolution_;
  std::vector<float> grid_;
  std::vector<float> costs_;
};
std::ostream& operator<<(std::ostream& os, const Node& node);
std::ostream& operator<<(std::ostream& os, const Sweep& sweep);
}
#endif  // SRC_FAST_SWEEP_H
