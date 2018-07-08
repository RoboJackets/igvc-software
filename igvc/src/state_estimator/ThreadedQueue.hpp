#include <condition_variable>
#include <mutex>
#include <queue>

template <class T>
class ThreadedQueue
{
private:
  std::queue<T> q;
  mutable std::mutex m;
  std::condition_variable c;

public:
  ThreadedQueue() : q(), m(), c()
  {
  }

  ~ThreadedQueue()
  {
  }

  T front()
  {
    std::lock_guard<std::mutex> lock(m);
    return q.front();
  }

  T back()
  {
    std::lock_guard<std::mutex> lock(m);
    return q.back();
  }

  void push(T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q.push(t);
    c.notify_one();
  }

  T pop()
  {
    // blocks until an element is available
    std::unique_lock<std::mutex> lock(m);
    c.wait(lock, [&] { return !q.empty(); });
    T t = q.front();
    q.pop();
    return t;
  }

  size_t size()
  {
    std::lock_guard<std::mutex> lock(m);
    return q.size();
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(m);
    return q.empty();
  }
};
