#ifndef FLPRIOTITYQUEUE_H
#define FLPRIOTITYQUEUE_H


#include <queue>


using namespace std;

class HarrisScore
{
  public:
    inline HarrisScore(double score, int row, int col) : _score(score), _row(row), _col(col)
    {
    }

    inline double score()
    {
      return _score;
    }

    inline int row()
    {
      return _row;
    }
    inline int col()
    {
      return _col;
    }

    inline friend bool operator>(HarrisScore score1, HarrisScore score2)
    {
      return score1.score() > score2.score();
    }

    inline bool operator>(HarrisScore otherScore)
    {
      return score() > otherScore.score();
    }

    inline unsigned int operator-(HarrisScore& otherScore)
    {
      return score()-otherScore.score();
    }


    inline friend bool operator<(HarrisScore& score1, HarrisScore& score2)
    {
      return score1.score() < score2.score();
    }

    inline friend unsigned int operator+(HarrisScore& score1, HarrisScore& score2)
    {
      return score1.score() + score2.score();
    }

  private:
    double _score;
    int _row;
    int _col;
};


template <typename dataType, typename comparison>
class FLPriotityQueue
{
  public:
    FLPriotityQueue(int length);
    bool empty();
    unsigned int size();
    dataType top();
    bool push(dataType);
    bool pop(dataType&);
    virtual ~FLPriotityQueue();
  protected:
  private:
      //priority_queue< int, vector<int>, greater<int> > data;
      priority_queue< dataType, vector<dataType>, comparison> _data;
      unsigned int _sizeLimit;
};




template <typename dataType, typename comparison>
FLPriotityQueue<dataType, comparison>::FLPriotityQueue(int length) : _sizeLimit(length)
{

}

template <typename dataType, typename comparison>
bool FLPriotityQueue<dataType, comparison>::empty()
{
  return _data.empty();
}

template <typename dataType, typename comparison>
unsigned int FLPriotityQueue<dataType, comparison>::size()
{
  return _data.size();
}

template <typename dataType, typename comparison>
dataType FLPriotityQueue<dataType, comparison>::top()
{
  return _data.top();
}


template <typename dataType, typename comparison>
bool FLPriotityQueue<dataType, comparison>::pop(dataType& returnVal)
{
  if (!_data.empty())
  {
    returnVal = _data.top();
    _data.pop();
    return true;
  }
  else
  {
    return false;
  }
}

template <typename dataType, typename comparison>
bool FLPriotityQueue<dataType, comparison>::push( const dataType x)
{
  if (size() < _sizeLimit)
  {
    _data.push(x);
    return false;
  }
  else
  {
    _data.push(x);
    _data.pop();
    return true;
  }
}

template <typename dataType, typename comparison>
FLPriotityQueue<dataType, comparison>::~FLPriotityQueue()
{
  //dtor
}

#endif // FLPRIOTITYQUEUE_H
