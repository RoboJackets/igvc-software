#ifndef FLPRIOTITYQUEUE_H
#define FLPRIOTITYQUEUE_H


#include "opencv2/features2d/features2d.hpp"
#include <queue>


using namespace std;
using namespace cv;

class HarrisScore
{
  public:
    HarrisScore() : _score(0), _row(0), _col(0)
    {
    }

    HarrisScore(double score, int row, int col) : _score(score), _row(row), _col(col)
    {
    }

    double score()
    {
      return _score;
    }

    int row()
    {
      return _row;
    }
    int col()
    {
      return _col;
    }

    friend bool operator>(HarrisScore score1, HarrisScore score2)
    {
      return score1.score() > score2.score();
    }

    bool operator>(HarrisScore otherScore)
    {
      return score() > otherScore.score();
    }

    unsigned int operator-(HarrisScore& otherScore)
    {
      return score()-otherScore.score();
    }


    friend bool operator<(HarrisScore& score1, HarrisScore& score2)
    {
      return score1.score() < score2.score();
    }

    friend unsigned int operator+(HarrisScore& score1, HarrisScore& score2)
    {
      return score1.score() + score2.score();
    }

    friend ostream& operator<< (ostream &out, HarrisScore& score)
    {
      out << "Score: " << score.score() << ". Row: " << score.row() << ". Col: " << score.col() << "." << std::endl;
      return out;
    }

    static void toKeyPointArray(HarrisScore* scores, int numEl, KeyPoint* points, int diameter=8)
    {
      KeyPoint nextPoint;
      for (int i =0;i<numEl;i++)
      {
        nextPoint = KeyPoint(scores[i].col(), scores[i].row(), 8);
        points[i] = nextPoint;
      }
    }

    static void toKeyPointVector(HarrisScore* scores, int numEl, Vector<KeyPoint> points, int diameter=8)
    {
      KeyPoint nextPoint;
      for (int i =0;i<numEl;i++)
      {
        nextPoint = KeyPoint(scores[i].col(), scores[i].row(), 8);
        points.push_back(nextPoint);
      }
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
    void unload(dataType* array);
    virtual ~FLPriotityQueue();

    friend ostream& operator<< (ostream &out, FLPriotityQueue<dataType, comparison>& queue)
    {
      int length=queue.size();
      dataType array[length];
      int i = 0;
      for(i=0;i<length;i++)
      {
        queue.pop(array[i]);
        cout << array[i];
      }

      for(i=0;i<length;i++)
      {
        queue.push(array[i]);
      }
    cout.flush();
    }


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
void FLPriotityQueue<dataType, comparison>::unload(dataType* array)
{
  for(int i =0;i<size();i++)
  {
    array[i] = pop();
  }
}


template <typename dataType, typename comparison>
FLPriotityQueue<dataType, comparison>::~FLPriotityQueue()
{
  //dtor
}





#endif // FLPRIOTITYQUEUE_H
