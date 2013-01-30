#ifndef DATAARRAY_H
#define DATAARRAY_H
#include <boost/circular_buffer.hpp>

template <class DataType>
class DataArray
{
    public:
        DataArray(int);
        void push(DataType);
        int size();
        DataType operator[](int);
        virtual ~DataArray();

    private:
        int firstIndBefore(long long);
        int firstIndAfter(long long);
        boost::circular_buffer<DataType> buff;
        int length;
};


template<class DataType>
DataArray<DataType>::DataArray(int NumEl=100) : buff(boost::circular_buffer<DataType>(NumEl)), length(NumEl)
{
}

template<class DataType>
void DataArray<DataType>::push(DataType newData)
{
    buff.push_front(newData);
}

template<class DataType>
int DataArray<DataType>::size(void)
{
    return buff.size();
}

/***
* This function makes the asssumptions that the array is a chronologically ordered with the newest time being first
***/
template<class DataType>
int DataArray<DataType>::firstIndBefore(long long aTime)
//TODO Reconsider the return values of this function.
{
    int size = buff.size();
    if (size == 0)
    {
        return -1; //buffer is empty, no valid answer
    }

    for (int i =0;i<size;i++)
    {
        if (aTime > buff[i].time)
        {
            return i; //Handles situations where time is later than first entry or between two existing ones.
        }
    }

    return -1; //All entries are after the queried time
}

template<class DataType>
int DataArray<DataType>::firstIndAfter(long long aTime)
//TODO Reconsider the return values of this function.
//TODO Verify logic for this function, it was a quick port from firstIndAfter
{
    int size = buff.size();
    if (size == 0)
    {
        return -1; //buffer is empty, no valid answer
    }

    for (int i =size-1;i>-1;i--)
    {
        if( aTime > buff[i].time)
        {
            return i; //Handles situations where time is later than first entry or between two existing ones.
        }
    }

    return -1; //All entries are before the queried time
}


template<class DataType>
DataType DataArray<DataType>::operator[] (int subscript)
{
    return buff[subscript];
}

template<class DataType>
DataArray<DataType>::~DataArray()
{
    //TODO - find out if you need to destruct buff
}


#endif // DATAARRAY_H
