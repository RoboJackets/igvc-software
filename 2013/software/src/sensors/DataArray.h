#ifndef DATAARRAY_H
#define DATAARRAY_H
#include <boost/circular_buffer.hpp>

template <class DataType>
class DataArray
{
    public:
        DataArray(int);
        void push(DataType);
        DataType operator[](int);
        virtual ~DataArray();

    private:
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
