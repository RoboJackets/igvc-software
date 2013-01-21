/*
#include "DataArray.h"
//#include <boost/circular_buffer.hpp>

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
    ~buff;
}
*/
