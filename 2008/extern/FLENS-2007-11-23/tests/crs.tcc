#include <cassert>

namespace flens {

//-- CRS -----------------------------------------------------------------------

template <typename T, CRS_Storage Storage>
CRS<T, Storage>::CRS(int numRows, int numCols, int approxBandwidth)
    : rows(numRows+1), _numRows(numRows), _numCols(numCols), _k(approxBandwidth)
{
}

template <typename T, CRS_Storage Storage>
CRS<T, Storage>::~CRS()
{
}

template <typename T, CRS_Storage Storage>
typename CRS<T, Storage>::Initializer *
CRS<T, Storage>::initializer()
{
    return new Initializer(*this, _k);
}

template <typename T, CRS_Storage Storage>
void
CRS<T, Storage>::allocate(int numNonZeros)
{
    values.resize(numNonZeros);
    columns.resize(numNonZeros);

    rows(1) = 1;
    rows(_numRows+1) = numNonZeros+1;
}

template <typename T, CRS_Storage Storage>
int
CRS<T, Storage>::numRows() const
{
    return _numRows;
}

template <typename T, CRS_Storage Storage>
int
CRS<T, Storage>::numCols() const
{
    return _numCols;
}

template <typename T, CRS_Storage Storage>
int
CRS<T, Storage>::numNonZeros() const
{
    return rows(_numRows+1)-rows(1);
}

template <typename T, CRS_Storage Storage>
typename CRS<T, Storage>::const_iterator
CRS<T, Storage>::begin() const
{
    return const_iterator(*this, 1);
}

template <typename T, CRS_Storage Storage>
typename CRS<T, Storage>::iterator
CRS<T, Storage>::begin()
{
    return iterator(*this, 1);
}

template <typename T, CRS_Storage Storage>
typename CRS<T, Storage>::const_iterator
CRS<T, Storage>::end() const
{
    return const_iterator(*this, numNonZeros()+1);
}

template <typename T, CRS_Storage Storage>
typename CRS<T, Storage>::iterator
CRS<T, Storage>::end()
{
    return iterator(*this, numNonZeros()+1);
}

//-- CRS_Initializer -----------------------------------------------------------

template <typename T, CRS_Storage Storage>
CRS_Initializer<T, Storage>::CRS_Initializer(CRS<T, Storage> &crs)
    : _crs(crs)
{
    if (Storage!=CRS_General) {
        // store diagonal elements
        std::pair<int,int> key;
        for (int i=1; i<=crs.numRows(); ++i) {
            key.first = key.second = i;
            _map[key] = T(0);
        }
    }
}

template <typename T, CRS_Storage Storage>
CRS_Initializer<T, Storage>::~CRS_Initializer()
{
    typedef typename CRS_Map::const_iterator It;

    _crs.allocate(_map.size());
    int valueIndex=1,
        row=1;
    for (It it=_map.begin(); it!=_map.end(); ++it) {
        // A(row, col) = A(it->first.first, it->first.second)
        //             = it->second
        _crs.values(valueIndex) =  it->second;
        _crs.columns(valueIndex) = it->first.second;
        if (it->first.first>row) {
            assert(it->first.first==row+1); // no empty rows allowed

            row = it->first.first;
            _crs.rows(row) = valueIndex;
        }
        ++valueIndex;
    }
    _crs.rows(row+1) = valueIndex;
}

template <typename T, CRS_Storage Storage>
T &
CRS_Initializer<T, Storage>::operator()(int row, int col)
{
    assert(row>=1);
    assert(row<=_crs.numRows());
    assert(col>=1);
    assert(col<=_crs.numCols());

    std::pair<int,int> key;

    if (Storage==CRS_General) {
        key.first = row;
        key.second = col;
    }
    if (Storage==CRS_UpperTriangular) {
        key.first = std::min(row, col);
        key.second = std::max(row, col);
    }
    if (Storage==CRS_LowerTriangular) {
        key.first = std::max(row, col);
        key.second = std::min(row, col);
    }

    return _map[key];
}

//-- CRS_RandomAccessInitializer -----------------------------------------------

template <typename T, CRS_Storage S>
CRS_RandomAccessInitializer<T, S>::CRS_RandomAccessInitializer(CRS<T, S> &crs, int k)
    : _middle(0), _insertCount(0), _crs(crs), _isSorted(true)
{
    _coordinates.reserve(k*crs.numRows());
}

template <typename T, CRS_Storage Storage>
CRS_RandomAccessInitializer<T, Storage>::~CRS_RandomAccessInitializer()
{
    sort();

    // check for empty rows and insert 0 on diagonal if needed
    int row = 1;
    std::vector<int> fillIn;
    for (size_t k=1; k<_coordinates.size(); ++k) {
        int rowDiff = _coordinates[k].row - row;
        for (int r=1; r<rowDiff; ++r) {
            fillIn.push_back(row+r);
        }
        row += rowDiff;
    }
    if (fillIn.size()>0) {
        for (size_t k=0; k<fillIn.size(); ++k) {
#ifdef DEBUG
            std::cerr << "fill in ("
                      << fillIn[k] << ", " 
                      << fillIn[k] << ")"
                      << std::endl;
#endif // DEBUG
            operator()(fillIn[k], fillIn[k]) = T(0);
        }
        sort();
    }
    _crs.allocate(_coordinates.size());

    row = 1;
    for (size_t k=0; k<_coordinates.size(); ++k) {
        if (_coordinates[k].row>row) {
            _crs.rows(row+1) = k+1;
            row = _coordinates[k].row;
        }
        _crs.columns(k+1) = _coordinates[k].col;
        _crs.values(k+1) = _coordinates[k].value;
    }
    _crs.rows(row+1) = _coordinates.size()+1;
}

template <typename T, CRS_Storage Storage>
void
CRS_RandomAccessInitializer<T, Storage>::sort()
{
#ifdef DEBUG
    std::cerr << "before:" << std::endl;
    for (size_t k=0; k<_coordinates.size(); ++k) {
        std::cerr << "[" << k << "] "
                  << " (" << _coordinates[k].row
                  << ", " << _coordinates[k].col
                  << ") = " << _coordinates[k].value
                  << std::endl;
    }
#endif // DEBUG

    // sort
    if (!_isSorted) {
        std::sort(_coordinates.begin()+_middle, _coordinates.end(), _less);
#ifdef DEBUG
        std::cerr << "partial sorted:" << std::endl;
        for (size_t k=0; k<_coordinates.size(); ++k) {
            std::cerr << "[" << k << "] "
                      << " (" << _coordinates[k].row
                      << ", " << _coordinates[k].col
                      << ") = " << _coordinates[k].value
                      << std::endl;
        }
#endif // DEBUG
    }
#ifdef DEBUG
        std::cerr << "middle = " << _middle << std::endl;
        std::cerr << "size = " << _coordinates.size() << std::endl;
#endif // DEBUG
    if ((_middle>0) && (_middle<_coordinates.size())) {
#ifdef DEBUG
        std::cerr << "merging ..." << std::endl;
#endif // DEBUG
        std::inplace_merge(_coordinates.begin(),
                           _coordinates.begin() + _middle,
                           _coordinates.end(),
                           _less);
    }
    _isSorted = true;

#ifdef DEBUG
    std::cerr << "sorted:" << std::endl;
    for (size_t k=0; k<_coordinates.size(); ++k) {
        std::cerr << "[" << k << "] "
                  << " (" << _coordinates[k].row
                  << ", " << _coordinates[k].col
                  << ") = " << _coordinates[k].value
                  << std::endl;
    }
#endif // DEBUG

    // eliminate duplicates
    size_t i, I;
    for (i=0, I=1; I<_coordinates.size(); ++i, ++I) {
        while ((!_less(_coordinates[i], _coordinates[I]))
            && (I<_coordinates.size()))
        {
            _coordinates[i].value += _coordinates[I].value;
            _coordinates[I].value = T(0);
            ++I;
        }
        if (I<_coordinates.size()) {
            _coordinates[i+1] = _coordinates[I];
        }
    }
    if (i+1<I) {
#ifdef DEBUG
        std::cerr << "remove duplicates. i = " << i
                  << ", I = " << I << std::endl;
        for (size_t k=0; k<_coordinates.size(); ++k) {
            std::cerr << "[" << k << "] "
                      << " (" << _coordinates[k].row
                      << ", " << _coordinates[k].col
                      << ") = " << _coordinates[k].value
                      << std::endl;
        }
#endif // DEBUG

        if (i+1<_coordinates.size()) {
            _coordinates.erase(_coordinates.end()-(I-i-1), _coordinates.end());
        }
    }
    _middle = _coordinates.size();
#ifdef DEBUG
    std::cerr << "after:" << std::endl;
    for (size_t k=0; k<_coordinates.size(); ++k) {
        std::cerr << "[" << k << "] "
                  << " (" << _coordinates[k].row
                  << ", " << _coordinates[k].col
                  << ") = " << _coordinates[k].value
                  << std::endl;
    }
    std::cerr << "------------------" << std::endl << std::endl;
#endif // DEBUG
}

template <typename T, CRS_Storage Storage>
T &
CRS_RandomAccessInitializer<T, Storage>::operator()(int row, int col)
{
    assert(row>=1);
    assert(row<=_crs.numRows());
    assert(col>=1);
    assert(col<=_crs.numCols());

    int r = -1,
        c = -1;
    if (Storage==CRS_General) {
        r = row;
        c = col;
    }
    if (Storage==CRS_UpperTriangular) {
        r = std::min(row, col);
        c = std::max(row, col);
    }
    if (Storage==CRS_LowerTriangular) {
        r = std::max(row, col);
        c = std::min(row, col);
    }
    assert((r>0) && (c>0));

    if (_coordinates.size()>=_coordinates.capacity()) {
        sort();
        _coordinates.reserve(_coordinates.capacity() + _crs.numRows());
    }

    _coordinates.push_back(CRS_RAI_Coordinate<T>(r, c, T(0)));
    size_t lastIndex = _coordinates.size()-1;
    if ((lastIndex>0) && _isSorted) {
        if (_less(_coordinates[lastIndex-1], _coordinates[lastIndex])) {
            _middle = lastIndex;
        } else {
            _isSorted = false;
        }
    }
    return _coordinates[lastIndex].value;
}


//-- CRS_OrderedInitializer ----------------------------------------------------

template <typename T, CRS_Storage Storage>
CRS_OrderedInitializer<T, Storage>::CRS_OrderedInitializer(CRS<T, Storage> &crs)
    : _row(crs.numRows()), _crs(crs), _currentRow(1)
{
    _row[0] = 1;
    _col.reserve(crs.numRows());
    _value.reserve(crs.numRows());
}

template <typename T, CRS_Storage Storage>
CRS_OrderedInitializer<T, Storage>::~CRS_OrderedInitializer()
{
    _crs.allocate(_value.size());

    int nnz = _value.size();
    for (int k=1; k<=nnz; ++k) {
        _crs.values(k) = _value[k-1];
        _crs.columns(k) = _col[k-1];
    }

    int m = _crs.numRows();
    for (int k=1; k<=m; ++k) {
        _crs.rows(k) = _row[k-1];
    }
    _crs.rows(m+1) = nnz+1;
}

template <typename T, CRS_Storage Storage>
T &
CRS_OrderedInitializer<T, Storage>::operator()(int row, int col)
{
    assert(row>=1);
    assert(row<=_crs.numRows());
    assert(col>=1);
    assert(col<=_crs.numCols());

    int r = 0,
        c = 0;
    if (Storage==CRS_General) {
        r = row;
        c = col;
    }
    if (Storage==CRS_UpperTriangular) {
        r = std::min(row, col);
        c = std::max(row, col);
    }
    if (Storage==CRS_LowerTriangular) {
        r = std::max(row, col);
        c = std::min(row, col);
    }

    _col.push_back(c);
    _value.push_back(T(0));

    assert((r==_currentRow) || (r==_currentRow+1));

    if (r>_currentRow) {
        _currentRow = r;
        _row[r-1] = _col.size();
    }

    return _value[_value.size()-1];
}

//-- CRS_ConstIterator ---------------------------------------------------------

template <typename T, CRS_Storage Storage>
CRS_ConstIterator<T, Storage>::CRS_ConstIterator(const CRS_ConstIterator &rhs)
    : _crs(rhs._crs), _pos(rhs._pos)
{
    if (_pos<=_crs.numNonZeros()) {
        int col = _crs.columns(_pos);
        int row = _crs.numRows()+1;
        for (int i=1; i<=_crs.numRows(); ++i) {
            if ((_pos>=_crs.rows(i)) && (_pos<_crs.rows(i+1))) {
                row = i;
                break;
            }
        }
        _value.first = key_type(row, col);
        _value.second = _crs.values(_pos);
    }
}

template <typename T, CRS_Storage Storage>
CRS_ConstIterator<T, Storage>::CRS_ConstIterator(const
                                                 CRS_Iterator<T, Storage> &rhs)
    : _crs(rhs._crs), _pos(rhs._pos)
{
    if (_pos<=_crs.numNonZeros()) {
        int col = _crs.columns(_pos);
        int row = _crs.numRows()+1;
        for (int i=1; i<=_crs.numRows(); ++i) {
            if ((_pos>=_crs.rows(i)) && (_pos<_crs.rows(i+1))) {
                row = i;
                break;
            }
        }
        _value.first = key_type(row, col);
        _value.second = _crs.values(_pos);
    }
}

template <typename T, CRS_Storage Storage>
CRS_ConstIterator<T, Storage>::CRS_ConstIterator(const CRS<T, Storage> &crs,
                                                 int pos)
    : _crs(crs), _pos(pos)
{
    if (pos<=_crs.numNonZeros()) {
        int col = _crs.columns(_pos);
        int row = _crs.numRows()+1;
        for (int i=1; i<=_crs.numRows(); ++i) {
            if ((pos>=_crs.rows(i)) && (pos<_crs.rows(i+1))) {
                row = i;
                break;
            }
        }
        _value.first = key_type(row, col);
        _value.second = _crs.values(_pos);
    }
}

template <typename T, CRS_Storage Storage>
bool
CRS_ConstIterator<T, Storage>::operator==(const CRS_ConstIterator &rhs) const
{
    return ((&_crs==&rhs._crs) && (_pos==rhs._pos));
}

template <typename T, CRS_Storage Storage>
bool
CRS_ConstIterator<T, Storage>::operator!=(const CRS_ConstIterator &rhs) const
{
    return !(*this==rhs);
}


template <typename T, CRS_Storage Storage>
CRS_ConstIterator<T, Storage> &
CRS_ConstIterator<T, Storage>::operator++()
{
    ++_pos;

    if (_pos<=_crs.numNonZeros()) {
        if (_pos>=_crs.rows(_value.first.first+1)) {
            ++_value.first.first;
        }
        _value.first.second = _crs.columns(_pos);
        _value.second = _crs.values(_pos);
    }
    return *this;
}

template <typename T, CRS_Storage Storage>
const typename CRS_ConstIterator<T, Storage>::value_type &
CRS_ConstIterator<T, Storage>::operator*() const
{
    return _value;
}

template <typename T, CRS_Storage Storage>
const typename CRS_ConstIterator<T, Storage>::value_type *
CRS_ConstIterator<T, Storage>::operator->() const
{
    return &_value;
}

//-- CRS_Iterator --------------------------------------------------------------

template <typename T, CRS_Storage Storage>
CRS_Iterator<T, Storage>::CRS_Iterator(const CRS_Iterator &rhs)
    : _crs(rhs._crs), _pos(rhs._pos)
{
    if (_pos<=_crs.numNonZeros()) {
        int col = _crs.columns(_pos);
        int row = _crs.numRows()+1;
        for (int i=1; i<=_crs.numRows(); ++i) {
            if ((_pos>=_crs.rows(i)) && (_pos<_crs.rows(i+1))) {
                row = i;
                break;
            }
        }
        _value.first = key_type(row, col);
        _value.second = _crs.values(_pos);
    }
}

template <typename T, CRS_Storage Storage>
CRS_Iterator<T, Storage>::CRS_Iterator(const CRS<T, Storage> &crs, int pos)
    : _crs(crs), _pos(pos)
{
    if (pos<=_crs.numNonZeros()) {
        int col = _crs.columns(_pos);
        int row = _crs.numRows()+1;
        for (int i=1; i<=_crs.numRows(); ++i) {
            if ((pos>=_crs.rows(i)) && (pos<_crs.rows(i+1))) {
                row = i;
                break;
            }
        }
        _value.first = key_type(row, col);
        _value.second = _crs.values(_pos);
    }
}

template <typename T, CRS_Storage Storage>
bool
CRS_Iterator<T, Storage>::operator==(const CRS_Iterator &rhs) const
{
    return ((&_crs==&rhs._crs) && (_pos==rhs._pos));
}

template <typename T, CRS_Storage Storage>
bool
CRS_Iterator<T, Storage>::operator!=(const CRS_Iterator &rhs) const
{
    return !(*this==rhs);
}

template <typename T, CRS_Storage Storage>
CRS_Iterator<T, Storage> &
CRS_Iterator<T, Storage>::operator++()
{
    ++_pos;

    if (_pos<=_crs.numNonZeros()) {
        if (_pos>=_crs.rows(_value.first.first+1)) {
            ++_value.first.first;
        }
        _value.first.second = _crs.columns(_pos);
        _value.second = _crs.values(_pos);
    }
    return *this;
}

template <typename T, CRS_Storage Storage>
typename CRS_Iterator<T, Storage>::value_type &
CRS_Iterator<T, Storage>::operator*()
{
    return _value;
}

template <typename T, CRS_Storage Storage>
typename CRS_Iterator<T, Storage>::value_type *
CRS_Iterator<T, Storage>::operator->()
{
    return &_value;
}

} // namespace flens

