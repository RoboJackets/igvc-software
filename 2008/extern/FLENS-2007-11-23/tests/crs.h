#ifndef FLENS_CRS_H
#define FLENS_CRS_H 1

#include <flens/array.h>
#include <flens/densevector.h>
#include <map>

namespace flens {

//-- CRS -----------------------------------------------------------------------

enum CRS_Storage {
    CRS_General,
    CRS_UpperTriangular,
    CRS_LowerTriangular
};

template <typename T, CRS_Storage Storage>
class CRS_Initializer;

template <typename T, CRS_Storage Storage>
class CRS_OrderedInitializer;

template <typename T, CRS_Storage Storage>
class CRS_RandomAccessInitializer;

template <typename T, CRS_Storage Storage>
class CRS_ConstIterator;

template <typename T, CRS_Storage Storage>
class CRS_Iterator;

template <typename T, CRS_Storage Storage=CRS_General>
class CRS
{
    public:
        typedef T                                          ElementType;
        typedef CRS_RandomAccessInitializer<T, Storage>    Initializer;
        typedef CRS_ConstIterator<T, Storage>              const_iterator;
        typedef CRS_ConstIterator<T, Storage>              iterator;

        CRS(int numRows, int numCols, int approxBandwidth=1);

        ~CRS();

        Initializer *
        initializer();

        void
        allocate(int numNonZeros);

        int
        numRows() const;

        int
        numCols() const;

        int
        numNonZeros() const;

        const_iterator
        begin() const;

        iterator
        begin();

        const_iterator
        end() const;

        iterator
        end();

        DenseVector<Array<T> >    values;
        DenseVector<Array<int> >  columns, rows;

    private:
        int  _numRows, _numCols, _k;
};

//-- CRS_Initializer -----------------------------------------------------------

class CRS_IndexCmp
{
    public:

        // return true if a < b
        //   <=>  a(1)<b(1)  or  a(1)==b(1) and a(2)<b(2)
        bool
        operator()(const std::pair<int,int> &a,
                   const std::pair<int,int> &b) const;
};

template <typename T, CRS_Storage Storage>
class CRS_Initializer
{
    public:
        CRS_Initializer(CRS<T, Storage> &crs);

        ~CRS_Initializer();

        T &
        operator()(int row, int col);

    private:
        typedef std::map<std::pair<int,int>,T,CRS_IndexCmp> CRS_Map;
        CRS_Map         _map;
        CRS<T, Storage> &_crs;
};

//-- CRS_RandomAccessInitializer -----------------------------------------------

template <typename T>
struct CRS_RAI_Coordinate
{
    CRS_RAI_Coordinate(int _row, int _col, T _value)
        : row(_row), col(_col), value(_value)
    {
    }
    
    int row, col;
    T   value;
};

class CRS_RAI_IndexCmp
{
    public:

        // return true if a < b
        //   <=>  a(1)<b(1)  or  a(1)==b(1) and a(2)<b(2)
        template <typename T>
        bool
        operator()(const CRS_RAI_Coordinate<T> &a,
                   const CRS_RAI_Coordinate<T> &b) const
        {
            if (a.row<b.row) {
                return true;
            }
            if (a.row>b.row) {
                return false;
            }
            if (a.col<b.col) {
                return true;
            }
            return false;
        }
};

template <typename T, CRS_Storage Storage>
class CRS_RandomAccessInitializer
{
    public:
        CRS_RandomAccessInitializer(CRS<T, Storage> &crs, int k);

        ~CRS_RandomAccessInitializer();

        void
        sort();

        T &
        operator()(int row, int col);

    private:
        typedef std::vector<CRS_RAI_Coordinate<T> > Coordinates;
        Coordinates _coordinates;
        size_t _middle;
        int    _insertCount;
        CRS<T, Storage> &_crs;
        bool _isSorted;
        CRS_RAI_IndexCmp _less;
};

//-- CRS_OrderedInitializer ----------------------------------------------------

template <typename T, CRS_Storage Storage>
class CRS_OrderedInitializer
{
    public:
        CRS_OrderedInitializer(CRS<T, Storage> &crs);

        ~CRS_OrderedInitializer();

        T &
        operator()(int row, int col);

    private:
        std::vector<int> _row, _col;
        std::vector<T> _value;
        CRS<T, Storage> &_crs;
        int _currentRow;
};

//-- CRS_ConstIterator ---------------------------------------------------------

template <typename T, CRS_Storage Storage>
class CRS_Iterator;

template <typename T, CRS_Storage Storage>
class CRS_ConstIterator
{
    public:
        typedef std::pair<int,int>                      key_type;
        typedef T                                       mapped_type;
        typedef std::pair<key_type, mapped_type>        value_type;

        CRS_ConstIterator(const CRS_ConstIterator &rhs);

        CRS_ConstIterator(const CRS_Iterator<T, Storage> &rhs);

        CRS_ConstIterator(const CRS<T, Storage> &crs, int pos);

        bool
        operator==(const CRS_ConstIterator &rhs) const;

        bool
        operator!=(const CRS_ConstIterator &rhs) const;

        CRS_ConstIterator &
        operator++();

        const value_type &
        operator*() const;

        const value_type *
        operator->() const;

        const CRS<T, Storage>  &_crs;
        int                    _pos;
        value_type             _value;
};

//-- CRS_Iterator --------------------------------------------------------------

template <typename T, CRS_Storage Storage>
class CRS_Iterator
{
    public:
        typedef std::pair<int,int>                      key_type;
        typedef T                                       mapped_type;
        typedef std::pair<key_type, mapped_type>        value_type;

        CRS_Iterator(const CRS_Iterator &rhs);

        CRS_Iterator(const CRS<T, Storage> &crs, int pos);

        bool
        operator==(const CRS_Iterator &rhs) const;

        bool
        operator!=(const CRS_Iterator &rhs) const;

        CRS_Iterator<T, Storage> &
        operator++();

        value_type &
        operator*();

        value_type *
        operator->();

        const CRS<T, Storage>  &_crs;
        int                    _pos;
        value_type             _value;
};

} // namespace flens

#include <flens/crs.tcc>

#endif // FLENS_CRS_H
