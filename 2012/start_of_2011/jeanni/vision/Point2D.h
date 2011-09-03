#ifndef _POINT2D_H_
#define _POINT2D_H_

/*
 * Utility class that represents a 2D coordinate.
 */
template<class N>
class Point2D
{
public:
	N x;
	N y;

	Point2D<N>()
	{
		this->x = 0;
		this->y = 0;
	}

	Point2D<N>(N x, N y)
	{
		this->x = x;
		this->y = y;
	}

	//~Point2D<N>() {}

	Point2D operator+(Point2D other)
	{
		return Point2D(
				   this->x + other.x,
				   this->y + other.y);
	}

	Point2D operator+(N scalar)
	{
		return Point2D(
				   this->x + scalar,
				   this->y + scalar);
	}

	Point2D operator-(Point2D other)
	{
		return Point2D(
				   this->x - other.x,
				   this->y - other.y);
	}

	Point2D operator-(N scalar)
	{
		return Point2D(
				   this->x - scalar,
				   this->y - scalar);
	}

	Point2D operator*(N scalar)
	{
		return Point2D(
				   this->x * scalar,
				   this->y * scalar);
	}

	Point2D operator/(N scalar)
	{
		return Point2D(
				   this->x / scalar,
				   this->y / scalar);
	}

	bool operator==(const Point2D& other)
	{
		return (this->x == other.x) && (this->y == other.y);
	}

	bool operator!=(const Point2D& other)
	{
		return !(*this == other);
	}
};

#endif
