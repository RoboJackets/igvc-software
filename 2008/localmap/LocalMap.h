class LocalMap {
public:
	Timestamp timestamp;
	QList<CamObject> objects;
};

/*abstract*/ class CamObject {
public:
	CamColor color;
};

/** Timestamp in microseconds since initialization of .... */
typedef Timestamp uint64_t;

typedef enum {
	CC_ORANGE_STRIPED	= 1,
	CC_BLUE				= 2,
	CC_RED				= 3,
	CC_YELLOW			= 4,
	CC_WHITE			= 5,
	CC_BROWN			= 6,
	CC_GREEN			= 7,
	CC_BLACK			= 8,
} CamColor;

#include <math.h>	// for NaN

/**
 * Can represent:
 * 1. standalone point obstacles or
 * 2. component points of other CamObjects.
 */
class CamPoint : CamObject {
public:
	double x;
	double y;
	double z;		// NaN if unknown
	double error;	// 3D radius of maximum error
	
	CamPoint(double x, double y, double error)
		: CamPoint(x, y, NaN, error)
	{
		// nothing
	}
	
	CamPoint(double x, double y, double z, double error) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->error = error;
	}
};

/**
 * Can represent:
 * 1. normal lines (with no implied segments) or
 * 2. dashed lines (with some implied segments, not necessarily alternating).
 */
class CamLine : CamObject {
public:
	QList<CamPoint> points;	// N points
private:
	QList<boolean> implied;	// N-1 line segments
	
	// ### ACCESSORS: LINE SEGMENTS ###
public:
	int getSegmentCount() { return points.size()-1; }
	CamPoint& getSegmentStart(int segID) { return points[segID]; }
	CamPoint& getSegmentEnd(int segID) { return points[segID+1]; }
	boolean isSegmentImplied(int segID) { return implied[segID]; }
};

/**
 * Can represent:
 * 1. barrel base
 * 2. sandtrap
 * 3. ramp
 * 4. type A barricade AKA fence
 * 5. tree base
 * 6. person's foot
 */
class CamPolygon : CamObject {
public:
	CamPolygonType type;
	QList<CamPoint> points;
};

typedef enum {
	CPT_BARREL		= 1,
	CPT_SANDTRAP	= 2,
	CPT_RAMP		= 3,
	// type A barricade
	CPT_FENCE		= 4,
	CPT_PERSON		= 5,
} CamPolygonType;
