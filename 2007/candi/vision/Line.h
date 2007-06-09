#ifndef _LINE_H_
#define _LINE_H_
	
/*
 * Utility class that represents a line segment.
 */
 
template<class N>
class Line
{
	#define sqr(x) ((x)*(x))
	#define dist(p1, p2) (sqr(p1.x-p2.x) + sqr(p1.y-p2.y))
public:
	Point2D<N> a;	//note that line segments behave as distinguishible
	Point2D<N> b;	//but not ordered.
	
	Line<N>() {
		this->a = Point2D<N>();
		this->b = Point2D<N>();
	}
	
	Line<N>(const Line<N>& l) {
		this->a = l.a;
		this->b = l.b;
	}
	
	Line<N>(Point2D<N> a, Point2D<N> b) {
		this->a = a;
		this->b = b;
	}
	
	Line<N>(N x1, N y1, N x2, N y2) {
		this->a.x = x1;
		this->a.y = y1;
		this->b.x = x2;
		this->b.y = y2;
	}
	
	/*~Line<N>() {}
	length();
	angle();
	P2Pdist(Point p);
	P2Pdist(Line l);
	top();
	bool hastop();
	bottom();
	bool hasbottom();
	left();
	bool hasleft();
	right();
	bool hasright();
	schlunk();
	nearest();*/
	
	// top, bottom, left, right
		Point2D<N> top(){
			return (a.y<b.y)?a:b;
		}
		
		Point2D<N> bottom(){
			return (a.y>b.y)?a:b;
		}
		
		Point2D<N> left(){
			return (a.x<b.x)?a:b;
		}
		
		Point2D<N> right(){
			return (a.y<b.y)?a:b;
		}
	
	//schlunk
		Line<N> schlunk(const Line<N> l){
			//note that the result is guarenteed to go from this to l.
			return Line<N>(nearEnd(l),l.nearEnd(*this));
		}
		
		Line<N> schlunk(const Point2D<N> p){
			return Line<N>(nearEnd(p),p);
		}
	
	//nearEnd
		Point2D<N> nearEnd(const Line<N> l){
			Point2D<N> a1= (dist(a,l.a)<dist(a,l.b))?l.a:l.b;
			Point2D<N> b1= (dist(b,l.a)<dist(b,l.b))?l.a:l.b;
			return (dist(a,a1)<dist(b,b1))?a:b;
		}
		
		Point2D<N> nearEnd(const Point2D<N> p){
			return (dist(a,p)<dist(b,p))?a:b;
		}
	
	#undef sqr
	#undef dist
};

#endif //_LINE_H_
