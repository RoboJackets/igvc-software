//pid controler
double stime();//time in seconds

class pid{
public:
	double now,then,olderr,err,i,I,d,P,D,dt,out,goal;
	pid(double Pn,double In,double Dn){
		now=then=olderr=err=i=I=d=P=D=dt=out=goal=0;
		P=Pn;
		I=In;
		D=Dn;
	}
	void setgoal(double thegoal){
		goal=thegoal;
	}
	double update(double vel){
		now=stime();
		dt=now-then;
		err=vel-goal;
		d=(err-olderr)/dt;
		i=i+err*dt;
		out=-P*d-I*i-D*d;
		olderr=err;
		then=now;
		return out;
	}
};

