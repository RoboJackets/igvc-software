#ifndef GPSACCURACY_H
#define GPSACCURACY_H

class GPSAccuracy
{
    public:
        inline GPSAccuracy(): _LatVariance(0), _LongVariance(0),
         _HeadingVariance(0), _SpeedVariance(0)
        {

        }

        inline GPSAccuracy(double latVar, double longVar, double headingVar, double speedVar) : _LatVariance(latVar), _LongVariance(longVar),
         _HeadingVariance(headingVar), _SpeedVariance(speedVar)
         {
         }

          GPSAccuracy defaultGPSAccuracy()
          {
            double latVar = 1.1082;//*pow(10,-10);
            double longVar = 7.352;//*pow(10,-11);
            double headingVar = .0076; //Assumes std of 5 degrees
            double speedVar = 4.25;//*pow(10,-4);
            return GPSAccuracy(latVar, longVar, headingVar, speedVar);
          }

         inline double LatVar(void)
         {
            return _LatVariance;
         }


         inline double LongVar(void)
         {
             return _LongVariance;
         }

         inline double HeadingVar()
         {
             return _HeadingVariance;
         }

        inline double SpeedVar(void)
         {
            return _SpeedVariance;
         }

         inline void ScaleUncertainty(double scale)
         {
             _LatVariance*=scale;
             _LongVariance*=scale;
            _HeadingVariance*=scale;
            _SpeedVariance*=scale;
         }

        inline void LatVar(double newVar)
        {
            _LatVariance = newVar;
        }

        inline void LongVar(double newVar)
        {
            _LongVariance = newVar;
        }

        inline void HeadingVar(double newVar)
        {
            _HeadingVariance = newVar;
        }

        inline void SpeedVar(double newVar)
        {
            _SpeedVariance = newVar;
        }

    private:
        double _LatVariance;
        double _LongVariance;
        double _HeadingVariance;
        double _SpeedVariance;
};

#endif // GPSACCURACY_H
