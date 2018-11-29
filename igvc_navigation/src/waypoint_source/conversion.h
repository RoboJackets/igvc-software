#ifndef CONVERSION_H
#define CONVERSION_H
// Code taken from conversions.h source code
// Github: https://github.com/ktossell/gps_umd/blob/master/gps_common/include/gps_common/conversions.h

const double RADIANS_PER_DEGREE = M_PI / 180.0;

// WGS84 Parameters
const double WGS84_A = 6378137.0;         // major axis
const double WGS84_B = 6356752.31424518;  // minor axis
const double WGS84_F = 0.0033528107;      // ellipsoid flattening
const double WGS84_E = 0.0818191908;      // first eccentricity
const double WGS84_EP = 0.0820944379;     // second eccentricity
// UTM Parameters
const double UTM_K0 = 0.9996;                    // scale factor
const double UTM_FE = 500000.0;                  // false easting
const double UTM_FN_N = 0.0;                     // false northing on north hemisphere
const double UTM_FN_S = 10000000.0;              // false northing on south hemisphere
const double UTM_E2 = (WGS84_E * WGS84_E);       // e^2
const double UTM_E4 = (UTM_E2 * UTM_E2);         // e^4
const double UTM_E6 = (UTM_E4 * UTM_E2);         // e^6
const double UTM_EP2 = (UTM_E2 / (1 - UTM_E2));  // e'^2

static inline void UTM(double lat, double lon, double *x, double *y)
{
  // constants
  const static double m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256);
  const static double m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024);
  const static double m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024);
  const static double m3 = -(35 * UTM_E6 / 3072);

  // compute the central meridian
  int cm = ((lon >= 0.0) ? ((int)lon - ((int)lon) % 6 + 3) : ((int)lon - ((int)lon) % 6 - 3));

  // convert degrees into radians
  double rlat = lat * RADIANS_PER_DEGREE;
  double rlon = lon * RADIANS_PER_DEGREE;
  double rlon0 = cm * RADIANS_PER_DEGREE;

  // compute trigonometric functions
  double slat = sin(rlat);
  double clat = cos(rlat);
  double tlat = tan(rlat);

  // decide the false northing at origin
  double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

  double T = tlat * tlat;
  double C = UTM_EP2 * clat * clat;
  double A = (rlon - rlon0) * clat;
  double M = WGS84_A * (m0 * rlat + m1 * sin(2 * rlat) + m2 * sin(4 * rlat) + m3 * sin(6 * rlat));
  double V = WGS84_A / sqrt(1 - UTM_E2 * slat * slat);
  // compute the easting-northing coordinates
  *x = UTM_FE +
       UTM_K0 * V * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * pow(A, 5) / 120);
  *y = fn + UTM_K0 * (M + V * tlat *
                              (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 +
                               ((61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2) * pow(A, 6) / 720)));

  return;
}

#endif  // CONVERSION_H
