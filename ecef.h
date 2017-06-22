// Required: C++11 (for OSX: g++ -std=c++11 ecef.cpp)
// http://www.enri.go.jp/~fks442/K_MUSEN/1st/1st021118.pdf

#ifndef _ECEF_H_
#define _ECEF_H_

#include <stdio.h>
#include <math.h>


class ECEF {
  private:
    const double pi = 3.141592653589793;
    const double a = 6378137.0;
    const double one_f = 298.257223563;
    const double b = a * (1.0 - 1.0 / one_f);
    const double e2 = (1.0 / one_f) * (2 - (1.0 / one_f));
    const double ed2 = e2 * a * a / (b * b);
    double n(double p) {
      return a / sqrt(1.0 - e2 * pow(sin(p * M_PI / 180.0), 2));
    }
    void rx(double theta, double (*mat)[3]);
    void ry(double theta, double (*mat)[3]);
    void rz(double theta, double (*mat)[3]);
    void mm(double (*mat_a)[3], double (*mat_b)[3], double (*mat_answer)[3]);
    void mv(double (*mat_a)[3], double *mat_b, double *mat_answer);

  public:
    // WGS84 to ECEF
    // input :: phi = latitude, lambda = longitude, height = height
    // output :: ecef (x, y, z)
    void bih2ecef(double phi, double lambda, double height, double *ecef);

    // WGS84 to ECEF
    // input :: bih (latitude, longitude, height)
    // output :: ecef (x, y, z)
    void bih2ecef(double *bih, double *ecef);

    // ECEF to WGS84
    // input :: x = x, y = y, z = z
    // output :: bih (latitude, longitude, height)
    void ecef2bih(double x, double y, double z, double *bih);

    // ECEF to WGS84
    // input :: ecef (x, y, z)
    // output :: bih (latitude, longitude, height)
    void ecef2bih(double *ecef, double *bih);

    // ENU
    void ecef2enu(double *ecef_dest, double *ecef_origin, double *enu);
    void bih2enu(double *bih_dest, double *bih_origin, double *enu);

    // ENU to length, angle
    double enu2length(double *enu);
    double enu2angle(double *enu);
    double enu2direction(double *enu);

    // BIH to length, angle
    double bih2length(double *bih_dest, double *bih_origin);
    double bih2angle(double *bih_dest, double *bih_origin);
    double bih2direction(double *bih_dest, double *bih_origin);
};

#endif
