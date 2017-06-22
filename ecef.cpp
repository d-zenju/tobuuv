// Required: C++11 (for OSX: g++ -std=c++11 ecef.cpp)
// http://www.enri.go.jp/~fks442/K_MUSEN/1st/1st021118.pdf

#include "ecef.h"
#include <Arduino.h>

void ECEF::rx(double theta, double (*mat)[3]) {
    mat[0][0] = 1.0;
    mat[0][1] = 0.0;
    mat[0][2] = 0.0;
    mat[1][0] = 0.0;
    mat[1][1] = cos(theta * pi / 180.0);
    mat[1][2] = sin(theta * pi / 180.0);
    mat[2][0] = 0.0;
    mat[2][1] = -sin(theta * pi / 180.0);
    mat[2][2] = cos(theta * pi / 180.0);
}

void ECEF::ry(double theta, double (*mat)[3]) {
    mat[0][0] = cos(theta * pi / 180.0);
    mat[0][1] = 0.0;
    mat[0][2] = -sin(theta * pi / 180.0);
    mat[1][0] = 0.0;
    mat[1][1] = 1.0;
    mat[1][2] = 0.0;
    mat[2][0] = sin(theta * pi / 180.0);
    mat[2][1] = 0.0;
    mat[2][2] = cos(theta * pi / 180.0);
}

void ECEF::rz(double theta, double (*mat)[3]) {
    mat[0][0] = cos(theta * pi / 180.0);
    mat[0][1] = sin(theta * pi / 180.0);
    mat[0][2] = 0.0;
    mat[1][0] = -sin(theta * pi / 180.0);
    mat[1][1] = cos(theta * pi / 180.0);
    mat[1][2] = 0.0;
    mat[2][0] = 0.0;
    mat[2][1] = 0.0;
    mat[2][2] = 1.0;
}

void ECEF::mm(double (*mat_a)[3], double (*mat_b)[3], double (*mat_answer)[3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                mat_answer[i][j] += mat_a[i][k] * mat_b[k][j];
            }
        }
    }
}

void ECEF::mv(double (*mat_a)[3], double *mat_b, double *mat_answer) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat_answer[i] += mat_a[i][j] * mat_b[j];
        }
    }
}

// WGS84 to ECEF
// input :: phi = latitude, lambda = longitude, height = height
// output :: vector (x, y, z)
void ECEF::bih2ecef(double phi, double lambda, double height, double *ecef) {
  double x = (ECEF::n(phi) + height) * cos(phi * pi / 180.0) * cos(lambda * pi / 180.0);
  double y = (ECEF::n(phi) + height) * cos(phi * pi / 180.0) * sin(lambda * pi / 180.0);
  double z = (ECEF::n(phi) * (1 - e2) + height) * sin(phi * pi / 180.0);
  ecef[0] = x;
  ecef[1] = y;
  ecef[2] = z;
}

// WGS84 to ECEF
// input :: vector(latitude, longitude, height)
// output :: vector (x, y, z)
void ECEF::bih2ecef(double *bih, double *ecef) {
  double phi = bih[0];
  double lambda = bih[1];
  double height = bih[2];
  double x = (n(phi) + height) * cos(phi * pi / 180.0) * cos(lambda * pi / 180.0);
  double y = (n(phi) + height) * cos(phi * pi / 180.0) * sin(lambda * pi / 180.0);
  double z = (n(phi) * (1 - e2) + height) * sin(phi * pi / 180.0);
  ecef[0] = x;
  ecef[1] = y;
  ecef[2] = z;
}

// ECEF to WGS84
// input :: x = x, y = y, z = z
// output :: vector (latitude, longitude, height)
void ECEF::ecef2bih(double x, double y, double z, double *bih) {
  double p = sqrt(x * x + y * y);
  double sita = (180.0 / pi) * atan2(z * a, p * b);
  double phi = (180.0 / pi) * atan2(z + ed2 * b * pow(sin(sita * pi / 180.0), 3), (p - e2 * a * pow(cos(sita * pi / 180.0), 3)));
  double lambda = (180.0 / pi) * atan2(y, x);
  double height = (p / cos(phi * pi / 180.0)) - n(phi);
  bih[0] = phi;
  bih[1] = lambda;
  bih[2] = height;
}

// ECEF to WGS84
// input :: vector (x, y, z)
// output :: vector (latitude, longitude, height)
void ECEF::ecef2bih(double *ecef, double *bih) {
  double x = ecef[0];
  double y = ecef[1];
  double z = ecef[2];
  double p = sqrt(x * x + y * y);
  double sita = (180.0 / pi) * atan2(z * ECEF::a, p * b);
  double phi = (180.0 / pi) * atan2(z + ed2 * b * pow(sin(sita * pi / 180.0), 3), (p - e2 * a * pow(cos(sita * pi / 180.0), 3)));
  double lambda = (180.0 / pi) * atan2(y, x);
  double height = (p / cos(phi * pi / 180.0)) - n(phi);
  bih[0] = phi;
  bih[1] = lambda;
  bih[2] = height;
}

void ECEF::ecef2enu(double *ecef_dest, double *ecef_origin, double *enu) {
  double rot0[3][3];
  double rot1[3][3];
  double rot2[3][3];
  double m[3][3];
  double r[3][3];
  double p[3] = {ecef_dest[0] - ecef_origin[0], ecef_dest[1] - ecef_origin[1], ecef_dest[2] - ecef_origin[2]};
  double enu_a[3];
  double bih[3];

  ecef2bih(ecef_origin, bih);

  rz(90.0 * pi / 180.0, rot0);
  ry((90.0 - bih[0]) * pi / 180.0, rot1);
  rz(bih[2] * pi / 180.0, rot2);

  mm(rot0, rot1, m);
  mm(m, rot2, r);
  mv(r, p, enu_a);

  enu[0] = enu_a[0];
  enu[1] = enu_a[1];
  enu[2] = enu_a[2];
}


void ECEF::bih2enu(double *bih_dest, double *bih_origin, double *enu) {
  double ecef_dest[3];
  double ecef_origin[3];
  double bih[3];

  bih2ecef(bih_dest, ecef_dest);
  bih2ecef(bih_origin, ecef_origin);
  
  double rot0[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double rot1[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double rot2[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double m[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double r[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double p[3];
  for (int i = 0; i < 3; i++) {
      p[i] = ecef_dest[i] - ecef_origin[i];
  }
  double enu_a[3] = {0.0, 0.0, 0.0};
  ecef2bih(ecef_origin, bih);

  rz(90.0, rot0);
  ry(90.0 - bih[0], rot1);
  rz(bih[1], rot2);

  mm(rot0, rot1, m);
  mm(m, rot2, r);
  mv(r, p, enu_a);

  enu[0] = enu_a[0];
  enu[1] = enu_a[1];
  enu[2] = enu_a[2];
}

double ECEF::enu2length(double *enu) {
    return sqrt(pow(enu[0], 2) + pow(enu[1], 2));
}

double ECEF::enu2angle(double *enu) {
    return atan(enu[1] / enu[0]) * 180.0 / pi;
}

double ECEF::enu2direction(double *enu) {
    double r = atan(enu[1] / enu[0]) * 180.0 / pi;
    double direction;
    if (enu[0] < 0 && enu[1] >= 0) direction = 270.0 + (180.0 - r);
    else direction = 90.0 - r;
    return direction;
}

double ECEF::bih2length(double *bih_dest, double *bih_origin) {
    double enu[3];
    bih2enu(bih_dest, bih_origin, enu);
    return sqrt(pow(enu[0], 2) + pow(enu[1], 2));
}

double ECEF::bih2angle(double *bih_dest, double *bih_origin) {
    double enu[3];
    bih2enu(bih_dest, bih_origin, enu);
    return atan(enu[1] / enu[0]) * 180.0 / pi;
}

double ECEF::bih2direction(double *bih_dest, double *bih_origin) {
    double enu[3];
    bih2enu(bih_dest, bih_origin, enu);
    double r = atan(enu[1] / enu[0]) * 180.0 / pi;
    double direction;
    if (enu[0] < 0 && enu[1] >= 0) direction = 270.0 + (180.0 - r);
    else direction = 90.0 - r;
    return direction;
}
