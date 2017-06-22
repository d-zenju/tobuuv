/*
 * コメント
 * getGPSは常に割り込み処理ではなく、常時実行状態にすること
 * 受からないことの方が多いので、値を取得できない
 */

#include <stdlib.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include "TinyGPS++.h"
#include "ecef.h"
#include "mpu9250.h"


/***************/
/*** 各種設定 ***/
/***************/

// 定点座標 (緯度, 経度)
#define Lat 35.669715
#define Lng 139.797874


// シリアル通信速度
#define SerialBPS     38400
#define gpsSerialBPS  9600

// GPSシリアルピン番号
#define RX  10
#define TX  11
#define PPS 13


// GPS構造体
struct GPSData {
  int year;         // Year (2000+)
  int month;        // Month (1-12)
  int day;          // Day (1-31)
  int hour;         // Hour (0-23)
  int minute;       // Minute (0-59)
  int second;       // Second (0-59)
  int centisecond;  // 100ths of a second (0-99)
  double lat;       // Latitude
  double lng;       // Longitude
  double knots;     // Speed in knots
  double mph;       // Speed in miles per hour
  double mps;       // Speed in miles per second
  double kmph;      // Speed in kilometers per hour
  double course;    // Course in degrees
  double alt;       // Altitude in meters
  double hgeo;      // Height of geoid
  double ellipsh;   // Ellipsoid height
  int satellites;   // Number of sattelites in use
  double hdop;      // Horizontal Dim. of Precision
};


// GPS Age構造体 (最後に更新してからの時刻[ms], 1500ms以上は情報が古すぎるので注意)
struct GPSAge {
  int date;        // elapsed time from get old Date's Data (ms)
  int time;        // elapsed time from get old Time's Data (ms)
  int location;    // elapsed time from get old Location's Data (ms)
  int speed;       // elapsed time from get old Speed's Data (ms)
  int course;      // elapsed time from get old Course's Data (ms)
  int altitude;    // elapsed time from get old Altitude's Data (ms)
  int satellites;  // elapsed time from get old Satellites's Data (ms)
  int hdop;        // elapsed time from get old Hdop's Data (ms)
};


// GPS Valid構造体 (各種データの有効性, 0:無効, 1:有効)
struct GPSValid {
  int date;
  int time;
  int location;
  int speed;
  int course;
  int altitude;
  int satellites;
  int hdop;
};


// Waypoint構造体
struct Waypoint {
  double direction; // 方位
  double distance;  // 距離
};


// IMU構造体
struct YPR {
  double roll;  // roll
  double pitch; // pitch
  double yaw;   // yaw
};


// GPS関連グローバル変数
SoftwareSerial gpsSerial(RX, TX);
TinyGPSPlus gps;
GPSData gpsData;
GPSAge gpsAge;
GPSValid gpsValid;
TinyGPSCustom geoid(gps, "GPGGA", 11);

ECEF ecef;
Waypoint wp;

MPU9250 imu;
Kalman kalmanRoll;
Kalman kalmanPitch;
Kalman kalmanCompass;
uint32_t timer;
YPR ypr;



void setup() {
  // シリアル通信(bps)
  Serial.begin(SerialBPS);
  gpsSerial.begin(gpsSerialBPS);

  // 1PPSピンモード(READ)
  pinMode(PPS, INPUT);

  //MsTimer2::set(1000, getGPS);
  MsTimer2::set(500, calcWaypoint);
  MsTimer2::start();
}


void loop() {
  getGPS(); // GPS取得
  
  // Debug
  if (digitalRead(PPS) == LOW) {
    //calcWaypoint();  // Waypoint計算
    
    Serial.print(gpsData.hour); Serial.print(":");
    Serial.print(gpsData.minute); Serial.print(":");
    Serial.print(gpsData.second); Serial.print(" | ");
    Serial.print(gpsData.lat, 6); Serial.print(", ");
    Serial.print(gpsData.lng, 6); Serial.print(", ");
    Serial.print(gpsData.alt); Serial.print("[");
    Serial.print(gpsData.hgeo); Serial.print(", ");
    Serial.print(gpsData.ellipsh); Serial.print("] ");
    Serial.print(wp.distance); Serial.print(", ");
    Serial.print(wp.direction); Serial.print(" ");
    
    Serial.print("\n");
  }
}


// Waypoint計算
void calcWaypoint() {
  double target[3] = {Lat, Lng, gpsData.ellipsh};
  double own[3] = {gpsData.lat, gpsData.lng, gpsData.ellipsh};
  wp.distance = ecef.bih2length(own, target);
  wp.direction = ecef.bih2direction(own, target);
}


// GPS取得
void getGPS() {
  // GPS取得・エンコード
  if (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Date取得
  if (gps.date.isUpdated()) {
    gpsData.year = gps.date.year();
    gpsData.month = gps.date.month();
    gpsData.day = gps.date.day();
  }

  // Time取得
  if (gps.time.isUpdated()) {
    gpsData.hour = gps.time.hour();
    gpsData.minute = gps.time.minute();
    gpsData.second = gps.time.second();
    gpsData.centisecond = gps.time.centisecond();
  }

  // Location取得
  if (gps.location.isUpdated()) {
    gpsData.lat = gps.location.lat();
    gpsData.lng = gps.location.lng();
  }

  // Speed取得
  if (gps.speed.isUpdated()) {
    gpsData.knots = gps.speed.knots();
    gpsData.mph = gps.speed.mph();
    gpsData.mps = gps.speed.mps();
    gpsData.kmph = gps.speed.kmph();
  }

  // Altitude取得
  if (gps.altitude.isUpdated()) {
    gpsData.alt = gps.altitude.meters();
    if (geoid.isUpdated()) {
      gpsData.hgeo = atof(geoid.value());
      gpsData.ellipsh = gpsData.alt + gpsData.hgeo;
    }
  }

  // Course取得
  if (gps.course.isUpdated()) {
    gpsData.course = gps.course.deg();
  }

  // Satellites取得
  if (gps.satellites.isUpdated()) {
    gpsData.satellites = gps.satellites.value(); 
  }

  // HDOP取得
  if (gps.hdop.isUpdated()) {
    gpsData.hdop = gps.hdop.value() / 100.0;
  }

  // Age取得
  gpsAge.date = gps.date.age();
  gpsAge.time = gps.time.age();
  gpsAge.location = gps.location.age();
  gpsAge.speed = gps.speed.age();
  gpsAge.altitude = gps.altitude.age();
  gpsAge.course = gps.course.age();
  gpsAge.satellites = gps.satellites.age();
  gpsAge.hdop = gps.hdop.age();

  // Valid取得
  gpsValid.date = gps.date.isValid();
  gpsValid.time = gps.time.isValid();
  gpsValid.location = gps.location.isValid();
  gpsValid.speed = gps.speed.isValid();
  gpsValid.altitude = gps.altitude.isValid();
  gpsValid.course = gps.course.isValid();
  gpsValid.satellites = gps.satellites.isValid();
  gpsValid.hdop = gps.hdop.isValid();
}
