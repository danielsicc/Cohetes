#include <Wire.h>
#include <ADXL345.h>
#include <BMP085.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Servo.h>

NMEAGPS  gps;
gps_fix  fix;
ADXL345 accel;
BMP085 bmp;
Servo PayLoad;
int aceleracionBandera=3, alturaBandera=-50, tiempoBandera=100;
boolean desc=false, descConf=false, launch=false, alt=false, satFix=false;
long tiempoUno, tiempoDos;
double referencePressure;
float g=9.81, descensoBandera=0.9, latitud, longitud;

void setup() {
  PayLoad.attach(11);
  Serial.begin(115200);
  //Serial1.begin(9600);
  Serial.println("Initialize ADXL345");
  accel.begin();
  Serial.println("Initialize BMP085");
  bmp.begin(BMP085_ULTRA_HIGH_RES);
  accel.setRange(ADXL345_RANGE_16G);
  PayLoad.write(90);
  referencePressure = 101100;
}

void loop(){
  while(gps.available(Serial1)){
  fix=gps.read();
  latitud=fix.latitude();
  longitud=fix.longitude();
  }
  Vector norm = accel.readNormalize();
  Vector filt = accel.lowPassFilter(norm, 0.5);
  long pressure = bmp.readPressure();

  int pitch = -(atan2(filt.XAxis, sqrt(filt.YAxis*filt.YAxis + filt.ZAxis*filt.ZAxis))*180.0)/M_PI;
  int roll  = (atan2(filt.YAxis, filt.ZAxis)*180.0)/M_PI;

  float absoluteAltitude = bmp.getAltitude(pressure);
  float relativeAltitude = bmp.getAltitude(pressure, referencePressure);

  Serial.print(" X=");
  Serial.print(norm.XAxis);
  Serial.print(" Y=");
  Serial.print(norm.YAxis);
  Serial.print(" Z=");
  Serial.print(norm.ZAxis);
  Serial.print(" Pressure=");
  Serial.print(pressure);
  Serial.print("Pa absoluteAltitude=");
  Serial.print(absoluteAltitude);
  Serial.print("m relativeAltitude=");
  Serial.print(relativeAltitude);    
  Serial.print("m Pitch=");
  Serial.print(pitch);
  Serial.print(" Roll=");
  Serial.print(roll);
  Serial.print(" Lat=");
  Serial.print(latitud,7);
  Serial.print(" Lon=");
  Serial.println(longitud,7);

  if(norm.YAxis>aceleracionBandera*g){
    launch=true;
  }
  if(launch==true){
    Serial.println("Post-Launch");
  }
  else{
    Serial.println("Pre-Launch");
  }
  if(relativeAltitude>alturaBandera){
    alt=true;
  }
  if(alt==true){
    Serial.println("Good-Altitude");
  }
  else{
    Serial.println("Bad-Altitude");
  }
  if(fix.valid.location==true){
    satFix=true;
  }
  if(satFix==true){
    Serial.println("GPS-Locked");
  }
  else{
    Serial.println("GPS-No-Lock");
  }

  if(launch==true&&alt==true&&descConf==false){
    if(norm.YAxis<1&&desc==false){
      tiempoUno=millis();
      desc=true;
    }
    if(norm.YAxis<1&&desc==true){
      tiempoDos=millis()-tiempoUno;
      Serial.println(tiempoDos);
    }
    if(tiempoDos>tiempoBandera){
      descConf=true;
    }
    else if(norm.YAxis>1){
      desc=false;
    }
  }
  if(descConf==true){
    PayLoad.write(180);
    Serial.println("Descent");
    if(norm.YAxis>1&&norm.YAxis<descensoBandera*g){
      Serial.println("Post-Parachute");
    }
    else{
      Serial.println("Pre-Parachute");
    }
  }
}
