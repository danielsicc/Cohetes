#include <Wire.h>
#include <MPU6050.h>
#include <BMP085.h>
#include <SPI.h>
#include <LoRa.h>

MPU6050 accelerometer;
BMP085 barometer;

int aceleracionBandera=3, alturaBandera=5, tiempoBandera=500, counter=0;
boolean desc=false, descConf=false, launch=false, alt=false;
long tiempoUno, tiempoDos, pressure;
double referencePressure;
float g=9.81, acelMax1=0, acelMax2=0, alturaInicial=0, velocidadDesc=0, alturaMax=0, absoluteAltitude, relativeAltitude;
String data, launchData;
Vector norm, filtered;

void setup(){
  Serial.begin(115200);

  Serial.println("Initialize ADXL345");
  if (!accelerometer.begin()){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  Serial.println("Initialize BMP085");
  while(!barometer.begin(BMP085_ULTRA_HIGH_RES))
  {
    Serial.println("Could not find a valid BMP085 or BMP180 sensor, check wiring!");
    delay(500);
  }
  accelerometer.setRange(MPU6050_RANGE_16G);

  referencePressure = barometer.readPressure();
  alturaInicial=barometer.getAltitude(barometer.readPressure(), referencePressure);
  LoRa.begin(430E6);

  LoRa.beginPacket();
  LoRa.println("SYSTEM READY");
  LoRa.println("SYSTEM READY");
  LoRa.println("SYSTEM READY");
  LoRa.endPacket();
  delay(500);
}

void loop(){
  acquisition();
  phases();
  telemetry();
}

void acquisition(){
  norm = accelerometer.readNormalizeAccel();
  filtered = accelerometer.lowPassFilter(norm, 0.5);
  pressure = barometer.readPressure();

  absoluteAltitude = barometer.getAltitude(pressure);
  relativeAltitude = barometer.getAltitude(pressure, referencePressure);
}

void phases(){
  if(norm.YAxis>aceleracionBandera*g){
    launch=true;
  }

  if(launch==true){
    acelMax1=norm.YAxis;
    launchData=("POST-LAUNCH\n");
    if(acelMax1>acelMax2){
      acelMax2=acelMax1;
    }
  }

  else{
    launchData=("PRE-LAUNCH\n");
  }

  if(relativeAltitude>alturaBandera){
    alt=true;
  }

  if(alt==true){
    launchData+=("GOOD ALTITUDE\n");
    //Serial.println("Good-Altitude");
  }

  else{
    launchData+=("BAD ALTITUDE\n");
  }

  if(launch==true&&alt==true&&descConf==false){
    if(norm.YAxis<1&&desc==false){
      tiempoUno=millis();
      desc=true;
    }
    if(norm.YAxis<1&&desc==true){
      tiempoDos=millis()-tiempoUno;
      //Serial.println(tiempoDos);
    }
    if(tiempoDos>tiempoBandera){
      descConf=true;
      alturaMax=relativeAltitude+2;
      tiempoUno=millis();
    }
    else if(norm.YAxis>1){
      desc=false;
    }
  }

  if(descConf==true){
    launchData+=("DESCENDING\n");
    if(norm.YAxis>0.5*g&&relativeAltitude>alturaInicial+(alturaBandera/2)){
      //Serial.println("Post-Parachute");
      tiempoDos=(millis()-tiempoUno)/1000;
      /*
      Serial.println(tiempoDos);
      Serial.print("Max Alt: "); Serial.print(alturaMax); Serial.print(" Max Acel: "); Serial.print(acelMax2/g); Serial.print(" Altura Inicial: "); Serial.print(alturaInicial);
      Serial.print(" Tiempo de descenso: "); Serial.print(tiempoDos); Serial.print("m/s");
      */
      launchData+=("POST-PARACHUTE"); launchData+=("   Max Alt: "); launchData+=(alturaMax); launchData+=("m Max Accel: "); launchData+=(acelMax2/g); launchData+=("Gs Altura Inicial: ");
      launchData+=(alturaInicial); launchData+=("m Tiempo de descenso: "); launchData+=(tiempoDos); launchData+=("s");
    }

    else if(relativeAltitude<alturaInicial+(alturaBandera/2)){
      //Serial.println("Landed");
      velocidadDesc=(alturaMax-alturaInicial)/(tiempoDos);
      /*
      Serial.print("Max Alt: "); Serial.print(alturaMax); Serial.print(" Max Acel: "); Serial.print(acelMax2/g); Serial.print(" Altura Inicial: "); Serial.print(alturaInicial);
      Serial.print(" Velocidad Descenso: "); Serial.print(velocidadDesc);  Serial.print("m/s");
      */
      launchData+=("LANDED"); launchData+=("   Max Alt: "); launchData+=(alturaMax); launchData+=("m Max Accel: "); launchData+=(acelMax2/g); launchData+=("Gs Altura Inicial: ");
      launchData+=(alturaInicial); launchData+=("m Tiempo de descenso: "); launchData+=(tiempoDos); launchData+=("s Velocidad de descenso: "); launchData+=(velocidadDesc); launchData+=("m/s");
    }

    else if(relativeAltitude<alturaInicial+alturaBandera){
      launchData+=("PRE-PARACHUTE");
    }
  }
}

void telemetry(){
  data=(" X=");
  data+=(norm.XAxis);
  data+=(" Y=");
  data+=(norm.YAxis);
  data+=(" Z=");
  data+=(norm.ZAxis);
  data+=(" Pressure=");
  data+=(pressure);
  data+=("Pa absoluteAltitude=");
  data+=(absoluteAltitude);
  data+=("m relativeAltitude=");
  data+=(relativeAltitude);    

  //delay(500);

  if(counter==50){
  LoRa.beginPacket();
  LoRa.print(launchData);
  LoRa.endPacket();
  counter=0;
  }
  counter++;

  //Serial.println(data);
  Serial.println(launchData);
}
