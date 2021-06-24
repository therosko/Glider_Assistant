//*********************************************************************
// Tiny Glider Assistant
// by Rositsa Ivanova
//*********************************************************************

//BME280
#include <Wire.h>
#include "BlueDot_BME280.h"
//LCD
//www.diyusthad.com
#include <LiquidCrystal.h>
//NEO-6M GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//BME280 Sensor
BlueDot_BME280 bme;                                     //Object for Sensor 2
int bmeDetected = 0;                                    //Checks if Sensor 2 is available

//LCD Display
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//NEO-6M GPS
TinyGPS gps;
SoftwareSerial ss(9,10);

//*********************************************************************
//GPS distance measurement
//Note: This section is taken from geodatasource.com
//The sample code is licensed under LGPLv3.
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::                                                                         :*/
/*::  This routine calculates the distance between two points (given the     :*/
/*::  latitude/longitude of those points). It is being used to calculate     :*/
/*::  the distance between two locations using GeoDataSource(TM) products.   :*/
/*::                                                                         :*/
/*::  Definitions:                                                           :*/
/*::    South latitudes are negative, east longitudes are positive           :*/
/*::                                                                         :*/
/*::  Passed to function:                                                    :*/
/*::    lat1, lon1 = Latitude and Longitude of point 1 (in decimal degrees)  :*/
/*::    lat2, lon2 = Latitude and Longitude of point 2 (in decimal degrees)  :*/
/*::    unit = the unit you desire for results                               :*/
/*::           where: 'M' is statute miles (default)                         :*/
/*::                  'K' is kilometers                                      :*/
/*::                  'N' is nautical miles                                  :*/
/*::  Worldwide cities and other features databases with latitude longitude  :*/
/*::  are available at https://www.geodatasource.com                         :*/
/*::                                                                         :*/
/*::  For enquiries, please contact sales@geodatasource.com                  :*/
/*::                                                                         :*/
/*::  Official Web site: https://www.geodatasource.com                       :*/
/*::                                                                         :*/
/*::           GeoDataSource.com (C) All Rights Reserved 2018                :*/
/*::                                                                         :*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#include <math.h>

#define pi 3.14159265358979323846

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  Function prototypes                                           :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double);
double rad2deg(double);

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  if ((lat1 == lat2) && (lon1 == lon2)) {
    return 0;
  }
  else {
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    switch(unit) {
      case 'M':
        break;
      case 'K':
        dist = dist * 1.609344;
        break;
      case 'N':
        dist = dist * 0.8684;
        break;
    }
    return (dist);
  }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) {
  return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) {
  return (rad * 180 / pi);
}

//End of section from geodatasource.com
//*********************************************************************


void setup() {
  /***************************************************************************
  Example for BME280 Weather Station using two Sensors with I2C Communication
  written by Thiago Barros for BlueDot UG (haftungsbeschränkt)
  BSD License

  This sketch was written for the Bosch Sensor BME280.
  The BME280 is a MEMS device for measuring temperature, humidity and atmospheric pressure.
  For more technical information on the BME280, please go to ------> http://www.bluedot.space
 ***************************************************************************/
  Serial.begin(9600);
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.println(F("BME 280 code is adapted from the example Basic Weather Station"));

  //BME 280
  //This program is set for the I2C mode
    bme.parameter.communication = 0;                    //I2C communication for Sensor 2 (bme2)
  //Set the I2C address of your breakout board  
    bme.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 2 (bme2)
  //0b11:     In normal mode the sensor measures continually (default value)
    bme.parameter.sensorMode = 0b11;                    //Setup Sensor mode (default)  
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
    bme.parameter.IIRfilter = 0b100;                   //IIR Filter (default)
  //Next you'll define the oversampling factor for the humidity measurements
    bme.parameter.humidOversampling = 0b101;            //Humidity Oversampling (default)
  //Now define the oversampling factor for the temperature measurements
    bme.parameter.tempOversampling = 0b101;              //Temperature Oversampling (default)
  //Finally, define the oversampling factor for the pressure measurements
  //For altitude measurements a higher factor provides more stable values
    bme.parameter.pressOversampling = 0b101;             //Pressure Oversampling (defualt)
  //For precise altitude measurements please put in the current pressure corrected for the sea level
  //On doubt, just leave the standard pressure as default (1013.25 hPa);
    bme.parameter.pressureSeaLevel = 1013.25;           
  //Also put in the current average temperature outside (yes, really outside!)
  //For slightly less precise altitude measurements, just leave the standard temperature as default (15°C and 59°F);
    bme.parameter.tempOutsideCelsius = 15;               
    bme.parameter.tempOutsideFahrenheit = 59;           

  if (bme.init() != 0x60)
  {    
    Serial.println(F("Ops! BME280 Sensor not found!"));
    bmeDetected = 0;
  }

  else
  {
    Serial.println(F("BME280 Sensor detected!"));
    bmeDetected = 1;
  }

  if ((bmeDetected == 0))
  {
    Serial.println();
    Serial.println();
    Serial.println(F("Troubleshooting Guide"));
    Serial.println(F("*************************************************************"));
    Serial.println(F("1. Let's check the basics: Are the VCC and GND pins connected correctly? If the BME280 is getting really hot, then the wires are crossed."));
    Serial.println();
    Serial.println(F("2. Did you connect the SDI pin from your BME280 to the SDA line from the Arduino?"));
    Serial.println();
    Serial.println(F("3. And did you connect the SCK pin from the BME280 to the SCL line from your Arduino?"));
    Serial.println();
    Serial.println(F("4. One of your sensors should be using the alternative I2C Address(0x76). Did you remember to connect the SDO pin to GND?"));
    Serial.println();
    Serial.println(F("5. The other sensor should be using the default I2C Address (0x77. Did you remember to leave the SDO pin unconnected?"));
    Serial.println();
    while(1);
  }  
  Serial.println();
  Serial.println();
}

void loop() {
   
  Serial.print(F("Duration in Seconds:  "));
  Serial.println(float(millis())/1000);
 
  if (bmeDetected)
  {
    // For debuggin purposes
    Serial.print(F("Temperature Sensor [°C]:\t\t")); 
    Serial.println(bme.readTempC());
    //Serial.print(F("Temperature Sensor [°F]:\t\t")); 
    //Serial.println(bme2.readTempF());
    Serial.print(F("Humidity Sensor [%]:\t\t\t")); 
    Serial.println(bme.readHumidity());
    Serial.print(F("Pressure Sensor [hPa]:\t\t")); 
    Serial.println(bme.readPressure());
    Serial.print(F("Altitude Sensor [m]:\t\t\t")); 
    Serial.println(bme.readAltitudeMeter());
    //Serial.print(F("Altitude Sensor [ft]:\t\t\t")); 
    //Serial.println(bme2.readAltitudeFeet()); 
    lcd.begin(16, 2);
    lcd.print((int)bme.readTempC());
    lcd.print((char)223);
    lcd.print("C,"); 
    lcd.print((int)bme.readPressure());
    lcd.print("hP,");
    lcd.print((int)bme.readAltitudeMeter());
    lcd.print("m");
    }

  else
  {
    // For debuggin purposes
    Serial.print(F("Temperature Sensor [°C]:\t\t")); 
    Serial.println(F("Null"));
    //Serial.print(F("Temperature Sensor [°F]:\t\t")); 
    //Serial.println(F("Null"));
    Serial.print(F("Humidity Sensor [%]:\t\t\t")); 
    Serial.println(F("Null"));
    Serial.print(F("Pressure Sensor [hPa]:\t\t")); 
    Serial.println(F("Null"));
    Serial.print(F("Altitude Sensor [m]:\t\t\t")); 
    Serial.println(F("Null"));
    //Serial.print(F("Altitude Sensor [ft]:\t\t\t")); 
    //Serial.println(F("Null"));
  }

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  Serial.print("Looping, no data");

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    //for debugging
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

    lcd.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    lcd.print(" ");
    lcd.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //lcd.print(distance(48.208354, 16.372504, 47.807086, 16.23326, 'K'));
    lcd.setCursor(0,1);
    //Note: this could be an issue, if one of the directions is close to lon, lat of 0.00, 0.00. For Europe, this is not an issue.
    double curr_lat = (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat);
    double curr_lon = (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon);
    
    // calculation of shortest distance
    char *names[] = {"LOGP", "LOWW", "LOAG", "LOXN"};
    char *frequency[] = {"122.855", "119.40", "125.110", "123.25"};
    int arr[4] = {distance(curr_lat, curr_lon, 47.386660, 16.113100, 'K'), distance(curr_lat, curr_lon, 48.105964, 16.574261, 'K'), distance(curr_lat, curr_lon, 48.446055, 15.630925, 'K'), distance(curr_lat, curr_lon, 47.838419, 16.223777, 'K')}; 
    //Calculate length of array arr    
    int length = sizeof(arr)/sizeof(arr[0]);    
        
    //Initialize min with first element of array.    
    int min = arr[0];
    char *loc = names[0];
    char *freq = frequency[0];  
        
    //Loop through the array    
    for (int i = 0; i < length; i++) {     
        //Compare elements of array with min    
        if(arr[i] < min)  {   
            min = arr[i];
            loc = names[i];
            freq = frequency[i];
        }
    } 
    Serial.println(min);
    Serial.println(loc);
    lcd.setCursor(0,1);
    lcd.print(min);
    lcd.print("km,");
    lcd.print(loc);
    lcd.print(",");
    lcd.print(freq);
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
    lcd.setCursor(0,1);
    lcd.print("GPS Searching...");
   
   Serial.println();
   Serial.println();

   delay(1000);
   
 
}
