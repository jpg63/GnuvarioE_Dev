/* VarioImuTwoWire -- 
 *
 * Copyright 2020 MichelPa / Jpg63
 * 
 * This file is part of GnuVario-E.
 *
 * ToneHAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ToneHAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/* 
 *********************************************************************************
 *                                                                               *
 *                          VarioImuTwoWire                                      *
 *                                                                               *
 *  version    Date     Description                                              *
 *    1.0    22/03/20                                                            *
 *    1.0.1  25/03/20   Ajout haveMeasure(void)																	 *
 *    1.0.2  25/12/20   Modif getCap                                             *
 *                                                                               *
 *********************************************************************************
 */
 
#include <Arduino.h> 
#include <HardwareConfig.h>
#include <DebugConfig.h>
#include <VarioLog.h>

#include "VarioImuTwoWire.h"
#include "VarioData.h"

#ifdef TWOWIRESCHEDULER


#ifdef HAVE_BMP280
Bmp280 TWScheduler::bmp280;
#else
Ms5611 TWScheduler::ms5611;
#endif

#ifdef HAVE_ACCELEROMETER
Vertaccel TWScheduler::vertaccel;
#endif //HAVE_ACCELEROMETER

#include <math.h>

#define R2D 57.2958

//**********************************
VarioImuTwoWire::VarioImuTwoWire()
//**********************************
{
}

//**********************************
void VarioImuTwoWire::init()
//**********************************
{
    /**************************/
    /* init Two Wires devices */
    /**************************/
    //!!!
#ifdef HAVE_ACCELEROMETER
    intTW.begin();
    twScheduler.init();
    //  vertaccel.init();

#endif //HAVE_ACCELEROMETER
}

//**********************************
bool VarioImuTwoWire::havePressure(void)
//**********************************
{
	return(twScheduler.havePressure());
}


//**********************************
bool VarioImuTwoWire::updateData(void)
//**********************************
{
#ifdef HAVE_ACCELEROMETER
  if (twScheduler.havePressure() && twScheduler.haveAccel())
  {
		CompteurAccel = 0;
		twScheduler.resetNewAccel();
    twScheduler.getTempAlti(Temp, Alti);
    Temp += GnuSettings.COMPENSATION_TEMP; //MPU_COMP_TEMP;
    Accel = twScheduler.getAccel(NULL);
		
#ifdef DATA_DEBUG
    SerialPort.print("VarioImuTwoWire Update");
    SerialPort.print("Alti : ");
    SerialPort.println(Alti);
    SerialPort.print("Temperature : ");
    SerialPort.println(Temp);
    SerialPort.print("Accel : ");
    SerialPort.println(Accel);
#endif //DATA_DEBUG
				
		return true;
	} 
	else if (twScheduler.haveNewAccel())
	{
		CompteurAccel++;
		twScheduler.resetNewAccel();
		if (CompteurAccel > 100) {
			CompteurAccel = 0;
			twScheduler.resetNewAccel();
			MESSLOG(LOG_TYPE_DEBUG,MS5611_DEBUG_LOG,"ERREUR MPU");
			MESSLOG(LOG_TYPE_DEBUG,MS5611_DEBUG_LOG,"AUCUNE MESURE MS5611");       
		}
	}
#else //HAVE_ACCELEROMETER
	
  if (twScheduler.havePressure())
  {

#ifdef MS5611_DEBUG
//    SerialPort.println("havePressure");
#endif //MS5611_DEBUG

    twScheduler.getTempAlti(Temp, Alti);
    Temp += GnuSettings.COMPENSATION_TEMP; //MPU_COMP_TEMP;
		Accel = 0;
		
#ifdef DATA_DEBUG
    SerialPort.print("Alti : ");
    SerialPort.println(Alti);
    SerialPort.print("Temperature : ");
    SerialPort.println(Temp);
    SerialPort.print("Accel : ");
    SerialPort.println(Accel);
#endif //DATA_DEBUG
		
		return true;
	}
#endif

/*  Temp = 0;
	Alti = 0;
	Accel =0;
	
#ifdef DATA_DEBUG
	SerialPort.println("ERREUR ACQUISITION MS5611/MPU");
	SerialPort.print("Alti : ");
	SerialPort.println(Alti);
	SerialPort.print("Temperature : ");
	SerialPort.println(Temp);
	SerialPort.print("Accel : ");
	SerialPort.println(Accel);
#endif //DATA_DEBUG*/
	
	return false;
}

//**********************************
void VarioImuTwoWire::updateAlti()
//**********************************
{
  Alti = twScheduler.getAlti();
}

//**********************************
double VarioImuTwoWire::getAlti()
//**********************************
{
  return Alti; //twScheduler.getAlti();
}

//**********************************
double VarioImuTwoWire::getTemp()
//**********************************
{
  return Temp; //twScheduler.getAlti();
}

//**********************************
double VarioImuTwoWire::getAccel()
//**********************************
{
  return Accel; //twScheduler.getAlti();
}

/*******************************************/
int VarioImuTwoWire::getCap(void) {
/*******************************************/

	int bearing;
	
	TRACE();
	if (twScheduler.haveAccel() ) {
		double vertVector[3];
		double vertAccel = twScheduler.getAccel(vertVector);
		
/*  NEW */

		// accelerometer and magnetometer data 
		float a, ax, ay, az, h, hx, hy, hz;

    ax = vertVector[0];
    ay = vertVector[1];
    az = vertVector[2];

    // Normalize accelerometer and magnetometer data 
    a = sqrtf(ax * ax + ay * ay + az * az);
    ax /= a;
    ay /= a;
    az /= a;

#ifdef BEARING_DEBUG
    SerialPort.print("ax : ");
    SerialPort.println(ax);
    SerialPort.print("ay : ");
    SerialPort.println(ay);
    SerialPort.print("az : ");
    SerialPort.println(az);
#endif //DATA_DEBUG

/*

  static void getRawAccel(int16_t* rawAccel, int32_t* quat);
  static double getAccel(double* vertVector); //vertVector = NULL if not needed
  static void getRawMag(int16_t* rawMag);


/ accelerometer and magnetometer data 
float a, ax, ay, az, h, hx, hy, hz;
// magnetometer calibration data 
float hxb, hxs, hyb, hys, hzb, hzs;
// euler angles 
float pitch_rad, roll_rad, yaw_rad, heading_rad;
// filtered heading 
float filtered_heading_rad;

    ax = imu.getAccelX_mss();
    ay = imu.getAccelY_mss();
    az = imu.getAccelZ_mss();
    hx = imu.getMagX_uT();
    hy = imu.getMagY_uT();
    hz = imu.getMagZ_uT();
    // Normalize accelerometer and magnetometer data 
    a = sqrtf(ax * ax + ay * ay + az * az);
    ax /= a;
    ay /= a;
    az /= a;

    h = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= h;
    hy /= h;
    hz /= h;
    // Compute euler angles 
    pitch_rad = asinf(ax);
    roll_rad = asinf(-ay / cosf(pitch_rad));
    yaw_rad = atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad), hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) + hz * sinf(pitch_rad) * cosf(roll_rad));
    heading_rad = constrainAngle360(yaw_rad);
    // Filtering heading 
    filtered_heading_rad = (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;
    // Display the results 
    Serial.print(pitch_rad * R2D);
    Serial.print("\t");
    Serial.print(roll_rad * R2D);
    Serial.print("\t");
    Serial.print(yaw_rad * R2D);
    Serial.print("\t");
    Serial.print(heading_rad * R2D);
    Serial.print("\t");
    Serial.println(filtered_heading_rad * R2D);
  }
}

// Bound angle between 0 and 360 
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}
*/		
		
		if (twScheduler.haveMag() ) {
/*			double northVector[2];
			twScheduler.getNorthVector(vertVector,  northVector);
			 
			double norm = sqrt(northVector[0]*northVector[0]+northVector[1]*northVector[1]);
			northVector[0] = northVector[0]/norm;
			northVector[1] = northVector[1]/norm;
			
			DUMP(northVector[0]);
			DUMP(northVector[1]);
			
			int tmpcap = 180 - atan2(northVector[1],northVector[0]) * 180/M_PI;
			
//			tmpcap = 360 - tmpcap;
			
			DUMP(tmpcap);*/
			
			int16_t magVector[3];
			int tmpcap;
			twScheduler.getRawMag(magVector);

			// magnetometer calibration data 
		//	float hxb, hxs, hyb, hys, hzb, hzs;

			hx = magVector[0];
			hy = magVector[1];
			hz = magVector[2];

			h = sqrtf(hx * hx + hy * hy + hz * hz);
			hx /= h;
			hy /= h;
			hz /= h;

#ifdef BEARING_DEBUG
      SerialPort.print("hx : ");
      SerialPort.println(hx);
      SerialPort.print("hy : ");
      SerialPort.println(hy);
      SerialPort.print("hz : ");
      SerialPort.println(hz);
#endif //DATA_DEBUG

			// Compute euler angles 
			float pitch_rad, roll_rad, yaw_rad, heading_rad;

			pitch_rad = asinf(ax);
			roll_rad = asinf(-ay / cosf(pitch_rad));
			yaw_rad = atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad), hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) + hz * sinf(pitch_rad) * cosf(roll_rad));
			heading_rad = constrainAngle360(yaw_rad);

#ifdef BEARING_DEBUG
      SerialPort.print("pitch_rad : ");
      SerialPort.println(pitch_rad);
      SerialPort.print("roll_rad : ");
      SerialPort.println(roll_rad);
      SerialPort.print("yaw_rad : ");
      SerialPort.println(yaw_rad);
      SerialPort.print("heading_rad : ");
      SerialPort.println(heading_rad);
#endif //DATA_DEBUG
			
			tmpcap = heading_rad * R2D;
 			
#ifdef BEARING_DEBUG
      SerialPort.print("Compas magnetique : ");
      SerialPort.println(tmpcap);
#endif //DATA_DEBUG
			
//Moyenne
			
/*			if (nbMesureCap < 10) {
				if (moyCap == -1) moyCap = 0;
				moyCap += tmpcap;
				nbMesureCap++;
				DUMP(nbMesureCap);
			}
			else {
				moyCap += tmpcap;
				DUMP(moyCap);
				bearing = moyCap / 10;
				moyCap = 0;
				nbMesureCap = 0;				
			}*/
			
			bearing = tmpcap;
			 
			DUMP(bearing); 
//			return bearing;
		}
		else {
			bearing = -1;
			TRACE();
			return 0;
		}
	} 
	else {
		bearing = -1;
		TRACE();
		return 0;
	}
	
	if (bearing > 360) bearing = bearing - 360;
	if (bearing < 0)   bearing = 360 + bearing;
	return bearing;
}

// Bound angle between 0 and 360 
/*******************************************/
float VarioImuTwoWire::constrainAngle360(float dta) {
/*******************************************/
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}
#endif // TWOWIRESCHEDULER
