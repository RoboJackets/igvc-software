// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_AHRS interface
//

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins( &adc );
#else
AP_InertialSensor_Stub ins;
#endif

AP_Compass_HMC5843 compass;

GPS *g_gps;

AP_GPS_Auto g_gps_driver(&g_gps);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(&ins, g_gps);
//AP_AHRS_MPU6000  ahrs(&ins, g_gps);		// only works with APM2

AP_Baro_BMP085_HIL barometer;


#define HIGH 1
#define LOW 0

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define A_LED_PIN        27
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD
#else
 # define A_LED_PIN        37
 # define C_LED_PIN        35
 # define LED_ON           HIGH
 # define LED_OFF          LOW
 # define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
#endif

static void flash_leds(bool on)
{
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

void setup() {
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
	// we need to stop the barometer from holding the SPI bus
	hal.gpio->pinMode(40, GPIO_OUTPUT);
	hal.gpio->write(40, HIGH);
#endif

	ins.init(AP_InertialSensor::COLD_START,
			 AP_InertialSensor::RATE_100HZ,
			 flash_leds);
	ins.init_accel(flash_leds);

	compass.set_orientation(MAG_ORIENTATION);
	ahrs.init();
	
	if( compass.init() ) {
		ahrs.set_compass(&compass);
	} else {
		hal.console->printf("No compass detected\n");
	}
	g_gps = &g_gps_driver;
#if WITH_GPS
	g_gps->init(hal.uartB);
#endif
}

Vector3f velocity;
uint32_t last_update;
bool first_loop = true;

void loop()
{
#if WITH_GPS
	g_gps->update();
#endif

	ahrs.update();

	Vector3f accelVals = ahrs.get_accel_ef();
	//Vector3f accelVals = ahrs.get_ins()->get_accel();
	accelVals.z += GRAVITY_MSS;
	Vector3f filteredVals;
	filteredVals.x = accelVals.x;//(abs(accelVals.x) > 0.05 ? accelVals.x : 0.0);
	filteredVals.y = accelVals.y;//(abs(accelVals.y) > 0.05 ? accelVals.y : 0.0);
	filteredVals.z = accelVals.z;//(abs(accelVals.z) > 0.05 ? accelVals.z : 0.0);
	
	uint32_t tnow = hal.scheduler->micros();
	float deltat = ( tnow - last_update ) / 1000000.0;
	if(first_loop)
	{
		first_loop = false;
		deltat = 0;
	}

	velocity.x += filteredVals.x * deltat;
	velocity.y += filteredVals.y * deltat;
	velocity.z += filteredVals.z * deltat;

	hal.console->printf_P(
			PSTR("%4.2f %4.2f %4.2f\n"),
			velocity.x,
			velocity.y,
			velocity.z);
	last_update = tnow;
}

AP_HAL_MAIN();
