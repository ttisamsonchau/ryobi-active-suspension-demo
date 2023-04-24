#pragma once
#include "share_param.h"
#include "JY901.h"

// imu object init
CJY901 JY901;

// imu device init
void imu_init()
{
    JY901.StartIIC(&I2C);
    vTaskDelay(pdMS_TO_TICKS(1000)); // init time of the JY901/ JY601 required at least 80ms
}

// imu update parameter
void imu_update()
{
    JY901.GetAngle();
    JY901.GetAcc();
    JY901.GetGyro();
    JY901.GetMag();
    JY901.GetPress();

    // Filtered angle in deg
    // Accuracy:X, Y-axis: 0.05°
    //         Z-axis: 1°(after magnetic
    //         calibration)
    // Output
    // X, Z-axis: ±180°
    // Y ±90°
    // (Y-axis 90° is singular point)
    lockVariable();
    imu.roll = ((float)JY901.stcAngle.Angle[0] / 32768 * 180);
    imu.pitch = ((float)JY901.stcAngle.Angle[1] / 32768 * 180);
    imu.yaw = ((float)JY901.stcAngle.Angle[2] / 32768 * 180);

    // Accelerometer
    // Accuracy: 0.01g
    // Resolution: 16bit
    // Stability: 0.005g
    // Output
    // ±16g

    imu.ax = ((float)JY901.stcAcc.a[0] / 32768 * 16);
    imu.ay = ((float)JY901.stcAcc.a[1] / 32768 * 16);
    imu.az = ((float)JY901.stcAcc.a[2] / 32768 * 16);

    // Gyroscope
    // Resolution: 16bit
    // Stability: 0.05°/s
    // Output
    // -±2000°/s

    imu.gx = ((float)JY901.stcGyro.w[0] / 32768 * 2000);
    imu.gy = ((float)JY901.stcGyro.w[1] / 32768 * 2000);
    imu.gz = ((float)JY901.stcGyro.w[2] / 32768 * 2000);

    // Magnometer
    // 0.15μT/LSB typ. (16-bit)
    // Output
    // ±4900μT

    imu.mx = (JY901.stcMag.h[0]);
    imu.my = (JY901.stcMag.h[1]);
    imu.mz = (JY901.stcMag.h[2]);

    // JY901.GetLonLat();
    // lattitude = ((double)(JY901.stcLonLat.lLat % 10000000)/1e5);
    // longitude = ((double)(JY901.stcLonLat.lLon % 10000000)/1e5);

    // air pressure and estimate height
    // Output
    //  pressure: in Pa
    //  altitude: in m

    imu.pressure = ((float)JY901.stcPress.lPressure / 1000);
    imu.altitude = ((float)JY901.stcPress.lAltitude / 100);
    unlockVariable();
}