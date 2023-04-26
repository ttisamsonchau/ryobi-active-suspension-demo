#pragma once
#include "share_param.h"
#include "JY901.h"

// imu object init
CJY901 JY901;

// zero point offset of the pitch & roll angle
float pitch_zero_offset = 0.0;
float roll_zero_offset = 0.0;
float yaw_zero_offset = 0.0;

// imu device init
void imu_init()
{
    JY901.StartIIC(&I2C);
    vTaskDelay(pdMS_TO_TICKS(1000)); // init time of the JY901/ JY601 required at least 80ms
}

// imu set current position as zero
void imu_set_zero(){
    JY901.GetAngle();
    pitch_zero_offset = ((float)JY901.stcAngle.Angle[0] / 32768 * 180);
    roll_zero_offset = ((float)JY901.stcAngle.Angle[1] / 32768 * 180);
    yaw_zero_offset = ((float)JY901.stcAngle.Angle[2] / 32768 * 180);
}

// imu update parameter
void imu_update()
{
    // reset the zero offset if set zero flag triggered
    lockVariable();
    if (imu.set_zero_flag == true){
        imu_set_zero();
        imu.set_zero_flag = false;
    }
    unlockVariable();

    // update parameter from imu
    JY901.GetAngle();
    JY901.GetAcc();
    JY901.GetGyro();
    JY901.GetMag();
    JY901.GetPress();

    // Filtered angle in deg
    // Accuracy: X, Y-axis: 0.05°
    //           Z-axis: 1°(after magnetic calibration)
    // Output
    // X, Z-axis: ±180°
    // Y ±90°
    // (Y-axis 90° is singular point)
    lockVariable();
    imu.roll = ((float)JY901.stcAngle.Angle[0] / 32768 * 180) - roll_zero_offset;
    imu.pitch = ((float)JY901.stcAngle.Angle[1] / 32768 * 180) - pitch_zero_offset;
    imu.yaw = ((float)JY901.stcAngle.Angle[2] / 32768 * 180) - yaw_zero_offset;

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

    // air pressure and estimate height
    // Output
    //  pressure: in Pa
    //  altitude: in m

    imu.pressure = ((float)JY901.stcPress.lPressure / 1000);
    imu.altitude = ((float)JY901.stcPress.lAltitude / 100);
    unlockVariable();
}