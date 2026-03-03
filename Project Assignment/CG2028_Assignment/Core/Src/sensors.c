#include "sensors.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"


const int N=4; // size of buffer

int accel_buff_x[4]={0};
int accel_buff_y[4]={0};
int accel_buff_z[4]={0};

float roll_pitch_yaw[3] = {0.0f};

// Sliding windows for recent accel/gyro magnitudes (for MAX-MIN range)
// Shorter window so gyro range "memory" decays faster after a spike
float acc_mag_window[FD_WINDOW_SIZE] = {0.0f};
float gyro_mag_window[FD_WINDOW_SIZE] = {0.0f};
int fd_win_idx = 0;
int fd_win_filled = 0;

// Pressure sensor (barometer) for optional "going up" detection
float last_pressure = 0.0f;
int pressure_ready = 0;

void sensors_init(void)
{
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_PSENSOR_Init();
    BSP_MAGNETO_Init();
}

void get_IMU_reading(int i, sensors_t *sensors)
{
    /* -------------------------------------- READ ACCELEROMETER VALUES AND PRE-PROCESS -------------------------------------- */
    int16_t accel_data_i16[3] = { 0 };						// Array to store the x, y and z readings of accelerometer
    BSP_ACCELERO_AccGetXYZ(accel_data_i16);					// Function to read accelerometer values
    
    // Copy the values over to a circular style buffer
    accel_buff_x[i%4]=accel_data_i16[0]; 					// Acceleration along X-Axis
    accel_buff_y[i%4]=accel_data_i16[1]; 					// Acceleration along Y-Axis
    accel_buff_z[i%4]=accel_data_i16[2]; 					// Acceleration along Z-Axis
    // Preprocessing the filtered outputs using moving average filter.
    float accel_filt_asm[3]={0}; 							// final value of filtered acceleration values
    accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
    accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
    accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);
    
    // Calculate magnitude of filtered accelerometer vector (AT_t)
    float accel_magnitude_asm = sqrtf(sqr(accel_filt_asm[0]) + sqr(accel_filt_asm[1]) + sqr(accel_filt_asm[2]));
    sensors->accel_magnitude_asm = accel_magnitude_asm;
    
    /* -------------------------------------- READ GYROSCOPE VALUES AND PRE-PROCESS -------------------------------------- */
    float gyro_data[3]={0.0};
    BSP_GYRO_GetXYZ(gyro_data);								// Function to read gyroscope values
    
    // The output of gyro has been made to display in dps(degree per second)
    float gyro_velocity[3]={0.0};
    gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
    gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
    gyro_velocity[2]=(gyro_data[2]*9.8/(1000));
    
    // Calculate magnitude of gyro vector (GT_t)
    float gyro_magnitude_asm = sqrtf(sqr(gyro_velocity[0]) + sqr(gyro_velocity[1]) + sqr(gyro_velocity[2]));
    sensors->gyro_magnitude_asm = gyro_magnitude_asm;
    
    // Update sliding windows and derive recent instability ranges for accel and gyro
    fd_update_windows(accel_magnitude_asm, gyro_magnitude_asm);
    float accel_recent_range = 0.0f;
    float gyro_recent_range  = 0.0f;
    fd_get_ranges(&accel_recent_range, &gyro_recent_range);
    sensors->accel_recent_range = accel_recent_range;
    sensors->gyro_recent_range = gyro_recent_range;
    
    /* -------------------------------------- READ MAGNETOMETER VALUES AND PRE-PROCESS -------------------------------------- */
    int16_t mag_raw[3] = {0};
    BSP_MAGNETO_GetXYZ(mag_raw);              // BSP expects int16_t* buffer
    float mag_xyz[3] = {0.0f};
    mag_xyz[0] = (float)mag_raw[0];
    mag_xyz[1] = (float)mag_raw[1];
    mag_xyz[2] = (float)mag_raw[2];
    
    /* -------------------------------------- COMPUTE ROLL & PITCH FROM ACCEL -------------------------------------- */
    float roll_rad  = atan2f(accel_filt_asm[1], hypotf(accel_filt_asm[0], accel_filt_asm[2]));
    float pitch_rad = atan2f(-accel_filt_asm[0], hypotf(accel_filt_asm[1], accel_filt_asm[2]));
    sensors->roll_pitch_yaw[0] = roll_rad  * 180.0f / (float)M_PI;
    sensors->roll_pitch_yaw[1] = pitch_rad * 180.0f / (float)M_PI;
    
    /* -------------------------------------- COMPUTE YAW WITH COUPLING EFFECT MITIGATION -------------------------------------- */
    // float original_magX = mag_xyz[0];
    // float original_magY = mag_xyz[1];
    // // float original_magZ = mag_xyz[2];
    
    // // compensate for roll
    // float polar_r_xz = sqrt(mag_xyz[0]*mag_xyz[0] + mag_xyz[2]*mag_xyz[2]);
    // // float theta_xz_init = atan2(mag_xyz[0],mag_xyz[2]);
    // // float polar_angle_after_roll = theta_xz_init - roll_rad;
    // mag_xyz[0] = polar_r_xz * cos(roll_rad);
    // mag_xyz[2] = polar_r_xz * sin(roll_rad);
    
    // // compensate for pitch
    // float polar_r_yz = sqrt(mag_xyz[1]*mag_xyz[1] + mag_xyz[2]*mag_xyz[2]);
    // // float theta_yz_init = atan2(mag_xyz[1],mag_xyz[2]);
    // // float polar_angle_after_pitch = theta_yz_init - pitch_rad;
    // mag_xyz[1] = polar_r_yz * cos(pitch_rad);
    // mag_xyz[2] = polar_r_yz * sin(pitch_rad);
    
    // float deltaX = mag_xyz[0] - original_magX;
    // float deltaY = mag_xyz[1] - original_magY;
    
    // float yaw = atan2(mag_xyz[0] - deltaX, mag_xyz[1] - deltaY);
    // sensors->roll_pitch_yaw[2] = yaw  * 180.0f/M_PI;

    // float roll = roll_rad  * 180.0f / (float)M_PI;
    // float pitch = pitch_rad * 180.0f / (float)M_PI;

    // float cosPhi = cos(roll);
    // float sinPhi = sin(roll);
    // float cosTheta = cos(pitch);
    // float sinTheta = sin(pitch);

    // float mX_rot = mag_xyz[0] * cosTheta + mag_xyz[2] * sinTheta;
    // float mY_rot = mag_xyz[0] * sinPhi * sinTheta + mag_xyz[1] * cosPhi - mag_xyz[2] * sinPhi * cosTheta;

    // float yaw = atan2(mY_rot, mX_rot) * 180.0f / M_PI;
    // sensors->roll_pitch_yaw[2] = yaw;

    float mx = mag_xyz[0];
    float my = mag_xyz[1];
    float mz = mag_xyz[2];

    float yaw = atan2(
        my * cos(roll_rad) - mz * sin(roll_rad),
        mx * cos(pitch_rad) + my * sin(roll_rad) * sin(pitch_rad) + mz * cos(roll_rad) * sin(pitch_rad)
    );

    sensors->roll_pitch_yaw[2] = yaw * 180.0f / M_PI;
}

void get_baro_reading(sensors_t *sensors)
{
    /* -------------------------------------- BAROMETER (PRESSURE SENSOR) -------------------------------------- */
    sensors->pressure_hpa = BSP_PSENSOR_ReadPressure();
    float dp_hpa = 0.0f;
    if (pressure_ready)
    {
        dp_hpa = sensors->pressure_hpa - last_pressure;
    }
    else
    {
        pressure_ready = 1;
    }
    last_pressure = sensors->pressure_hpa;
    sensors->dp_hpa = dp_hpa;
}


// Window used to determine instability based on recent accel/gyro magnitudes.
// Updated every iteration and used to derive recent range for fall detection.
// Uses a simple circular buffer approach.
void fd_update_windows(float acc_mag, float gyro_mag)
{
	acc_mag_window[fd_win_idx]  = acc_mag;
	gyro_mag_window[fd_win_idx] = gyro_mag;
	fd_win_idx = (fd_win_idx + 1) % FD_WINDOW_SIZE;
	if (fd_win_idx == 0)
	{
		fd_win_filled = 1;
	}
}

void fd_get_ranges(float *acc_range, float *gyro_range)
{
	int count = fd_win_filled ? FD_WINDOW_SIZE : fd_win_idx;
	if (count <= 0)
	{
		*acc_range = 0.0f;
		*gyro_range = 0.0f;
		return;
	}

	float acc_min = acc_mag_window[0];
	float acc_max = acc_mag_window[0];
	float gyro_min = gyro_mag_window[0];
	float gyro_max = gyro_mag_window[0];

	for (int k = 1; k < count; ++k)
	{
		if (acc_mag_window[k] < acc_min) acc_min = acc_mag_window[k];
		if (acc_mag_window[k] > acc_max) acc_max = acc_mag_window[k];
		if (gyro_mag_window[k] < gyro_min) gyro_min = gyro_mag_window[k];
		if (gyro_mag_window[k] > gyro_max) gyro_max = gyro_mag_window[k];
	}

	*acc_range = acc_max - acc_min;
	*gyro_range = gyro_max - gyro_min;
}
    
    

/**
 * @brief  C implementation of the moving average filter. This is a reference implementation and is not optimized for performance.
 * @param  N: Number of samples to average (window size)
 * @param  accel_buff: Pointer to the buffer containing the accelerometer readings
 * @return The average of the last N samples in the buffer
 */
int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result = 0;
	for(int i = 0; i < N; i ++)
	{
		result += accel_buff[i];
	}
	result = result / 4;
	return result;
}
