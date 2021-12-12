#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>


// physical constants
#define MPU9250_G                     9.807f
#define MPU9250_D2R                   3.14159265359f/180.0f

// MPU9250 registers
#define MPU9250_ACCEL_OUT             0x3B
#define MPU9250_GYRO_OUT              0x43
#define MPU9250_TEMP_OUT              0x41
#define MPU9250_EXT_SENS_DATA_00      0x49
#define MPU9250_ACCEL_CONFIG 	      0x1C
#define MPU9250_ACCEL_FS_SEL_2G       0x00
#define MPU9250_ACCEL_FS_SEL_4G       0x08
#define MPU9250_ACCEL_FS_SEL_8G       0x10
#define MPU9250_ACCEL_FS_SEL_16G      0x18
#define MPU9250_GYRO_CONFIG           0x1B
#define MPU9250_GYRO_FS_SEL_250DPS    0x00
#define MPU9250_GYRO_FS_SEL_500DPS    0x08
#define MPU9250_GYRO_FS_SEL_1000DPS   0x10
#define MPU9250_GYRO_FS_SEL_2000DPS   0x18

#define MPU9250_ACCEL_CONFIG2         0x1D
#define MPU9250_ACCEL_DLPF_184        0x01
#define MPU9250_ACCEL_DLPF_92         0x02
#define MPU9250_ACCEL_DLPF_41         0x03
#define MPU9250_ACCEL_DLPF_20         0x04
#define MPU9250_ACCEL_DLPF_10         0x05
#define MPU9250_ACCEL_DLPF_5          0x06
#define MPU9250_CONFIG                0x1A
#define MPU9250_GYRO_DLPF_184         0x01
#define MPU9250_GYRO_DLPF_92          0x02
#define MPU9250_GYRO_DLPF_41          0x03
#define MPU9250_GYRO_DLPF_20          0x04
#define MPU9250_GYRO_DLPF_10          0x05
#define MPU9250_GYRO_DLPF_5           0x06
#define MPU9250_SMPDIV                0x19
#define MPU9250_INT_PIN_CFG           0x37
#define MPU9250_INT_ENABLE            0x38
#define MPU9250_INT_DISABLE           0x00
#define MPU9250_INT_PULSE_50US        0x00
#define MPU9250_INT_WOM_EN            0x40
#define MPU9250_INT_RAW_RDY_EN        0x01
#define MPU9250_PWR_MGMNT_1           0x6B
#define MPU9250_PWR_CYCLE             0x20
#define MPU9250_PWR_RESET             0x80
#define MPU9250_CLOCK_SEL_PLL         0x01
#define MPU9250_PWR_MGMNT_2           0x6C
#define MPU9250_SEN_ENABLE            0x00
#define MPU9250_DIS_GYRO              0x07
#define MPU9250_USER_CTRL             0x6A
#define MPU9250_I2C_MST_EN            0x20
#define MPU9250_I2C_MST_CLK           0x0D
#define MPU9250_I2C_MST_CTRL          0x24
#define MPU9250_I2C_SLV0_ADDR         0x25
#define MPU9250_I2C_SLV0_REG          0x26
#define MPU9250_I2C_SLV0_DO           0x63
#define MPU9250_I2C_SLV0_CTRL         0x27
#define MPU9250_I2C_SLV0_EN           0x80
#define MPU9250_I2C_READ_FLAG         0x80
#define MPU9250_MOT_DETECT_CTRL       0x69
#define MPU9250_ACCEL_INTEL_EN        0x80
#define MPU9250_ACCEL_INTEL_MODE      0x40
#define MPU9250_LP_ACCEL_ODR          0x1E
#define MPU9250_WOM_THR               0x1F
#define MPU9250_WHO_AM_I              0x75
#define MPU9250_FIFO_EN               0x23
#define MPU9250_FIFO_TEMP             0x80
#define MPU9250_FIFO_GYRO             0x70
#define MPU9250_FIFO_ACCEL            0x08
#define MPU9250_FIFO_MAG              0x01
#define MPU9250_FIFO_COUNT            0x72
#define MPU9250_FIFO_READ             0x74

// AK8963 registers
#define MPU9250_AK8963_I2C_ADDR       0x0C
#define MPU9250_AK8963_HXL            0x03
#define MPU9250_AK8963_CNTL1          0x0A
#define MPU9250_AK8963_PWR_DOWN       0x00
#define MPU9250_AK8963_CNT_MEAS1      0x12
#define MPU9250_AK8963_CNT_MEAS2      0x16
#define MPU9250_AK8963_FUSE_ROM       0x0F
#define MPU9250_AK8963_CNTL2          0x0B
#define MPU9250_AK8963_RESET          0x01
#define MPU9250_AK8963_ASA            0x10
#define MPU9250_AK8963_WHO_AM_I       0x00


//Different options for basic MPU9250 setting registers
typedef enum
{
   MPU9250_ACCEL_RANGE_2G,
   MPU9250_ACCEL_RANGE_4G,
   MPU9250_ACCEL_RANGE_8G,
   MPU9250_ACCEL_RANGE_16G
} MPU9250_AccelRange_t;

typedef enum
{
   MPU9250_GYRO_RANGE_250DPS,
   MPU9250_GYRO_RANGE_500DPS,
   MPU9250_GYRO_RANGE_1000DPS,
   MPU9250_GYRO_RANGE_2000DPS
} MPU9250_GyroRange_t;

typedef enum
{
   MPU9250_DLPF_BANDWIDTH_184HZ,
   MPU9250_DLPF_BANDWIDTH_92HZ,
   MPU9250_DLPF_BANDWIDTH_41HZ,
   MPU9250_DLPF_BANDWIDTH_20HZ,
   MPU9250_DLPF_BANDWIDTH_10HZ,
   MPU9250_DLPF_BANDWIDTH_5HZ
} MPU9250_DlpfBandwidth_t;

typedef enum
{
   MPU9250_LP_ACCEL_ODR_0_24HZ  = 0,
   MPU9250_LP_ACCEL_ODR_0_49HZ  = 1,
   MPU9250_LP_ACCEL_ODR_0_98HZ  = 2,
   MPU9250_LP_ACCEL_ODR_1_95HZ  = 3,
   MPU9250_LP_ACCEL_ODR_3_91HZ  = 4,
   MPU9250_LP_ACCEL_ODR_7_81HZ  = 5,
   MPU9250_LP_ACCEL_ODR_15_63HZ = 6,
   MPU9250_LP_ACCEL_ODR_31_25HZ = 7,
   MPU9250_LP_ACCEL_ODR_62_50HZ = 8,
   MPU9250_LP_ACCEL_ODR_125HZ   = 9,
   MPU9250_LP_ACCEL_ODR_250HZ   = 10,
   MPU9250_LP_ACCEL_ODR_500HZ   = 11
} MPU9250_LpAccelOdr_t;

//Control structure for MPU9250 operation (only one IMU per project)
typedef struct {
   // scale factors
   float _accelScale;
   float _gyroScale;
   float _magScaleX;
   float _magScaleY;
   float _magScaleZ;
   float _tempScale;
   float _tempOffset;

   // configuration
   MPU9250_AccelRange_t    _accelRange;
   MPU9250_GyroRange_t     _gyroRange;
   MPU9250_DlpfBandwidth_t _bandwidth;
   unsigned char _srd;

   // buffer for reading from sensor
   unsigned char _buffer[21];

   // data buffer
   float _ax, _ay, _az;
   float _gx, _gy, _gz;
   float _hx, _hy, _hz;
   float _t;

   // gyro bias estimation
   unsigned char _numSamples;
   double _gxbD, _gybD, _gzbD;
   float _gxb, _gyb, _gzb;

   // accel bias and scale factor estimation
   double _axbD, _aybD, _azbD;
   float _axmax, _aymax, _azmax;
   float _axmin, _aymin, _azmin;
   float _axb, _ayb, _azb;
   float _axs;
   float _ays;
   float _azs;

   // magnetometer bias and scale factor estimation
   unsigned short _maxCounts;
   float _deltaThresh;
   unsigned char _coeff;
   unsigned short _counter;
   float _framedelta, _delta;
   float _hxfilt, _hyfilt, _hzfilt;
   float _hxmax, _hymax, _hzmax;
   float _hxmin, _hymin, _hzmin;
   float _hxb, _hyb, _hzb;
   float _hxs;
   float _hys;
   float _hzs;
   float _avgs;

   // data counts
   short _axcounts, _aycounts, _azcounts;
   short _gxcounts, _gycounts, _gzcounts;
   short _hxcounts, _hycounts, _hzcounts;
   short _tcounts;

   // transformation matrix
   /* transform the accel and gyro axes to match the magnetometer axes */
   short tX[3];
   short tY[3];
   short tZ[3];

   // track success of interacting with sensor
   bool _status;

} MPU9250_control_t;


static MPU9250_control_t handler;
int mpu9250 = 0;

static bool mpu9250ReadRegisters(unsigned char subAddress, unsigned char count);
static bool mpu9250WriteRegister(unsigned char subAddress, unsigned char data);
static char mpu9250SetSrd(unsigned char srd);
static char mpu9250SetDlpfBandwidth(MPU9250_DlpfBandwidth_t bandwidth);
static bool mpu9250SetGyroRange(MPU9250_GyroRange_t range);
static char mpu9250CalibrateGyro(void);
static char mpu9250WhoAmIAK8963(void);
static char mpu9250WhoAmI(void);
static char mpu9250ReadAK8963Registers(unsigned char subAddress, unsigned char data);
static char mpu9250WriteAK8963Register(unsigned char subAddress, unsigned char data);
static char mpu9250InitializeControlStructure(void);
static char mpu9250Init(void);
static bool mpu9250Read(void);
static float mpu9250GetGyroX_rads(void);
static float mpu9250GetGyroY_rads(void);
static float mpu9250GetGyroZ_rads(void);


static bool mpu9250ReadRegisters(unsigned char subAddress, unsigned char count)
{
	
	if (write(mpu9250, &subAddress, 1) < 0) {
		printf("Error mpu9250ReadRegisters on writing operationp\n ");
		return false;
	}
	
	if (read(mpu9250, handler._buffer, count) < 0) {
		printf("Error mpu9250ReadRegisters on reading operationp\n ");
		return false;
	}

	return true;
}

static bool mpu9250WriteRegister(unsigned char subAddress, unsigned char data)
{
	unsigned char  transmitDataBuffer[2];

	transmitDataBuffer[0] = subAddress;
	transmitDataBuffer[1] = data;

	if (write(mpu9250, transmitDataBuffer, 2)) {
		printf("Error writing for I2C\n");
		return false;
	}	
	usleep(10000);

	mpu9250ReadRegisters(subAddress,1);
	
	/* check the read back register against the written register */
	if(handler._buffer[0] == data) {
      		return true;
	}
	else{
      		return false;
	}
}

static char mpu9250SetSrd(unsigned char srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up 
      magnetometer */
   // setting the sample rate divider
	if (!mpu9250WriteRegister(MPU9250_SMPDIV, 19)) {
		return -1;
	}
	if (srd > 9) {
		// set AK8963 to Power Down
		if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		usleep(100000); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 8 Hz update rate
		if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS1) < 0) {
			return -3;
		}
		usleep(100000); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		mpu9250ReadAK8963Registers(MPU9250_AK8963_HXL, 7);
	} else {
		// set AK8963 to Power Down
		if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		usleep(100000); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS2) < 0) {
			return -3;
		}
		usleep(100000); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		mpu9250ReadAK8963Registers(MPU9250_AK8963_HXL, 7);
	}
	/* setting the sample rate divider */
	if (!mpu9250WriteRegister(MPU9250_SMPDIV, srd)) { // setting the sample rate divider
		return -4;
	}
	handler._srd = srd;
	return 1;
}

static char mpu9250SetDlpfBandwidth(MPU9250_DlpfBandwidth_t bandwidth)
{
	switch (bandwidth) {
		case MPU9250_DLPF_BANDWIDTH_184HZ: {
         // setting accel bandwidth to 184Hz
			if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_184)) { 
				return -1;
			}
         // setting gyro bandwidth to 184Hz
			if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_184)) { 
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_92HZ: {
         // setting accel bandwidth to 92Hz
			if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_92)) { 
				return -1;
			}
         // setting gyro bandwidth to 92Hz
			if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_92)) { 
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_41HZ: {
         // setting accel bandwidth to 41Hz
			if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_41)) { 
				return -1;
			}
         // setting gyro bandwidth to 41Hz
			if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_41)) { 
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_20HZ: {
         // setting accel bandwidth to 20Hz
			if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_20)) { 
				return -1;
			}
         // setting gyro bandwidth to 20Hz
			if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_20)) { 
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_10HZ: {
         // setting accel bandwidth to 10Hz
			if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_10)) { 
				return -1;
			}
         // setting gyro bandwidth to 10Hz
			if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_10)) { 
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_5HZ: {
         // setting accel bandwidth to 5Hz
			if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_5)) { 
				return -1;
			}
         // setting gyro bandwidth to 5Hz
			if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_5)) { 
				return -2;
			}
			break;
		}
	}
	handler._bandwidth = bandwidth;
	return 1;
}

static bool mpu9250SetGyroRange(MPU9250_GyroRange_t range)
{
	switch(range) {
		case MPU9250_GYRO_RANGE_250DPS: {
		  // setting the gyro range to 250DPS
		  if(!mpu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_250DPS)){
			return false;
		  }
        // setting the gyro scale to 250DPS
		  handler._gyroScale = 250.0f/32767.5f * MPU9250_D2R; 
		  break;
		}
		case MPU9250_GYRO_RANGE_500DPS: {
		  // setting the gyro range to 500DPS
		  if(!mpu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_500DPS)){
			return false;
		  }
        // setting the gyro scale to 500DPS
		  handler._gyroScale = 500.0f/32767.5f * MPU9250_D2R; 
		  break;
		}
		case MPU9250_GYRO_RANGE_1000DPS: {
		  // setting the gyro range to 1000DPS
		  if(!mpu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_1000DPS)){
			return false;
		  }
        // setting the gyro scale to 1000DPS
		  handler._gyroScale = 1000.0f/32767.5f * MPU9250_D2R; 
		  break;
		}
		case MPU9250_GYRO_RANGE_2000DPS: {
		  // setting the gyro range to 2000DPS
		  if(!mpu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_2000DPS)){
			return false;
		  }
        // setting the gyro scale to 2000DPS
		  handler._gyroScale = 2000.0f/32767.5f * MPU9250_D2R; 
		  break;
		}
	}
	handler._gyroRange = range;
	return true;
}

static char mpu9250CalibrateGyro(void)
{
	// set the range, bandwidth, and srd
	if (!mpu9250SetGyroRange(MPU9250_GYRO_RANGE_250DPS)) {
		return -1;
	}

	if (mpu9250SetDlpfBandwidth(MPU9250_DLPF_BANDWIDTH_20HZ) < 0) {
		return -2;
	}

	if (mpu9250SetSrd(19) < 0) {
		return -3;
	}

	// take samples and find bias
	handler._gxbD = 0;
	handler._gybD = 0;
	handler._gzbD = 0;

	for (uint8_t i=0; i < handler._numSamples; i++) {
		mpu9250Read();
		handler._gxbD += ((mpu9250GetGyroX_rads() + handler._gxb)/handler._numSamples);
		handler._gybD += ((mpu9250GetGyroY_rads() + handler._gyb)/handler._numSamples);
		handler._gzbD += ((mpu9250GetGyroZ_rads() + handler._gzb)/handler._numSamples);
		usleep(20000);
	}

	handler._gxb = (float)handler._gxbD;
	handler._gyb = (float)handler._gybD;
	handler._gzb = (float)handler._gzbD;

	// set the range, bandwidth, and srd back to what they were
	if (!mpu9250SetGyroRange(handler._gyroRange)) {
		return -4;
	}

	if (mpu9250SetDlpfBandwidth(handler._bandwidth) < 0) {
		return -5;
	}

	if (mpu9250SetSrd(handler._srd) < 0) {
		return -6;
	}
	return 1;
}

static char mpu9250WhoAmIAK8963(void)
{
	// read the WHO AM I register
	if (mpu9250ReadAK8963Registers(MPU9250_AK8963_WHO_AM_I,1) < 0) {
		return -1;
	}
	// return the register value
	return handler._buffer[0];
}

static char mpu9250WhoAmI(void)
{
	// read the WHO AM I register
	if (mpu9250ReadRegisters(MPU9250_WHO_AM_I,1) < 0) {
		return -1;
	}

	// return the register value
	return handler._buffer[0];
}


static char mpu9250ReadAK8963Registers(unsigned char subAddress, unsigned char data)
{
	// set slave 0 to the AK8963 and set for read
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ_FLAG) < 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// enable I2C and request the bytes
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | (uint8_t)1) < 0) {
		return -3;
	}
	usleep(1000); // takes some time for these registers to fill
	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	handler._status = mpu9250ReadRegisters(MPU9250_EXT_SENS_DATA_00, 1);
	return handler._status;
}


static char mpu9250WriteAK8963Register(unsigned char subAddress, unsigned char data)
{
	// set slave 0 to the AK8963 and set for write
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR) < 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// store the data for write
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_DO, data) < 0) {
		return -3;
	}
	// enable I2C and send 1 byte
	if (mpu9250WriteRegister(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | (uint8_t)1) < 0) {
		return -4;
	}
	// read the register and confirm
	if (mpu9250ReadAK8963Registers(subAddress,1) < 0) {
		return -5;
	}
	if(handler._buffer[0] == data) {
		return 1;
	} else{
		return -6;
	}
}


static char mpu9250InitializeControlStructure(void)
{
	handler._tempScale = 333.87f;
	handler._tempOffset = 21.0f;
	handler._numSamples = 100;
	handler._axs = 1.0f;
	handler._ays = 1.0f;
	handler._azs = 1.0f;
	handler._maxCounts = 1000;
	handler._deltaThresh = 0.3f;
	handler._coeff = 8;
	handler._hxs = 1.0f;
	handler._hys = 1.0f;
	handler._hzs = 1.0f;
	handler.tX[0] = 0;
	handler.tX[1] = 1;
	handler.tX[2] = 0;
	handler.tY[0] = 1;
	handler.tY[1] = 0;
	handler.tY[2] = 0;
	handler.tZ[0] = 0;
	handler.tZ[1] = 0;
	handler.tZ[2] = -1;
}
	
static char  mpu9250Init(void)
{
	mpu9250InitializeControlStructure();
	
	// select clock source to gyro
	if (!mpu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL)) {
		return -1;
	}

	// enable I2C master mode
    if (!mpu9250WriteRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN)) {
        	return -2;
	}

	// set the I2C bus speed to 400 kHz
	if (!mpu9250WriteRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK)) {
		return -2;
	}

	// set AK8963 to Power Down
	mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN);
	// reset the MPU9250
	mpu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_PWR_RESET);
	// wait for MPU-9250 to come back up
	usleep(1000);
	// reset the AK8963
	mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL2, MPU9250_AK8963_RESET);

	// select clock source to gyro
	if (!mpu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL)) {
		return -4;
	}

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	if ((mpu9250WhoAmI() != 113) && (mpu9250WhoAmI() != 115)) {
		return -5;
	}

	// enable accelerometer and gyro
	if (!mpu9250WriteRegister(MPU9250_PWR_MGMNT_2, MPU9250_SEN_ENABLE)) {
		return -6;
	}
	// setting accel range to 16G as default
	if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_16G)) {
		return -7;
	}

	handler._accelScale = MPU9250_G * 16.0f / 32767.5f; // setting the accel scale to 16G
	handler._accelRange = MPU9250_ACCEL_RANGE_16G;

	// setting the gyro range to 2000DPS as default
	if (!mpu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_2000DPS)) {
		return -8;
	}

   // setting the gyro scale to 2000DPS
	handler._gyroScale = 2000.0f / 32767.5f * MPU9250_D2R; 
	handler._gyroRange = MPU9250_GYRO_RANGE_2000DPS;

	// setting bandwidth to 184Hz as default
	if (!mpu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_184)) {
		return -9;
	}

   // setting gyro bandwidth to 184Hz
	if (!mpu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_184)) { 
		return -10;
	}

	handler._bandwidth = MPU9250_DLPF_BANDWIDTH_184HZ;

	// setting the sample rate divider to 0 as default
	if (!mpu9250WriteRegister(MPU9250_SMPDIV, 0x00)) {
		return -11;
	}

	handler._srd = 0;

	// enable I2C master mode
	if (!mpu9250WriteRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN)) {
		return -12;
	}

	// set the I2C bus speed to 400 kHz
	if (!mpu9250WriteRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK)) {
		return -13;
	}

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if (mpu9250WhoAmIAK8963() != 72) {
		return -14;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
		return -15;
	}

	usleep(100000);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	mpu9250ReadAK8963Registers(MPU9250_AK8963_ASA, 3);
	handler._magScaleX = ((((float) handler._buffer[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f
			/ 32760.0f; // micro Tesla
	handler._magScaleY = ((((float) handler._buffer[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f
			/ 32760.0f; // micro Tesla
	handler._magScaleZ = ((((float) handler._buffer[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f
			/ 32760.0f; // micro Tesla

	// set AK8963 to Power Down
	if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
		return -17;
	}

	usleep(100000); // long wait between AK8963 mode changes

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	if (mpu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS2) < 0) {
		return -18;
	}

	usleep(100000); // long wait between AK8963 mode changes

	// select clock source to gyro
	if (!mpu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL)) {
		return -19;
	}

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	mpu9250ReadAK8963Registers(MPU9250_AK8963_HXL, 7);
	
	// estimate gyro bias
	if (mpu9250CalibrateGyro() < 0) {
		return -20;
	}

	return true;
}

// Funciones para obtener los datos.

//Read sensor registers and store data at control structure
static bool mpu9250Read(void)
{
	// grab the data from the MPU9250
	if(!mpu9250ReadRegisters(MPU9250_ACCEL_OUT, 21)) {
		return false;
	}
	// combine into 16 bit values
	handler._axcounts = (((int16_t)handler._buffer[0]) << 8)  | handler._buffer[1];
	handler._aycounts = (((int16_t)handler._buffer[2]) << 8)  | handler._buffer[3];
	handler._azcounts = (((int16_t)handler._buffer[4]) << 8)  | handler._buffer[5];
	handler._tcounts  = (((int16_t)handler._buffer[6]) << 8)  | handler._buffer[7];
	handler._gxcounts = (((int16_t)handler._buffer[8]) << 8)  | handler._buffer[9];
	handler._gycounts = (((int16_t)handler._buffer[10]) << 8) | handler._buffer[11];
	handler._gzcounts = (((int16_t)handler._buffer[12]) << 8) | handler._buffer[13];
	handler._hxcounts = (((int16_t)handler._buffer[15]) << 8) | handler._buffer[14];
	handler._hycounts = (((int16_t)handler._buffer[17]) << 8) | handler._buffer[16];
	handler._hzcounts = (((int16_t)handler._buffer[19]) << 8) | handler._buffer[18];
	// transform and convert to float values
	handler._ax = (((float)(handler.tX[0]*handler._axcounts + handler.tX[1]*handler._aycounts + handler.tX[2]*handler._azcounts) * handler._accelScale) - handler._axb)*handler._axs;
	handler._ay = (((float)(handler.tY[0]*handler._axcounts + handler.tY[1]*handler._aycounts + handler.tY[2]*handler._azcounts) * handler._accelScale) - handler._ayb)*handler._ays;
	handler._az = (((float)(handler.tZ[0]*handler._axcounts + handler.tZ[1]*handler._aycounts + handler.tZ[2]*handler._azcounts) * handler._accelScale) - handler._azb)*handler._azs;
	handler._gx = ((float) (handler.tX[0]*handler._gxcounts + handler.tX[1]*handler._gycounts + handler.tX[2]*handler._gzcounts) * handler._gyroScale) -  handler._gxb;
	handler._gy = ((float) (handler.tY[0]*handler._gxcounts + handler.tY[1]*handler._gycounts + handler.tY[2]*handler._gzcounts) * handler._gyroScale) -  handler._gyb;
	handler._gz = ((float) (handler.tZ[0]*handler._gxcounts + handler.tZ[1]*handler._gycounts + handler.tZ[2]*handler._gzcounts) * handler._gyroScale) -  handler._gzb;
	handler._hx = (((float)(handler._hxcounts) * handler._magScaleX) - handler._hxb)*handler._hxs;
	handler._hy = (((float)(handler._hycounts) * handler._magScaleY) - handler._hyb)*handler._hys;
	handler._hz = (((float)(handler._hzcounts) * handler._magScaleZ) - handler._hzb)*handler._hzs;
	handler._t = ((((float) handler._tcounts)  - handler._tempOffset)/ handler._tempScale) + handler._tempOffset;
	
	return true;
}

// Returns the gyroscope measurement in the x direction, rad/s
static float mpu9250GetGyroX_rads(void)
{
	return handler._gx;
}

// Returns the gyroscope measurement in the y direction, rad/s
static float mpu9250GetGyroY_rads(void)
{
	return handler._gy;
}

// Returns the gyroscope measurement in the z direction, rad/s
static float mpu9250GetGyroZ_rads(void)
{
	return handler._gz;
}

int main(void)
{
	int status = 0, index = 0;
	mpu9250 = open("/dev/mse00", O_RDWR);

	status = mpu9250Init();
	usleep(10000);

	if (status < 0) {
		printf("Error on initialization of mpu9250 with error = %d", status);
	}
	else {
		printf("Success initialization\n");
	}

	while(index < 5){
		//Leer el sensor y guardar en estructura de control
		if (!mpu9250Read()) {
			printf("Fail reading value of MPU9250");
		}
		usleep(10000);
      		// Imprimir resultados
      		printf( "Giroscopo:      (%f, %f, %f)   [rad/s]\r\n", handler._gx, handler._gy, handler._gz);
		usleep(10000);
		printf( "Acelerometro:   (%f, %f, %f)   [m/s2]\r\n", handler._ax, handler._ay, handler._az);
		usleep(10000);
		printf( "Magnetometro:   (%f, %f, %f)   [uT]\r\n", handler._hx, handler._hy, handler._hz);
		usleep(10000);
		printf( "Temperatura:    %f   [C]\r\n\r\n", handler._t);
		sleep(1);
		index++;
   	}
	
	close(mpu9250);
}
