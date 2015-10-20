/* ========================================
 PROG: MPU9250 C library
 Based on InvenSense MPU-9250 register map 
 date: 20 sept 2015
 by: Alexis MASLYCZYK alex93nlg@gmail.com
 place: Montreal, Canada.

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
 * ========================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cytypes.h>
#include <project.h>
#include"mpu9250Alexis.h"

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};
uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR  
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
int16_t magCount[3]; 
float orientation[1];
float magn_x, magn_y;
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias

void I2CReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *value) {
	uint8_t i=0;
	I2C_MasterSendStart(devAddr, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(regAddr);
	I2C_MasterSendRestart(devAddr, I2C_READ_XFER_MODE);
	while (i++ < (length-1)) {
		*value++ = I2C_MasterReadByte(I2C_ACK_DATA);
	}
	*value = I2C_MasterReadByte(I2C_NAK_DATA);
	I2C_MasterSendStop();	
}

void I2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t *value) {
	I2CReadBytes(devAddr, regAddr, 1, value);
}

void I2CReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *value) {
   	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    I2CReadByte(devAddr, regAddr, value);
    *value &= mask;
    *value >>= (bitStart - length + 1);
}

void I2CReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *value) {
	I2CReadByte(devAddr, regAddr, value);
	*value = *value & (1 << bitNum);
}
	
void I2CWriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *value) {
	uint8_t i=0;
	I2C_MasterSendStart(devAddr, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(regAddr);
	while (i++ < length) {
		I2C_MasterWriteByte(*value++);
	}
	I2C_MasterSendStop();	
}

void I2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
	I2CWriteBytes(devAddr, regAddr, 1, &value);
}

void I2CWriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t value) {
	uint8_t b;
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	I2CReadByte(devAddr, regAddr, &b);
	value <<= (bitStart - length + 1);
	value &= mask;
	b &= ~(mask);
	b |= value;
	I2CWriteByte(devAddr, regAddr, b);	
}

void I2CWriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t value) {
	uint8_t b;
	I2CReadByte(devAddr, regAddr, &b);
	b = (value != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	I2CWriteByte(devAddr, regAddr, b);
}

void I2CWriteWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *value) {
	uint8_t i=0;
	I2C_MasterSendStart(devAddr, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(regAddr);
	while (i++ < length) {
		I2C_MasterWriteByte(((uint8_t)*value) >> 8);
		I2C_MasterWriteByte((uint8_t)*value++);
	}
	I2C_MasterSendStop();		
}

void I2CWriteWord(uint8_t devAddr, uint8_t regAddr, uint16_t value) {
	I2CWriteWords(devAddr, regAddr, 1, &value);
}


void initMPU9250(void){
    // Initialize MPU9250 device
        // wake up device
        I2CWriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
        CyDelay(10); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

        // get stable time source
        I2CWriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

        // Configure Gyro and Accelerometer
        // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
        // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
        // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
        I2CWriteByte(MPU9250_ADDRESS, CONFIG, 0x03);  
 
         // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        I2CWriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
        // Set gyroscope full scale range
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c ;//=  readByte(MPU9250_ADDRESS, GYRO_CONFIG);
        I2CReadByte(MPU9250_ADDRESS, GYRO_CONFIG,&c);
        I2CWriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
        I2CWriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        I2CWriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
        // Set accelerometer configuration
        //c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
        I2CReadByte(MPU9250_ADDRESS, ACCEL_CONFIG,&c);
        I2CWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
        I2CWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        I2CWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        //c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
        I2CReadByte(MPU9250_ADDRESS, ACCEL_CONFIG2,&c);        
        I2CWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
        I2CWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
        // can join the I2C bus and all can be controlled by the Arduino as master
        I2CWriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
        I2CWriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

void readAccelData(int16_t * destination){
        
        uint8_t rawData[6];  // x/y/z accel register data stored here
        I2CReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
        destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

void readGyroData(int16_t * destination){
        uint8_t rawData[6];  // x/y/z gyro register data stored here    
        I2CReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
        destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
        destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    }

void ReadMagData(int16_t * destination){
        uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
            uint8_t ST1; 
do
      {
        I2CReadBytes(MAG_ADDRESS,0x02,1,&ST1);
      }
      while (!(ST1&0x01));
        I2CReadBytes(MAG_ADDRESS, 0x03, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
            uint8_t c = rawData[6]; // End data read by reading ST2 register
            if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
                destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
                destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
                destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
                 /**mx=-(rawData[3]<<8 | rawData[2]);
                 *my=-(rawData[1]<<8 | rawData[0]);
                 *mz=-(rawData[5]<<8 | rawData[4]);*/
            }
}
void MPU6050_init(void){
   // Configure gyroscope range
  I2CWriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CWriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
}
/////////////
bool MPU9250WIA(){
    unsigned char data[1];
    I2CReadBytes(MPU9250_ADDRESS, 0x75,1, data ); //WHO I AM ACCEL
    if (data[0] == 0x71)
        {
            UART_1_PutString("I AM 0x71\r MLPU9250 is online...\n\r");
            return 1;            
        }
        else 
        {
            UART_1_PutString(" Error ID MPU9250 I SHOULD BE 0x71\n\r");
            return 0;
        }
    
}

bool CompassWIA(){
    unsigned char data[1]={0x00},destination[3]={8,9,85};  
I2CReadBytes(MAG_ADDRESS, 0x00,1, data ); //WHO I AM COMPASS
CyDelay(10);
if (data[0] == 0x48)
        {
//            UART_1_PutString("I AM 0x48\r Compass is online...\n\r");            
            return 1;         
        }
        else 
        {
            //UART_1_PutString(" Error ID COMPASS I SHOULD BE 0x48\n\r");
            return 0;
        }
}


    
void initCompass(float * destination){
        I2CWriteByte(MPU9250_ADDRESS,0x37,0x02);// Set by pass mode for the magnetometers
        I2CWriteByte(MAG_ADDRESS,0x0A,0x02);// Request first magnetometer single measurement
// First extract the factory calibration for each magnetometer axis
        uint8_t rawData[3];  // x/y/z gyro calibration data stored here
        I2CWriteByte(MAG_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
        CyDelay(10);
        I2CWriteByte(MAG_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
        CyDelay(10);
        I2CReadBytes(MAG_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
        destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
        destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
        destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
        I2CWriteByte(MAG_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
        CyDelay(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        I2CWriteByte(MAG_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
        CyDelay(10);
    }

void getCompassOrientation(float * orient){ // Obtains the orientation of the device in degrees. 0 degrees North. 180 degrees South.
        /*
        Remember that it is the earth's rotational axis that defines the geographic north and south poles that we use for map references.
        It turns out that there is a discrepancy of about 11.5 degrees between the geographic poles and the magnetic poles. The last is 
        what the magnetometer will read. A value, called the declination angle, can be applied to the magnetic direction to correct for this.
        On Valencia (Spain) this value is about 0 degrees.
        */
                
        // First of all measure 3 axis magnetometer values (only X and Y axis is used):        
        ReadMagData(magCount);  // Read the x/y/z adc values   
                                // Calculate the magnetometer values in milliGauss
                                // Include factory calibration per data sheet and user environmental corrections
        
            magn_x = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
            magn_y = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
        
        
        // Now obtains the orientation value:
        if (magn_y>0)
            orient[0] = 90.0 - (float) ( atan(magn_x/magn_y)*180/M_PI );
        else if (magn_y<0)
            orient[0] = 270.0 - (float) ( atan(magn_x/magn_y)*180/M_PI );
        else if (magn_y == 0){
            if (magn_x<0)
                orient[0] = 180.0;
            else
                orient[0] = 0.0;
        }
    }
/* [] END OF FILE */
