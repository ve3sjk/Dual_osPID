/*
 * CPPFile1.cpp
 *
 * Created: 11/4/2015 3:08:50 PM
 *  Author: sysop
 */

/*
*
*

typedef struct sensorData_t{
	byte stat;
	byte sensorId;
	byte sensortype;
	byte isWet;
	uint16_t temp;
	float volts;
	byte signal;
};

#define sizeof(sensorData_t);

typedef union I2C_Packet_t{
	sensorData_t sensor;
	byte I2CPacket[sizeof(sensorData_t)];
};

leakinfo.sensor.stat = 0;
leakinfo.sensor.sensorId = 22;
leakinfo.sensor.sensortype = 0;
leakinfo.sensor.isWet = 0;
leakinfo.sensor.temp = 75;
leakinfo.sensor.volts = 3141 / 1000.0;
leakinfo.sensor.signal = 88;


Wire.write(leakinfo.I2CPacket, PACKET_SIZE);

*
*
*
/

/************
*
*
*
 byte byteArray[PACKET_SIZE];
 int readstatus = I2c.read(addrSlaveI2C, PACKET_SIZE, byteArray ); //request data and store directly to i2CData array

 for (int k=0; k < PACKET_SIZE; k++)
 {  leakinfo.I2CPacket[k] = byteArray[k]; }
	 
*
*
*
/
	 




