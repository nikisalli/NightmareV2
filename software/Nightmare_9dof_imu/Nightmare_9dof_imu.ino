#include <GY953.h>
#include <SPI.h>

GY953 mag = GY953(3, 2);

void setup() {
	Serial.begin(115200);
	long unsigned debug_start = millis ();
	while (!Serial && ((millis () - debug_start) <= 5000));
	mag.begin();
	mag.setRefreshRate(200);
	Serial.println("started");
	while(!mag.update()){ }
	int data[3];
	byte data2[4];
	byte data3[3];
	memset(data,0,3);
	memset(data2,0,4);
	memset(data3,0,3);
}

void loop() {
	if (mag.update()) {
		int data[3];
		mag.getRPY(data);
		Serial.write(0xBB);
		Serial.write(0xBB);
		Serial.write((byte)(map(data[0],-18000,18000,0,255))); //roll
		Serial.write((byte)(map(data[1],-18000,18000,0,255))); //pitch
		Serial.write((byte)(map(data[2],-18000,18000,0,255))); //yaw
	}
}
