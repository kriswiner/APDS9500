/* APDS9500_functiontest.ino
 * by: Kris Winer, Tlera Corporation, Copyright 2017
 * date: May 28, 2017
 * license: Beerware - Use this code however you'd like with attribution. If you 
 * find it useful you can buy me a beer some time.
 *
 * Basic sketch to test I2C communications for the APDS-9500 gesture sensor
 * using the STM32L4 family of Arduino-compatible development boards, SDA on
 * pin 20, SCL on pin 21, and 400 kHz (fast mode) clock speed.
 * 
 * 
 */
 
 #include <Wire.h>

/* Bank 0*/

#define APDS9500_PartID_L     0x00 // Low  byte of Part ID
#define APDS9500_PartID_H     0x01 // High byte of Part ID
#define APDS9500_VersionID    0x02 // High byte of Part ID

#define APDS9500_ADDRESS 0x73

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);
  Wire.begin(TWI_PINS_20_21); // set master mode 
  Wire.setClock(400000);      // I2C frequency at 400 kHz 
  delay(2000);
 
  I2Cscan();
 
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select bank 0, wake up APDS9500 I2C

  I2Cscan();

  Serial.println("I2C scan done");

  uint8_t partID_L  = readByte(APDS9500_ADDRESS, APDS9500_PartID_L);
  uint8_t partID_H  = readByte(APDS9500_ADDRESS, APDS9500_PartID_H);
  uint8_t versionID = readByte(APDS9500_ADDRESS, APDS9500_VersionID);
  Serial.println("APDS9500 Gesture Sensor");
  Serial.print("Part ID = 0x"); Serial.print(partID_L, HEX);  
  Serial.print(" and 0x"); Serial.println(partID_H, HEX);
  Serial.println("Part ID should be 0x20 and 0x76!");
  Serial.println("  ");
  Serial.print("Version ID = 0x0"); Serial.println(versionID, HEX); 

  if(partID_L == 0x20 && partID_H == 0x76)
  {
    Serial.println("APDS9500 ID confirmed!");
  }
  
  


}


void loop() {
  // put your main code here, to run repeatedly:

}

/* end of main loop*/

  // I2C scan function
  void I2Cscan()
  {
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
      error = Wire.transfer(address, NULL, 0, NULL, 0);
      
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  }

   // I2C read/write functions for the MPU9250 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
        {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) 
        {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) 
        {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
