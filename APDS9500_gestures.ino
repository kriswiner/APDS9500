/* APDS9500_gestures.ino
 * by: Kris Winer, Tlera Corporation, Copyright 2017
 * date: May 29, 2017
 * license: Beerware - Use this code however you'd like with attribution. If you 
 * find it useful you can buy me a beer some time.
 *
 * Basic sketch to test gesture recognition with the APDS-9500 gesture sensor
 * using the STM32L4 family of Arduino-compatible development boards, SDA on
 * pin 20, SCL on pin 21, and 400 kHz (fast mode) clock speed.
 * 
 * 
 */
 
 #include <Wire.h>

 #define APDS9500_R_RegBankSet                  0xEF // 0x00 register bank 0, 0x01 register bank 1

 /* Bank 0*/
 #define APDS9500_PartID_L                      0x00 // Low  byte of Part ID
 #define APDS9500_PartID_H                      0x01 // High byte of Part ID
 #define APDS9500_VersionID                     0x02 // High byte of Part ID

 /* Cursor Mode Controls */
 #define APDS9500_R_CursorUseTop                0x32
 #define APDS9500_R_PositionFilterStartSizeTh_L 0x33
 #define APDS9500_R_PositionFilterStartSizeTh_H 0x34
 #define APDS9500_R_ProcessFilterStartSizeTh_L  0x35
 #define APDS9500_R_ProcessFilterStartSizeTh_H  0x36
 #define APDS9500_R_CursorClampLeft             0x37
 #define APDS9500_R_CursorClampRight            0x38
 #define APDS9500_R_CursorClampUp               0x39
 #define APDS9500_R_CursorClampDown             0x3A
 #define APDS9500_R_CursorClampCenterX_L        0x3B
 #define APDS9500_R_CursorClampCenterX_H        0x3C
 #define APDS9500_R_CursorClampCenterY_L        0x3D
 #define APDS9500_R_CursorClampCenterY_H        0x3E
 #define APDS9500_R_Cursor_ObjectSizeTh         0x8B
 #define APDS9500_R_PositionResolution          0x8C
  
 /* Interrupt Controls */
 #define APDS9500_R_MCU_Int_Flag                0x40
 #define APDS9500_R_Int1_En                     0x41
 #define APDS9500_R_Int2_En                     0x42
 #define APDS9500_Int_Flag_1                    0x43
 #define APDS9500_Int_Flag_2                    0x44

 /* AE/AG Controls */
 #define APDS9500_R_AELedOff_UB                 0x46
 #define APDS9500_R_AELedOff_LB                 0x47
 #define APDS9500_R_AE_Exposure_UB_L            0x48
 #define APDS9500_R_AE_Exposure_UB_H            0x49
 #define APDS9500_R_AE_Exposure_LB_L            0x4A
 #define APDS9500_R_AE_Exposure_LB_H            0x4B
 #define APDS9500_R_AE_Gain_UB                  0x4C
 #define APDS9500_R_AE_Gain_LB                  0x4D
 #define APDS9500_R_AE_Gain_Step                0x4E
 #define APDS9500_R_AE_Gain_Default             0x4F
 #define APDS9500_R_Exp_Sel                     0x50
 #define APDS9500_R_Manual                      0x51
 #define APDS9500_AG_Stage_GG                   0x54
 #define APDS9500_Reg_ExposureNum_L             0x55
 #define APDS9500_Reg_ExposureNum_H             0x56
 #define APDS9500_Reg_global                    0x57
 #define APDS9500_AE_LED_Off_YAvg               0x58
 #define APDS9500_AE_Dec_Inc                    0x59
 #define APDS9500_AE_Normal_Factor              0x5A

 /* GPIO Setting*/
 #define APDS9500_InputMode_GPIO_0_1            0x80
 #define APDS9500_InputMode_GPIO_2_3            0x81
 #define APDS9500_InputMode_INT                 0x82

 /* Gesture Mode Controls */
 #define APDS9500_R_LightThd                    0x83
 #define APDS9500_R_GestureStartTh_L            0x84
 #define APDS9500_R_GestureStartTh_H            0x85
 #define APDS9500_R_GestureEndTh_L              0x86
 #define APDS9500_R_GestureEndTh_H              0x87
 #define APDS9500_R_ObjectMinZ                  0x88
 #define APDS9500_R_ObjectMaxZ                  0x89
 #define APDS9500_R_ProcessResolution           0x8C
 #define APDS9500_R_TimeDelayNum                0x8D 
 #define APDS9500_R_Disable45Degree             0x8E 
 #define APDS9500_R_XtoYGain                    0x8F 
 #define APDS9500_R_NoMotionCountThd            0x90 
 #define APDS9500_R_NoObjectCountThd            0x91 
 #define APDS9500_R_NormalizedImageWidth        0x92 
 #define APDS9500_R_XDirectionThd               0x93 
 #define APDS9500_R_YDirectionThd               0x94 
 #define APDS9500_R_ZDirectionThd               0x95 
 #define APDS9500_R_ZDirectionXYThd             0x96 
 #define APDS9500_R_ZDirectionAngleThd          0x97 
 #define APDS9500_R_RotateAngleThd              0x98 
 #define APDS9500_R_RotateConti                 0x99 
 #define APDS9500_R_RotateXYThd                 0x9A 
 #define APDS9500_R_RotateZThd                  0x9B 
 #define APDS9500_R_Filter                      0x9C 
 #define APDS9500_R_DistThd                     0x9D 
 #define APDS9500_R_GestureDetEn                0x9F 
 #define APDS9500_R_FilterImage                 0xA5 
 #define APDS9500_R_DiffAngleThd                0xA9 
 #define APDS9500_ObjectCenterX_L               0xAC 
 #define APDS9500_ObjectCenterX_H               0xAD 
 #define APDS9500_ObjectCenterY_L               0xAE 
 #define APDS9500_ObjectCenterY_H               0xAF 
 #define APDS9500_ObjectAvgY                    0xB0 
 #define APDS9500_ObjectSize_L                  0xB1 
 #define APDS9500_ObjectSize_H                  0xB2 
 #define APDS9500_Gx                            0xB3 
 #define APDS9500_Gy                            0xB4
 #define APDS9500_Gy                            0xB5
 #define APDS9500_GestureResult                 0xB6
 #define APDS9500_WaveCount                     0xB7
 #define APDS9500_NoObjectCount                 0xB8
 #define APDS9500_NoMotionCount                 0xB9
 #define APDS9500_LightCount                    0xBA
 #define APDS9500_LightAcc_L                    0xBB
 #define APDS9500_LightAcc_H                    0xBC
 #define APDS9500_TimeAcc_L                     0xBD
 #define APDS9500_TimeAcc_H                     0xBE
 #define APDS9500_AngleAcc_L                    0xC7
 #define APDS9500_AngleAcc_H                    0xC8
 #define APDS9500_XGainValue                    0xCA
 #define APDS9500_YGainValue                    0xCB
 #define APDS9500_R_YtoZSum                     0xCC
 #define APDS9500_R_YtoZFactor                  0xCD
 #define APDS9500_R_FilterLength                0xCE
 #define APDS9500_R_WaveThd                     0xCF
 #define APDS9500_R_AbortCountThd               0xD0
 #define APDS9500_R_AbortLength                 0xD1
 #define APDS9500_R_WaveEnH                     0xD2
 #define APDS9500_PositionFilterCenterX_L       0xD3
 #define APDS9500_PositionFilterCenterXY_H      0xD4
 #define APDS9500_PositionFilterCenterY_L       0xD5
 #define APDS9500_PositionFilterAvgY_L          0xD6
 #define APDS9500_PositionFilterAvgY_H          0xD7
 #define APDS9500_PositionFilterSize_L          0xD8
 #define APDS9500_ProcessFilterAvgY_H           0xD9
 #define APDS9500_ProcessFilterCenterX_L        0xDA
 #define APDS9500_ProcessFilterCenterXY_H       0xDB
 #define APDS9500_ProcessFilterCenterY_L        0xDC
 #define APDS9500_ProcessFilterSize_L           0xDD
 #define APDS9500_ProcessFilterAvgY_L           0xDE
 #define APDS9500_AbortIntervalCount_L          0xDF

 /* Bank 1 */
 
 /* Image size settings */
 #define APDS9500_Cmd_HSize                     0x00
 #define APDS9500_Cmd_VSize                     0x01
 #define APDS9500_Cmd_HStart                    0x02
 #define APDS9500_Cmd_VStart                    0x03
 #define APDS9500_Cmd_HV                        0x04
 
 /* Lens Shading */ 
 #define APDS9500_R_LensShadingComp_EnH         0x25
 #define APDS9500_R_Offest_X                    0x26
 #define APDS9500_R_Offest_Y                    0x27
 #define APDS9500_R_LSC                         0x28
 #define APDS9500_R_LSFT                        0x29
   
 #define APDS9500_R_global                      0x42
 #define APDS9500_R_ggh                         0x44
 
/* Sleep Mode Parameters */
 #define APDS9500_R_IDLE_TIME_L                 0x65
 #define APDS9500_R_IDLE_TIME_H                 0x66
 #define APDS9500_R_IDLE_TIME_SLEEP_1_L         0x67
 #define APDS9500_R_IDLE_TIME_SLEEP_1_H         0x68
 #define APDS9500_R_IDLE_TIME_SLEEP_2_L         0x69
 #define APDS9500_R_IDLE_TIME_SLEEP_2_H         0x6A
 #define APDS9500_R_Object_TIME_1_L             0x6B
 #define APDS9500_R_Object_TIME_1_H             0x6C
 #define APDS9500_R_Object_TIME_2_L             0x6D
 #define APDS9500_R_Object_TIME_2_H             0x6E
 #define APDS9500_R_TG_INIT_TIME                0x6F
 #define APDS9500_R_TG_POWERON_WAKEUP_TIME      0x71
 #define APDS9500_R_TG_EnH                      0x72
 #define APDS9500_R_Auto_SLEEP_Mode             0x73
 #define APDS9500_R_Wake_Up_Sig_Sel             0x74

 /* Image Controls */
 #define APDS9500_R_SRAM_Read_EnH               0x77

 /* I2C Address */
 #define APDS9500_ADDRESS 0x73

 bool intFlag = false;
 uint8_t intFlag1 = 0, intFlag2 = 0, gestResult, getEnabled;

 uint8_t myLed = 26;
 uint8_t intPin = 39;
 
 void setup() 
 {
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

  // define io pins
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with led off, active HIGH
  pinMode(intPin, INPUT);
  
  /* Initialize Gesture Sensor */
  // Choose bank 0
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select bank 0

  // Define cursor limits
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampLeft, 0x07);    // min horiz value
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampRight,0x17);    // max horiz value
  writeByte(APDS9500_ADDRESS, APDS9500_R_CursorClampUp,0x06);       // min vert value
  writeByte(APDS9500_ADDRESS, APDS9500_R_Int2_En,0x01);             // enable interrupt on proximity
  // Auto exposure/Auto gain Controls
  writeByte(APDS9500_ADDRESS, APDS9500_R_AELedOff_UB, 0x2D);        // exposure time upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AELedOff_LB, 0x0F);        // exposure time lower bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_UB_L, 0x3C);   // low byte auto exposure upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_UB_H, 0x00);   // high byte auto exposure upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Exposure_LB_L, 0x1E);   // low byte auto exposure lower bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_AE_Gain_LB, 0x20);         // auto gain upper bound
  writeByte(APDS9500_ADDRESS, APDS9500_R_Manual, 0x10);             // enable auto exposure
  writeByte(APDS9500_ADDRESS, 0x5E, 0x10);                          // don't know
  writeByte(APDS9500_ADDRESS, 0x60, 0x27);                          // don't know
  // Set up Interrupt
  writeByte(APDS9500_ADDRESS, APDS9500_InputMode_GPIO_0_1, 0x42);   // set GPIO0 as OUTPUT, GPIO1 as INPUT
  writeByte(APDS9500_ADDRESS, APDS9500_InputMode_GPIO_2_3, 0x44);   // set GPIO2 as INPUT, GPIO3 as INPUT
  writeByte(APDS9500_ADDRESS, APDS9500_InputMode_INT, 0x04);        // set INT as INPUT
  // Detection thresholds
  writeByte(APDS9500_ADDRESS, APDS9500_R_Cursor_ObjectSizeTh, 0x01); // object size threshold for cursor mode
  writeByte(APDS9500_ADDRESS, APDS9500_R_NoMotionCountThd, 0x06);    // no motion counter threshold
  writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionThd, 0x0A);       // gesture detection z threshold
  writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionXYThd, 0x0C);     // gesture detection x and y thresholds
  writeByte(APDS9500_ADDRESS, APDS9500_R_ZDirectionAngleThd, 0x05);  // angle threshold for forward and backward detection
  writeByte(APDS9500_ADDRESS, APDS9500_R_RotateXYThd, 0x14);         // rotation detection threshold
  writeByte(APDS9500_ADDRESS, APDS9500_R_Filter, 0x3F);              // filter weight and frame position threshold
  writeByte(APDS9500_ADDRESS, APDS9500_R_FilterImage, 0x19);         // use pixel brightness for weak average filter
  writeByte(APDS9500_ADDRESS, APDS9500_R_YtoZSum, 0x19);             // z-direction mapping parameter
  writeByte(APDS9500_ADDRESS, APDS9500_R_YtoZFactor, 0x0B);          // z-direction mapping parameter
  writeByte(APDS9500_ADDRESS, APDS9500_R_FilterLength, 0x03);        // filter length for cursor object center
  writeByte(APDS9500_ADDRESS, APDS9500_R_WaveThd, 0x64);             // wave gesture counter and angle thresholds
  writeByte(APDS9500_ADDRESS, APDS9500_R_AbortCountThd, 0x21);       // abort gesture counter threshold

  // Change to Bank 1
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x01);          // select bank 1

  // Image size settings  
  writeByte(APDS9500_ADDRESS, APDS9500_Cmd_HStart, 0x0F);            // horizontal starting point
  writeByte(APDS9500_ADDRESS, APDS9500_Cmd_VStart, 0x10);            // vertical starting point
  writeByte(APDS9500_ADDRESS, APDS9500_Cmd_HV, 0x02);                // vertical flip
  writeByte(APDS9500_ADDRESS, APDS9500_R_LensShadingComp_EnH, 0x01); // enable lens shading compensation
  writeByte(APDS9500_ADDRESS, APDS9500_R_Offest_Y, 0x39);            // vertical offset of lens, set to 55
  writeByte(APDS9500_ADDRESS, APDS9500_R_LSC, 0x7F);                 // Lens shading coefficient, set to 127
  writeByte(APDS9500_ADDRESS, APDS9500_R_LSFT, 0x08);                // shift amount, initialize to 10
  writeByte(APDS9500_ADDRESS, 0x3E,0xFF);                            // don't know
  writeByte(APDS9500_ADDRESS, 0x5E,0x3D);                            // don't know
  /* Sleep mode parameters */
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_L, 0x96);         // idle time low byte = 150 which is set for ~120 fps
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_1_L, 0x97); // idle time for weak sleep, set for report rate ~ 120 Hz
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_2_L, 0xCD); // idle time for deep sleep, low byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_IDLE_TIME_SLEEP_2_H, 0x01); // idle time for deep sleep, high byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_Object_TIME_2_L, 0x2C);     // deep sleep enter time, low byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_Object_TIME_2_H, 0x01);     // deep sleep enter time, high byte
  writeByte(APDS9500_ADDRESS, APDS9500_R_TG_EnH, 0x01);              // enable time gating
  writeByte(APDS9500_ADDRESS, APDS9500_R_Auto_SLEEP_Mode, 0x35);     // no object weak and deep sleep, object wake
  writeByte(APDS9500_ADDRESS, APDS9500_R_Wake_Up_Sig_Sel, 0x00);     // interrupt on time gate start
  /* Start sensor */
  writeByte(APDS9500_ADDRESS, APDS9500_R_SRAM_Read_EnH, 0x01);       //SRAM read enable

  // Change back to bank 0 for data read
  writeByte(APDS9500_ADDRESS, APDS9500_R_RegBankSet, 0x00);         // select bank 0

  getEnabled = readByte(APDS9500_ADDRESS, APDS9500_R_GestureDetEn);
  if(getEnabled & 0x10) Serial.println("ROTATE gesture detection enabled");
  if(getEnabled & 0x20) Serial.println("BACKWARD and FORWARD gesture detection enabled");
  if(getEnabled & 0x40) Serial.println("UP and DOWN gesture detection enabled");
  if(getEnabled & 0x80) Serial.println("LEFT and RIGHT gesture detection enabled");

  getEnabled = readByte(APDS9500_ADDRESS, APDS9500_R_WaveEnH);
  if(getEnabled & 0x80) Serial.println("WAVE gesture detection enabled");
  //  disable wave gesture
  //  writeByte(APDS9500_ADDRESS, APDS9500_R_WaveEnH, getEnabled & ~(0x80) );
  //  getEnabled = readByte(APDS9500_ADDRESS, APDS9500_R_WaveEnH);
  //  if(getEnabled & 0x80) Serial.println("WAVE gesture detection enabled");
  
  attachInterrupt(intPin, myHandler, FALLING);

 }


void loop() 
{
  
 if(intFlag == true)
 {
  intFlag = false;
  intFlag1 = readByte(APDS9500_ADDRESS, APDS9500_Int_Flag_1);
  intFlag2 = readByte(APDS9500_ADDRESS, APDS9500_Int_Flag_2);
  gestResult = readByte(APDS9500_ADDRESS, APDS9500_GestureResult);

  if(gestResult & 0x20) {Serial.print("gesture result = "); Serial.println(gestResult & 0x0F);}

  if(intFlag1 & 0x01) Serial.println("UP event detected");
  if(intFlag1 & 0x02) Serial.println("DOWN event detected");
  if(intFlag1 & 0x04) Serial.println("LEFT event detected");
  if(intFlag1 & 0x08) Serial.println("RIGHT event detected");
  if(intFlag1 & 0x10) Serial.println("FORWARD event detected");
  if(intFlag1 & 0x20) Serial.println("BACKWARD event detected");
  if(intFlag1 & 0x40) Serial.println("CLOCKWISE event detected");
  if(intFlag1 & 0x80) Serial.println("COUNTERCLOCKWISE event detected");

  if(intFlag2 & 0x01) Serial.println("WAVE event detected");
  if(intFlag2 & 0x02) Serial.println("PROXIMITY event detected");
  if(intFlag2 & 0x04) Serial.println("HAS OBJECT event detected");
  if(intFlag2 & 0x08) Serial.println("WAKE UP TRIGGER event detected");
  if(intFlag2 & 0x80) Serial.println("NO OBJECT event detected");

  digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH); // flash led when interrupt received
}

}

/* end of main loop*/

  void myHandler()
  {
    intFlag = true;
  }

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
