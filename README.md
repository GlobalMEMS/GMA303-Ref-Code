Quick Porting Guide
===================

## Codes to complete
Add the following codes for your platform and you are pretty much ready to go.

### sensor_driver_test.c: main program

* Add the time delay function
    ```
    #define DELAY_MS(ms)	//.....     /* Add your time delay function here */
    ```

* Add HW initialization
    ```
    /* Add your HW initialization code here
    ...
    ...
    ...
    ...
    */
    ```


### bus_support.c

* Add I2C read/write function pointer to your I2C functions
    ```
	 pbus->bus_read = I2C_read_bytes;    /* Put your I2C read function pointer here */
	 pbus->bus_write = I2C_write_bytes;  /* Put your I2C write function pointer here */
    ```

  Your I2C read/write functions should implement the following interface:
    ```
    //******************************************************************************
    //
    //! @brief Read multiple bytes from I2C slave with address devAddr
    //!
    //! @param devAddr 7-bit device slave address
    //! @param regAddr start register address
    //! @param dataBuf data byte array to store the reading
    //! @param len number of data to be read
    //
    //! @return number of bytes read
    //
    //******************************************************************************
    char I2C_read_bytes(unsigned char devAddr, 
    					unsigned char regAddr, 
    					unsigned char* dataBuf, 
    					unsigned char len)
     {
      /* ..... */
     }
    
    //******************************************************************************
    //
    //! @brief Write multiple bytes to I2C slave with address devAddr
    //!
    //! @param devAddr 7-bit device slave address
    //! @param regAddr register address
    //! @param dataBuf data bytes array buffer with datas to write
    //! @param len number of bytes to write
    //!
    //! @param return number of byte written
    //
    //******************************************************************************
    char I2C_write_bytes(unsigned char devAddr,
                         unsigned char regAddr,
    					 unsigned char* dataBuf,
    					 unsigned char len)
    
    {
     /* ..... */
    }
    ```

### gma302.c:gma303_initialization function
Default initialization actions:
 * Turn on offset temperature compensation
 * Set to continuous mode
 * Turn on low pass filter
 * Set data ready INT, ative high, push-pull
You may change the behavior of this initialization function to suit your purpose. Please refer to datasheet for more details on the register settings.

## Usage of AutoNil
 * The program will do an offset AutoNil when executed. Hold the g-sensor steady and maintai in level, then press 'y' after the program prompt for input.
 * You may change the `DATA_AVE_NUM` macro in the gSensor_autoNil.h for the moving averae order for the offset estimation. Defautl is 32.
   
