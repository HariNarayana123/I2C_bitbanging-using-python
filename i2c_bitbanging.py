import time
import RPi.GPIO as GPIO


# Define the GPIO pins for SDA and SCL
SDA_PIN = 26
SCL_PIN = 21

# Define the slave device address
dev_addr= 0x40

# Set up the GPIO pins in BCM mode(uses GPIO numbers)
GPIO.setmode(GPIO.BCM)

#set up the SDA_PIN and SCL_PIN as output pins
GPIO.setup(SDA_PIN, GPIO.OUT)
GPIO.setup(SCL_PIN, GPIO.OUT)


# Define the I2C start signal
def i2c_start():
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    GPIO.output(SDA_PIN, GPIO.LOW)
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.LOW)

# Define the I2C stop signal
def i2c_stop():
    GPIO.output(SDA_PIN, GPIO.LOW)
    GPIO.output(SCL_PIN, GPIO.LOW)
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    GPIO.output(SDA_PIN, GPIO.HIGH)
    

# Define the I2C write function
def i2c_write_byte(byte):
    for bit in range(8):
        if byte & (1 << (7 - bit)):
            GPIO.output(SDA_PIN, GPIO.HIGH)
        else:
            GPIO.output(SDA_PIN, GPIO.LOW)
        GPIO.output(SCL_PIN, GPIO.HIGH)
        time.sleep(0.00000005)
        GPIO.output(SCL_PIN, GPIO.LOW)
        
# Define the acknowledge
def ACK():	
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    while(SDA_PIN == GPIO.HIGH):
         continue
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.LOW)
    
    
    
# Define the I2C read function
def i2c_read_byte():
    byte = 0
    for bit in range(8):
        GPIO.output(SCL_PIN, GPIO.HIGH)
        time.sleep(0.00000003)
        if GPIO.input(SDA_PIN):
            byte |= 1 << (7 - bit)
        GPIO.output(SCL_PIN, GPIO.LOW)
    return byte

# Define non-acknowledgement function
def NACK():	
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.00000005)
    GPIO.output(SCL_PIN, GPIO.LOW)

 
def main():
  # Main program block
  GPIO.setwarnings(False)    
  while (True):       
      i2c_start() # send the start signal
      i2c_write_byte(dev_addr << 1) #  send the device address with 8th bit as zero for writing
      ACK() # sending the acknowledgement
      i2c_write_byte(0xFE) # send the command for software reset
      ACK() # sending the acknowledgement
      i2c_stop() # send the stop signal
      time.sleep(0.00020) # wait for 2 ms until software reset is completed 
       
      i2c_start() # send the start signal
      i2c_write_byte(dev_addr <<1) # send the device address with 8th bit as zero for writing
      ACK() # sending the acknowledgement
      i2c_write_byte(0xF3) # send the command for temeprature measurment
      ACK() # sending the acknowledgement
      time.sleep(0.086) # # wait, typ=66ms, max=85ms @ 14Bit resolution
      i2c_start() # send the start signal (repeated start)
      i2c_write_byte(dev_addr <<1 | 0x01) # send the device address with 8th bit as 1 for reading 
      ACK() # sending the acknowledgement
      temp_0= i2c_read_byte() # read the 1st byte of data (MSB)
      ACK() # sending the acknowledgement
      temp_1 =i2c_read_byte() # read the 2nd byte of data (MSB)
      ACK() # sending the acknowledgement
      temp_2 =i2c_read_byte() # read the 3rd byte of data (LSB)
      NACK() # sending the negative acknowledgement
      i2c_stop() # send the stop signal
        
      i2c_start() # send the start signal
      i2c_write_byte(dev_addr <<1) # send the device address with 8th bit as zero for writing
      ACK() # wait for the acknowledgement 
      i2c_write_byte(0xF5) # send the command for realtive humidity measurment
      ACK() #  sending the acknowledgement
      time.sleep(0.03) # wait, typ=22ms, max=29ms @ 12Bit resolution
      i2c_start() # send the start signal (repeated start)
      i2c_write_byte(dev_addr <<1 | 0x01) # send the device address with 8th bit as 1 for reading 
      ACK() # sending the acknowledgement
      hum_0= i2c_read_byte()
      ACK() # sending the acknowledgement
      hum_1 =i2c_read_byte()
      ACK() # sending the acknowledgement
      hum_2 =i2c_read_byte()
      NACK() # sending the negative acknowledgement
      i2c_stop() # sending the stop bit
       
      t = ((temp_0 << 8) +temp_1) & 0xFFFC  # set status bits to zero
      t = -46.85 + ((t * 175.72) / 65536)  # T = 46.82 + (175.72 * ST/2^16 )
      t=round(t, 1)
       
      rh = ((hum_0 << 8) + hum_1) & 0xFFFC  # zero the status bits
      rh = -6 + ((125 * rh) / 65536)
      if (rh > 100): rh = 100
      rh=round(rh, 1)
     
     print(f"Temperature : {t}{chr(248)}C")
     print("Relative humidity percentage: ",rh)
        
 
if __name__ == '__main__':
 
  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    GPIO.cleanup()
    
