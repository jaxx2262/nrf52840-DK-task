# nrf52840-DK-task
Task based on ble_app_blinky example

# Description of work 

I have used the ble_app_blinky example to do the task that was given. The software was written using the nrf5 SDK version 17.0.2 in the examples ble_app_blinky folder directly. 

## Short description of the task
The task was to write a BLE beacon with a unique name that advertises GPS coordinates. GPS coordinates are sent to the device with a command from a PC. 
The task was realized in Segger Embedded Studio using an example from the nrf5 SDK version 17.0.2 under ble_peripherals. The examples name is ble_app_blinky.  

## Requirements
* Bluetooth advertising beacon 
* Advertising data: GPS Coordinates
* Coordinates update from PC
* Distinctive device name

### Distinctive device name 
The name is compiled from two parts. A fixed prefix and a suffix. The prefix is defined in the code. The chosen prefix is "COM_". The suffix is a part of the bluetooth chips MAC address. The MAC address is a 48 bit unique address for every device. It consists of two main parts, the Organizationally Unique Identifier which is the upper 3 bytes and lower 3 bytes which is assigned by the vendor of the bluetooth chip. The last 3 bytes of the MAC address are chosen because these change based on the device by the vendor. This will give the possibility that every device will have a unique device name that should not repeat.
Example of the device unique name: `COM_33FFEE`

### Coordinates update from PC 
The coordinates must be updated from a PC. For this requirement I made a presumption that usually there are GPS modules that provide coordinate information. GPS modules mostly use UART for communication and thus I chose to add UART to the program to receive commands for the coordinates. Another presumption I made was that for the simplicity of the task it is written that the coordinates should be updated by a PC, but I would see that the bluetooth module in a final configuration would use a connected GPS module that sends out NMEA formatted data. From this presumption I used the GPS NMEA RMC packet information to acquire the GPS coordinates for the advertising data. Whenever a new RMC packet is received the module will dynamically update its advertising data to the new coordinates. 
Example command to update coordinates: `$GPRMC,105954.000,A,3150.6731,N,11711.9399,E,0.00,96.10,250313,,,A*53`

### Advertising data: GPS Coordinates
As the advertising data block is limited to 31 bytes, including the device name it took some time to fix the format of the data. For the task I chose to use the manufacturer data block for the GPS coordinates. To get the coordinate data size as small as possible I opted to convert the direct coordinates from the NMEA packet to a float. As the NMEA packet is human readbale ASCII data, then the full coordinates would not fit in the manufacturing advertising data because of the data would be too long. The coordinates in ascii format would take up about 10 + 11 bytes of data but when cast to float, the data size is only 5 + 5 bytes, (4 bytes latitude, N or S designator, 4 bytes longitude, E or W designator). This means that the data is not human readable anymore. But regarding that it is advertising data then there would be an app to read and decode the data. 
The float data bytes are in reverse order in the advertising packet (LSB first). 

