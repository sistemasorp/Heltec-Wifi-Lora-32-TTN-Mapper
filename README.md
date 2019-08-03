# Heltec-Wifi-Lora-32-TTN-Mapper
## Create a TTN Mapper node with a Heltec Wifi Lora 32 V2

![TTN Mapper node](nodo_ttn_mapper.jpg?raw=true "TTN Mapper node")

The purpose of this project is to be able to use a Heltec WIFI Lora 32 V2 module together with a GPS module to map the coverage of TTN gateways. 

### Modules

The GPS's TX pin is connected to Heltec's pin 12.

You can power the entire system with a powerbank. The GPS module can be powered from the Heltec's 5v (or 3v3, depending on the module)  and GND pins, so you don't need the GND reference for the serial communication. Take care that the output voltage of the GPS TX pin is 3.3V.

### TTN console

For simplicity, the Activation Method used for the device is ABP. The Frame Counter Checks option must be deactivated in the Device Settings section so that it does not have to be reset every time the module is switched on. In the Decoder section of the Payload Formats of the application, the following code has to be applied:

```
function Decoder(bytes, port) {
  var decoded = {};
  decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
  decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
  decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
  decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
  var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
  var sign = bytes[6] & (1 << 7);
  if(sign)
  {
    decoded.alt = 0xFFFF0000 | altValue;
  }
  else
  {
    decoded.alt = altValue;
  }
  decoded.hdop = bytes[8] / 10.0;
  return decoded;
}
```
