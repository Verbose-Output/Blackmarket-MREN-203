
// // Need this library installed to read from the SGP30
// // For more details, look here: https://github.com/adafruit/Adafruit_SGP30
// // And https://adafruit.github.io/Adafruit_SGP30/html/index.html
// #include <Adafruit_SGP30.h>

// Adafruit_SGP30 sgp;

// // Return absolute humidity [mg/m^3] with approximation formula
// // @param temperature [°C]
// // @param humidity [%RH]
// uint32_t getAbsoluteHumidity(float temperature, float humidity);

// void setup()
// {
//   // Open the serial port at 115200 bps
//   Serial.begin(115200);

//   // Wait for serial connection before starting
//   while (!Serial)
//   {
//     delay(10);
//   }

//   Serial.println("__SGP30 demo__");

//   if (!sgp.begin())
//   {
//     Serial.println("Sensor not found :(");
//     while (1)
//     {
//       delay(10); // This will stay here forever if a sensor isn't found
//     }
//   }
// }
