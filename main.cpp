#include <Arduino.h>
#include <RCSwitch.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>

volatile unsigned long tempFunk = 0;
volatile unsigned int count = 99;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define ONE_WIRE_VCC 5

#define RF_DATA 9
#define RF_VCC 8

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;


ISR(WDT_vect)
  /* Watchdog Timer Interrupt Service Routine */
  {
    count++;
  }

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
//    Serial.println("Error: Could not read temperature data");
    return;
  }
//< Serial.print("Temp C: ");
//  Serial.print(tempC);
  tempFunk = tempC * 100;
//  Serial.print(" Temp F: ");
//  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


RCSwitch mySwitch = RCSwitch();

void setup() {

  Serial.begin(9600);
  
  ADCSRA = ADCSRA & B01111111; // ADC abschalten, ADEN bit7 zu 0
  ACSR = B10000000; // Analogen Comparator abschalten, ACD bit7 zu 1
  DIDR0 = DIDR0 | B00111111; // Digitale Eingangspuffer ausschalten, analoge Eingangs Pins 0-5 auf 1

  /* Setup des Watchdog Timers */
  MCUSR &= ~(1<<WDRF);             /* WDT reset flag loeschen */
  WDTCSR |= (1<<WDCE) | (1<<WDE);  /* WDCE setzen, Zugriff auf Presclaler etc. */
  WDTCSR = 1<<WDP0 | 1<<WDP3;      /* Prescaler auf 8.0 s */
  WDTCSR |= 1<<WDIE;               /* WDT Interrupt freigeben */
  
  // Transmitter is connected to Arduino Pin #10  
  mySwitch.enableTransmit(RF_DATA);
  
  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set pulse length.
  mySwitch.setPulseLength(320);
  
  // Optional set number of transmission repetitions.
  mySwitch.setRepeatTransmit(3);

  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(ONE_WIRE_VCC, OUTPUT);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(RF_VCC, OUTPUT); 
  pinMode(RF_DATA, OUTPUT); 
  
  digitalWrite(RF_VCC, LOW);
  digitalWrite(ONE_WIRE_VCC, HIGH);

  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
 
//  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
//  Serial.print("Locating devices...");
  sensors.begin();
//  Serial.print("Found ");
//  Serial.print(sensors.getDeviceCount(), DEC);
//  Serial.println(" devices.");

  // report parasite power requirements
//  Serial.print("Parasite power is: "); 
//  if (sensors.isParasitePowerMode()) Serial.println("ON");
//  else Serial.println("OFF");
  
  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // Note that you will need to use your specific address here
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  //if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  
  sensors.getAddress(insideThermometer, 0);

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them. It might be a good idea to 
  // check the CRC to make sure you didn't get garbage. The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
//  Serial.print("Device 0 Address: ");
//  printAddress(insideThermometer);
//  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
//  Serial.print("Device 0 Resolution: ");
//  Serial.print(sensors.getResolution(insideThermometer), DEC); 
//  Serial.println();
}

void loop() {
  if (count > 40) {
  //  digitalWrite(ONE_WIRE_VCC, HIGH); // switch thermometer on
  //  delay(4000);
    sensors.requestTemperatures(); // Send the command to get temperatures

    delay(800);
  
  // It responds almost immediately. Let's print out the data
    printTemperature(insideThermometer); // Use a simple function to print out the data

  //  digitalWrite(ONE_WIRE_VCC, LOW);    // switch thermometer off
    digitalWrite(RF_VCC, HIGH);    // switch RC on

    delay(30);

    mySwitch.send((unsigned long)805306368 + tempFunk,32); // 1879048192 legt das Device fest
    count = 0;

    digitalWrite(RF_VCC, LOW);    // switch RC off

    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();  
  power_usart0_disable();
  cli(); // deactivate interrupts
  sleep_enable(); // sets the SE (sleep enable) bit
  sei(); // 
  sleep_cpu(); // sleep now!!
  power_all_enable();
  sleep_disable();
}
