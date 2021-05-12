#define FULL_LENGTH 1398000
#define HOME_OFFSET   45000
#define BACKLASH      27000
#define RUNNING_CURRENT 800
#define IDLE_CURRENT    200

#define STALL_VALUE     200 // [0..255]

#define EN_PIN           38 // Enable
#define DIR_PIN          23 // Direction
#define STEP_PIN         22 // Step
#define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075
