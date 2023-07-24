#include <Arduino.h>
#include <i2c_device.h>

I2CMaster& master = Master;

I2CDevice motor_board = I2CDevice(master, 0x40);

// Registers to store data collected from motor_board
struct Registers {
    uint8_t flags = 0;      // Register 2. bit 0 => new data available
    int8_t temp = 0;        // Register 3. degrees C starting at temp_offset
    uint16_t reserved = 0;  // Register 4. Fill up to the next word boundary
    int32_t voltage = 0;    // Register 6. in 10ths of a volt
    int32_t current = 0;    // Register 10. in 10ths of an amp
};

// Registers that the caller can both read and write
struct Settings {
    int8_t temp_offset; // Register 0. Zero point for temperature. Default -40.
    int8_t scaling;     // Register 1. Writable. Sets scaling of voltage and current fields.
};
Settings settings = {0, 1};

volatile bool led_high = false;

void report_error(const char* message);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    master.begin(100 * 1000U);

    Serial.begin(9600);
    Serial.println("Started");

    delay(3000);
    motor_board.write(0, settings.temp_offset, true);
    motor_board.write(1, settings.scaling, true);

}

void loop() {
    Registers vals;

    if (motor_board.read(2, &vals.flags, true)) {
        Serial.println("New data available:");
        if (!motor_board.read(3, &vals.temp, true)) {
            report_error("ERROR: Failed to read temperature");
        }
        Serial.print("Temperature: ");
        Serial.println(vals.temp);

        if (!motor_board.read(6, &vals.voltage, true)) {
            report_error("ERROR: Failed to read voltage");
        }
        Serial.print("Voltage: ");
        Serial.println(vals.voltage);
        
        if (!motor_board.read(10, &vals.current, true)) {
            report_error("ERROR: Failed to read current");
        }
        Serial.print("Current: ");
        Serial.println(vals.current);
    }
    else {
            Serial.println("Not configured");
    }
    digitalWrite(LED_BUILTIN, led_high);
    delay(1000);
    led_high = !led_high;
}

void report_error(const char* message) {
    Serial.print(message);
    Serial.print(" Error: ");
    Serial.println((int)master.error());
}
