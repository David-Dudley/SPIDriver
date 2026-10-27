// Initialize the GPIO bits
int initGPIO(const char *consumer);

// Close the access to the GPIO bits
void closeGPIO(void);

// Output a value to a GPIO Bit
int GPIOOutput(uint8_t index, uint8_t value);

// Select a particular chip selec combination
int chipSelect(uint8_t select);

