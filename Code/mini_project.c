#include <xc.h>

// Configuration Bits
#pragma config FOSC = HS        // High-Speed Oscillator
#pragma config WDTE = OFF       // Watchdog Timer Disabled
#pragma config PWRTE = ON       // Power-up Timer Enabled
#pragma config BOREN = ON       // Brown-out Reset Enabled
#pragma config LVP = OFF        // Low Voltage Programming Disabled
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection Disabled
#pragma config WRT = OFF        // Flash Program Memory Write Disable
#pragma config CP = OFF         // Code Protection Disabled

#define _XTAL_FREQ 20000000     // 20MHz Crystal Frequency

// LCD Pin Definitions
#define LCD_RS RC1
#define LCD_EN RC2
#define LCD_D4 RD4
#define LCD_D5 RD5
#define LCD_D6 RD6
#define LCD_D7 RD7

// Sensor and Control Pins
#define TEMP_PIN RA0    // LM35 analog input (AN0)
#define VOLT_PIN RA1    // Voltage sensor analog input (AN1)
#define CURR_SENS RA2   // ACS712 current sensor analog input (AN2)
#define START_BTN RB0   // Button to start monitoring (same as crash button)
#define BUZZER RB1      // Buzzer output pin

// Motor Driver Pins (L293D)
#define MOTOR_EN RC3    // Motor Enable
#define MOTOR_IN1 RC4   // Motor Input 1
#define MOTOR_IN2 RC5   // Motor Input 2

// Safety thresholds
#define TEMP_MAX 60.0   // Maximum safe temperature (°C)
#define TEMP_MIN 0.0    // Minimum safe temperature (°C)
#define VOLT_MAX 10.0   // Maximum safe voltage
#define VOLT_MIN 5.0    // Minimum safe voltage
#define MIN_CURRENT 0.01 // Minimum current indicating motor is working (A)
#define MAX_CURRENT 0.2

// Function Prototypes
void lcd_init();
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_clear();
void lcd_set_cursor(unsigned char row, unsigned char column);
void lcd_display(const char* text);
unsigned int read_adc(unsigned char channel);
void display_message(const char* line1, const char* line2);
unsigned char check_battery_status();
float read_voltage();
float read_temperature();
float read_current();
void run_motor(unsigned char direction);
void stop_motor();
void test_motor();
void activate_alarm();

// Perform comprehensive battery check after crash detected
unsigned char check_battery_status() {
    unsigned char i;
    float voltage_samples[20];  // For storing voltage readings
    float temp_samples[20];     // For storing temperature readings
    float avg_voltage = 0, avg_temp = 0;
    unsigned char battery_ok = 1;  // Assume battery is OK initially
    
    // Display monitoring message
    display_message("Crash Detected!", "Checking battery..");
    __delay_ms(1000);
    
    // Take readings for 10 seconds (20 readings, 500ms apart)
    for(i = 0; i < 20; i++) {
        // Take readings
        voltage_samples[i] = read_voltage();
        temp_samples[i] = read_temperature();
        
        // Display current values
        lcd_clear();
        lcd_set_cursor(1, 1);
        lcd_display("Checking: ");
        lcd_data((i*5/10) + '0');  // Show progress 0-9
        lcd_data('.');
        lcd_data((i*5%10) + '0');  // Decimal part
        lcd_display("s");
        
        lcd_set_cursor(2, 1);
        lcd_display("V:");
        
        // Display current voltage
        int volt_int = (int)voltage_samples[i];
        int volt_dec = (int)((voltage_samples[i] - volt_int) * 10);
        lcd_data(volt_int / 10 + '0');
        lcd_data(volt_int % 10 + '0');
        lcd_data('.');
        lcd_data(volt_dec + '0');
        lcd_display("V ");
        
        // Display current temperature
        lcd_display("T:");
        int temp_int = (int)temp_samples[i];
        int temp_dec = (int)((temp_samples[i] - temp_int) * 10);
        
        if(temp_int < 10) lcd_data(' ');
        else lcd_data(temp_int / 10 + '0');
        lcd_data(temp_int % 10 + '0');
        lcd_data('.');
        lcd_data(temp_dec + '0');
        lcd_display("C");
        
        // Add to averages
        avg_voltage += voltage_samples[i];
        avg_temp += temp_samples[i];
        
        __delay_ms(500);  // 0.5 second between readings
    }
    
    // Calculate averages
    avg_voltage /= 20.0;
    avg_temp /= 20.0;
    
    // Check if any values are outside safe range
    for(i = 0; i < 20; i++) {
        // Check temperature
        if(temp_samples[i] > TEMP_MAX || temp_samples[i] < TEMP_MIN) {
            battery_ok = 0;  // Battery not OK
            break;
        }
        
        // Check voltage
        if(voltage_samples[i] > VOLT_MAX || voltage_samples[i] < VOLT_MIN) {
            battery_ok = 0;  // Battery not OK
            break;
        }
    }
    
    // Display results
    if(battery_ok) {
        display_message("Battery OK", "System Safe");
        __delay_ms(3000);
    } else {
        // Battery issue detected - activate alarm
        activate_alarm();
    }
    
    return battery_ok;
}

// Test motor function
void test_motor() {
    float current;
    unsigned char motor_ok = 0;
    
    // Test Motor forward
    display_message("Testing Motor", "");
    run_motor(1); // Run motor forward
    __delay_ms(5000);  // Run for 2 seconds
    
    // Check current to verify motor operation
    current = read_current();
    
    // Display current reading
    lcd_clear();
    lcd_set_cursor(1, 1);
    lcd_display("Current: ");
    
    // Convert current to display format
    int curr_int = (int)current;
    int curr_dec = (int)((current - curr_int) * 100);
    
    lcd_data(curr_int + '0');
    lcd_data('.');
    lcd_data(curr_dec / 10 + '0');
    lcd_data(curr_dec % 10 + '0');
    lcd_display("A");
    
    // Check if motor is working based on current
    lcd_set_cursor(2, 1);
    if (current >= MIN_CURRENT && current <= MAX_CURRENT) {
        lcd_display("Motor OK");
        motor_ok = 1;
    } else {
        lcd_display("Motor Failure!");
        // Sound alarm for motor failure
        BUZZER = 1;
        __delay_ms(1000);
        BUZZER = 0;
        __delay_ms(500);
    }
    
    __delay_ms(2000);
    
    // Stop motor
    stop_motor();
    
    // Final motor status
    if (motor_ok) {
        display_message("All Tests Passed", "System Safe");
        __delay_ms(3000);
    } else {
        display_message("Motor Failure!", "Service Required");
        // Sound alarm for motor failure
        while(1) {
            BUZZER = 1;
            __delay_ms(300);
            BUZZER = 0;
            __delay_ms(200);
        }
    }
}

// Run motor in specified direction
void run_motor(unsigned char direction) {
    // Configure Motor
    MOTOR_EN = 1;  // Enable Motor
    if (direction) {
        MOTOR_IN1 = 1;
        MOTOR_IN2 = 0;
    } else {
        MOTOR_IN1 = 0;
        MOTOR_IN2 = 1;
    }
}

// Stop motor
void stop_motor() {
    // Stop Motor
    MOTOR_EN = 0;  // Disable Motor
    MOTOR_IN1 = 0;
    MOTOR_IN2 = 0;
}

// Read current from ACS712 sensor
float read_current() {
    unsigned int curr_adc = read_adc(2);  // Read from AN2 (RA2)
    
    // Convert ADC value to current
    // For ACS712 5A version: 185mV/A sensitivity
    // Zero current output is VCC/2 (2.5V for 5V VCC)
    
    float voltage = (curr_adc * 5.0) / 255.0;  // Convert to voltage
    float current = (voltage - 2.5) / 0.185;    // Convert to current in Amperes
    
    
    
    
    
    // Return absolute value (direction doesn't matter for our test)
    return (current < 0) ? -current : current;
}

// Activate alarm if battery is damaged
void activate_alarm() {
    unsigned char i;
    
    // Display warning
    display_message("BATTERY DAMAGED!", "DANGER!");
    
    // Beep alarm pattern until reset
    for(i = 0; i < 10; i++) {
        BUZZER = 1;  // Buzzer on
        __delay_ms(300);
        BUZZER = 0;  // Buzzer off
        __delay_ms(200);
    }
    
    // Continuous alarm after initial pattern
    while(1) {
        display_message("BATTERY DAMAGED!", "DANGER!");
        BUZZER = 1;
        __delay_ms(500);
        BUZZER = 0;
        __delay_ms(500);
    }
}

// Read and calculate voltage
float read_voltage() {
    unsigned int volt_adc = read_adc(1);  // Read from AN1
    return (volt_adc * 5.0 ) / 255.0;  // Using multiplier of 5.0 as determined
}

// Read and calculate temperature
float read_temperature() {
    unsigned int temp_adc = read_adc(0);  // Read from AN0
    return (temp_adc * 5.0 * 100.0) / 255.0;  // LM35: 10mV/°C
}

// Display a two-line message on the LCD
void display_message(const char* line1, const char* line2) {
    lcd_clear();
    lcd_set_cursor(1, 1);
    lcd_display(line1);
    lcd_set_cursor(2, 1);
    lcd_display(line2);
}

// Read ADC Value (0-1023)
unsigned int read_adc(unsigned char channel) {
    // Select channel (AN0, AN1, or AN2)
    if(channel == 0) {
        ADCON0 = 0x01;  // Channel 0 (AN0)
    } else if(channel == 1) {
        ADCON0 = 0x09;  // Channel 1 (AN1)
    } else {
        ADCON0 = 0x11;  // Channel 2 (AN2)
    }
    
    __delay_us(20);          // Acquisition time
    ADCON0bits.GO = 1;       // Start conversion
    while(ADCON0bits.GO);    // Wait for conversion to complete
    
    // Return 10-bit result
    return ((ADRESH << 8) + ADRESL);
}

// LCD Functions
void lcd_init() {
    // Set direction bits properly - IMPORTANT FIX
    TRISC1 = 0;  // RS pin as output
    TRISC2 = 0;  // EN pin as output
    TRISD4 = 0;  // D4 pin as output
    TRISD5 = 0;  // D5 pin as output
    TRISD6 = 0;  // D6 pin as output
    TRISD7 = 0;  // D7 pin as output
    
    LCD_RS = 0;
    LCD_EN = 0;
    
    __delay_ms(50);      // Extended wait for power-up (FIXED)
    
    // 4-bit initialization sequence
    LCD_RS = 0;          // Command mode
    
    // Send 0x03 three times (required for 4-bit interface)
    // Make sure to manipulate only the data bits, not the entire port
    LCD_D4 = 1;
    LCD_D5 = 1;
    LCD_D6 = 0;
    LCD_D7 = 0;
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0; // FIXED: longer pulse
    __delay_ms(5);
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_ms(5);
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_ms(5);
    
    // Set to 4-bit mode
    LCD_D4 = 0;
    LCD_D5 = 1;
    LCD_D6 = 0;
    LCD_D7 = 0;
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_ms(5);
    
    // Now in 4-bit mode, send commands with proper timing
    lcd_command(0x28); // 2-line, 5x8 font
    __delay_ms(5);     // FIXED: added delay
    lcd_command(0x0C); // Display ON, cursor OFF
    __delay_ms(5);     // FIXED: added delay
    lcd_command(0x06); // Auto-increment cursor
    __delay_ms(5);     // FIXED: added delay
    lcd_command(0x01); // Clear display
    __delay_ms(5);     // FIXED: Wait longer for clear to complete
    
    // FIXED: Added extra settings for display contrast
    lcd_command(0x08); // Display off
    __delay_ms(5);
    lcd_command(0x0C); // Display on, cursor off
    __delay_ms(5);
}

// FIXED: modified command function to handle bits individually
void lcd_command(unsigned char cmd) {
    LCD_RS = 0;  // Command mode
    
    // Send high nibble
    LCD_D7 = (cmd & 0x80) ? 1 : 0;
    LCD_D6 = (cmd & 0x40) ? 1 : 0;
    LCD_D5 = (cmd & 0x20) ? 1 : 0;
    LCD_D4 = (cmd & 0x10) ? 1 : 0;
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_us(100);  // FIXED: longer delay
    
    // Send low nibble
    LCD_D7 = (cmd & 0x08) ? 1 : 0;
    LCD_D6 = (cmd & 0x04) ? 1 : 0;
    LCD_D5 = (cmd & 0x02) ? 1 : 0;
    LCD_D4 = (cmd & 0x01) ? 1 : 0;
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_ms(2);  // Wait for command execution
}

// FIXED: modified data function to handle bits individually
void lcd_data(unsigned char data) {
    LCD_RS = 1;  // Data mode
    
    // Send high nibble
    LCD_D7 = (data & 0x80) ? 1 : 0;
    LCD_D6 = (data & 0x40) ? 1 : 0;
    LCD_D5 = (data & 0x20) ? 1 : 0;
    LCD_D4 = (data & 0x10) ? 1 : 0;
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_us(100);  // FIXED: longer delay
    
    // Send low nibble
    LCD_D7 = (data & 0x08) ? 1 : 0;
    LCD_D6 = (data & 0x04) ? 1 : 0;
    LCD_D5 = (data & 0x02) ? 1 : 0;
    LCD_D4 = (data & 0x01) ? 1 : 0;
    
    LCD_EN = 1; __delay_us(5); LCD_EN = 0;
    __delay_ms(1);  // FIXED: longer delay for data
}

void lcd_clear() {
    lcd_command(0x01);  // Clear display command
    __delay_ms(5);      // FIXED: Wait longer for clear to complete
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
    unsigned char address;
    
    // Calculate DDRAM address based on row and column
    if (row == 1) {
        address = 0x80 + (col - 1);  // First row starts at 0x80
    } else {
        address = 0xC0 + (col - 1);  // Second row starts at 0xC0
    }
    
    lcd_command(address);  // Set DDRAM address
    __delay_ms(1);         // FIXED: Added delay
}

void lcd_display(const char* text) {
    while (*text) {
        lcd_data(*text++);
        __delay_us(50);    // FIXED: Added small delay between characters
    }
}

void main() {
    // Port Configuration - FIXED: Separate initialization of ports
    TRISA = 0x07;     // RA0, RA1, RA2 as analog inputs
    TRISB = 0x01;     // RB0 as input (button), RB1 as output (buzzer)
    TRISC = 0x00;     // PORTC as output (LCD control and motor driver)
    TRISD = 0x00;     // PORTD as output (LCD data)

    // Clear all ports
    PORTA = 0x00;
    PORTB = 0x00;     // Buzzer OFF initially
    PORTC = 0x00;     // Initialize PORTC
    PORTD = 0x00;     // Initialize PORTD
    
    // ADC Config
    ADCON0 = 0x01;    // ADC ON, channel 0 selected
    ADCON1 = 0x8E;    // Configure analog inputs
    
    // FIXED: Longer delays and proper initialization sequence
    __delay_ms(500);  // Long delay for system stabilization
    
    // Initialize LCD with proper delay
    lcd_init();
    __delay_ms(100);  // Small delay after initialization
    
    // Test LCD functionality by sending characters directly
    lcd_clear();
    __delay_ms(10);
    
    // FIXED: Show startup message one character at a time
    lcd_set_cursor(1, 1);
    lcd_display("EV safety system");
    __delay_ms(50);
    
    lcd_set_cursor(2, 1);
    lcd_display("Press button now");
    __delay_ms(50);

    // Wait for button press
    while (1) {
        if (START_BTN == 1) {
            __delay_ms(50);     // Debounce
            if (START_BTN == 1) {
                while (START_BTN == 1); // Wait for release
                __delay_ms(50);  // Debounce
                break;
            }
        }
    }

    // Continue to next step
    display_message("Crash Detected!", "Starting check");
    __delay_ms(2000);
    
    // Check battery status
    unsigned char battery_ok = check_battery_status();
    
    // If battery is OK, proceed to test motor
    if (battery_ok) {
        display_message("Battery OK", "Testing motor...");
        __delay_ms(2000);
        test_motor();
    }

    while (1); // Halt here
}