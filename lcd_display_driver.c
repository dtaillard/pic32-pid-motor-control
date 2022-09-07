#include <xc.h>
#include "lcd_display_driver.h"

void lcd_display_driver_enable() { 
    LATDbits.LATD4 = 1;
    
    int i = 0;
    for(; i < 1000; ++i) {
        ;
    }
    
    LATDbits.LATD4 = 0;
}

void lcd_display_driver_initialize() {
    LATB = 0;
    TRISB = 0x7FFF; // RB15 = RS
    
    LATD = 0;
    TRISD = 0xFFCF; // RD4 = E
                    // RD5 = R/W
    
    LATE = 0;
    TRISE = 0xFF00; // RE[0-7] = DB[0-7]
    
    // Function Set Command //
    LATE = 0x38; // dual-line, 5x7 dots
    lcd_display_driver_enable();
    
    // Display ON/OFF Command //
    LATE = 0x0C; // display on, cursor off, blink off
    lcd_display_driver_enable();
    
    // Clear Display //
    LATE = 0x01;
    lcd_display_driver_enable();
    
    // Entry Mode Set Command //
    LATE = 0x06; // increment cursor, disable shift
    lcd_display_driver_enable();
}

void lcd_display_driver_clear() {
    LATBbits.LATB15 = 0; // Set RS = 0
    LATDbits.LATD5 = 0;  // Set R/W = 0
    
    // Clear Display //
    LATE = 0x01;
    lcd_display_driver_enable();
}

void lcd_display_driver_write(char *data, int length) {
    LATBbits.LATB15 = 1; // Set RS = 1
    LATDbits.LATD5 = 0;  // Set R/W = 0
    
    int i = 0;
    for(; i < length; ++i) {
        LATE = data[i];
        lcd_display_driver_enable();
    }
}

void display_driver_use_first_line() {
    LATBbits.LATB15 = 0; // Set RS = 0
    LATDbits.LATD5 = 0;  // Set R/W = 0
    
    // Set DDRAM Address to 0 //
    LATE = 0x80;
    lcd_display_driver_enable();
}

void display_driver_use_second_line() {
    LATBbits.LATB15 = 0; // Set RS = 0
    LATDbits.LATD5 = 0;  // Set R/W = 0
    
    // Set DDRAM Address to 40 //
    LATE = 0xC0;
    // 1000 0000 
    lcd_display_driver_enable();
}
