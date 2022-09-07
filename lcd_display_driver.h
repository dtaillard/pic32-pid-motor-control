/* 
 * File:   lcd_display_driver.h
 * Author: Daniel Taillard
 * Date: September 10, 2021
 * 
 * Description: 
 */

#ifndef LCD_DISPLAY_DRIVER_H
#define	LCD_DISPLAY_DRIVER_H

void lcd_display_driver_enable();
void lcd_display_driver_initialize();
void lcd_display_driver_clear();
void lcd_display_driver_write(char *data, int length);
void display_driver_use_first_line();
void display_driver_use_second_line();
    

#endif	/* LCD_DISPLAY_DRIVER_H */