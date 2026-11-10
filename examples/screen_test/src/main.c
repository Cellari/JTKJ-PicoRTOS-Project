#include <pico/stdlib.h>
#include <tkjhat/sdk.h>
 
int main(void) {
    init_hat_sdk();      // Initialize I2C (SDA=@ref DEFAULT_I2C_SDA_PIN, SCL=@ref DEFAULT_I2C_SCL_PIN)
    init_display();          // Initialize SSD1306 (@ref SSD1306_I2C_ADDRESS — 0x3C)
 
    clear_display();         // Start clean
    // ~ 21 chars width
    // ~ 7 lines height (if each line is 9 px)
    write_text_xy(0, 0,  "Line 0: Top");
    write_text_xy(0, 9,  "Line 1");
    write_text_xy(0, 18, "Line 2");
    write_text_xy(0, 27, "Line 3");
    write_text_xy(0, 36,  "Line 4");
    write_text_xy(0, 45,  "Line 5");
    write_text_xy(0, 54, "Line 6");
    write_text_xy(0, 63, "Line 7");
 
    while (true) { tight_loop_contents(); }
}