#ifndef OledDisplay_h
#define OledDisplay_h

#define I2C_ADDRESS 0x3C

#include <SSD1306AsciiAvrI2c.h>

class OledDisplay : public SSD1306AsciiAvrI2c {
  private:
    bool enabled;

  public:
    OledDisplay();
    void setDisabled (void);
    void setEnabled (void);
    bool isEnabled (void);
};

#endif
