#include "OledDisplay.h"

OledDisplay::OledDisplay(void) {
  enabled = false;
}

void OledDisplay::setEnabled (void)
{
  this->ssd1306WriteCmd(SSD1306_DISPLAYON);
  enabled = true;
}

void OledDisplay::setDisabled (void)
{
  this->ssd1306WriteCmd(SSD1306_DISPLAYOFF);
  enabled = false;
}

bool OledDisplay::isEnabled (void) {
  return enabled;
}
