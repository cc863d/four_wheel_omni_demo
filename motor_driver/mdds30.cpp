#include "mdds30.hpp"
/* Serial Simplified */
MDDS30::MDDS30(UnbufferedSerial * md_serial, int num) : md_(md_serial), num_(num)
{   
    
}

void MDDS30::Rotate(int dir, double rate) {
    Rotate((0 < dir) ? -rate : rate);
}

void MDDS30::Rotate(double rate) {
    uint8_t c = 0;

    c += ((num_ % 2) << 7); // left or right
    c += ((rate >= 0) ? rotation_ << 6 : (!rotation_) << 6); // dir
    c += (0x3F & (uint8_t)(fabs(rate) * 63.0)); // speed
    if (md_->writeable()) md_->write(&c, 1);
}

void MDDS30::Brake() {
    uint8_t c = 0;
    c += ((num_ % 2) << 7); // left or right
    c += (0x01) << 6; // dir
    c &= (0xC0); // speed
    
    md_->write(&c, 1);
}
