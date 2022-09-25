#ifndef CONFIGURATION_CORRECTOR_HPP
#define CONFIGURATION_CORRECTOR_HPP

#include "mbed.h"
#include <vector>
// I would also like to include the address of the variable to be corrected in this class
class ConfigurationCorrector
{
private:
public:
    ConfigurationCorrector(std::vector<DigitalIn *> conditions, int * fixed_point, int value);
    ~ConfigurationCorrector();
    std::vector<DigitalIn * > conditions_;
    int * fixed_point_;
    int_least64_t value_; // correct this value
};


#endif