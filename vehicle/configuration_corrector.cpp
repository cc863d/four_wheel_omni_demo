#include "configuration_corrector.hpp"

ConfigurationCorrector::ConfigurationCorrector(std::vector<DigitalIn *> conditions, int  * fixed_point, int value)
: conditions_(conditions), fixed_point_(fixed_point), value_(value)
{
}

ConfigurationCorrector::~ConfigurationCorrector()
{
}
