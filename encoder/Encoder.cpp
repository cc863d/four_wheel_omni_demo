#include "Encoder.h"

Encoder::Encoder(PinName a_pin, PinName b_pin, float enc_resolution,
                 int omni_diameter, int interrupt_period)
    : _enc_a_pin(a_pin), _enc_b_pin(b_pin),
      _frequency_ratio(1 / (enc_resolution * 4)),
      _enc_resolution(enc_resolution), _omni_diameter(omni_diameter),
      old_enc_ab(0), _count_interrupt_period(interrupt_period)
{
    Setup();
    _encoder_increments.resize(10000 /*10000us = 10ms*/ /
                               _count_interrupt_period);
    _encoder_increments_size = (int)_encoder_increments.size();
    _count = 0;
}

void Encoder::Setup()
{
    _enc_a_pin.mode(PullUp);
    _enc_b_pin.mode(PullUp);
}

bool Encoder::Change_rotation()
{
    if (_enc_rotation == 1)
    {
        _enc_rotation = 0;
        return 0;
    }
    else
    {
        _enc_rotation = 1;
        return 1;
    }
}

int Encoder::Pulse_count()
{
    int count = _count;

    return count;
}

double Encoder::Distance()
{
    double distance = _count * _omni_diameter * M_PI * _frequency_ratio;

    return (distance);
}

bool Encoder::Reset()
{
    _count = 0;

    return 1;
}

void Encoder::Counter()
{
    const int enc[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

    uint8_t new_enc_ab =
        (_enc_a_pin.read() << 1) | _enc_b_pin.read(); // a,b相の値を保持

    uint8_t enc_ab = (old_enc_ab << 2) | new_enc_ab; //過去の値と合わせる

    if (_enc_rotation == 0)
        _count += enc[enc_ab]; //配列に入れて計算する
    else
        _count -= enc[enc_ab];

    old_enc_ab = new_enc_ab; //現在の値を過去に
    UpdateAngularVelocity((_enc_rotation == 0) ? enc[enc_ab] : -1 * enc[enc_ab]);
}

double Encoder::GetAngularVelocity()
{
    return ((double)_encoder_increments_per_10000us * 100.0 /*10ms * 100 = 1s*/ /
            (double)_enc_resolution / 4.0 * (2.0 * M_PI));
}

void Encoder::UpdateAngularVelocity(int variation)
{
    // add data new data
    _encoder_increments.at(_encoder_increments_pointer) = variation;
    _encoder_increments_per_10000us += variation;

    // deletes data from 10msec before
    if (_encoder_increments_pointer + 1 == _encoder_increments_size)
    {
        _encoder_increments_per_10000us -=
            _encoder_increments.at(0);
    }
    else
    {
        _encoder_increments_per_10000us -=
            _encoder_increments.at(_encoder_increments_pointer + 1);
    }

    _encoder_increments_pointer++;
    if (_encoder_increments_pointer >= _encoder_increments_size)
    {
        _encoder_increments_pointer = 0;
    }
}