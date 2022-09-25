#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"
#include <vector>

#ifndef M_PI
#define M_PI 3.141592f
#endif

class Encoder
{
public:
    /**
    *
    *@param a_pin - A相pin
    *@param b_pin - B相pin
    *@param enc_resolution - エンコーダの分解能
    *@param omni_diameter - オドメータの直径[mm]
    *@param enc_rotation - 1にすると正後転を入れ替える 何もなし(or 0)で軸方向から見て反時計回り
    *@param interrupt_period - 割り込み周期
    */
    Encoder(PinName a_pin, PinName b_pin, float enc_resolution, int omni_diameter, int interrupt_period);

    /**
    *pinのPullUP，タイマー割り込みの設定
    */
    void Setup();

    /**
    *正後転を入れ替える
    *
    *@return 1 - 軸方向から見て時計回り 0 - 反時計回り
    */
    bool Change_rotation();

    /**
    *エンコーダのカウントを表示する

    *@return パルス数(プラスマイナス)
    */
    int Pulse_count();

    /**
    *オドメータが進んだ距離を返す
    *
    *@return オドメータが進んだ距離[mm]
    */
    double Distance();

    /**
    *エンコーダの値を0にする
    *普通はtureになる
    *
    *@return 1 - 0にできた 0 - 0にできてない
    */
    bool Reset();

    /**
    * 
    * @return rad/s
    */
    double GetAngularVelocity();

    /**
    *タイマー割り込みで動かす関数
    */
    void Counter();
    void UpdateAngularVelocity(int variation);

    // private:
    DigitalIn _enc_a_pin;
    DigitalIn _enc_b_pin;
    float _frequency_ratio;            //分周比
    int _enc_resolution;               //エンコーダの分解能
    int _omni_diameter;                //オドメータの直径
    bool _enc_rotation;                //正後転判別フラグ
    int _count;                        //カウントする変数
    int _count_interrupt_period = 200; //  [us]
    int _encoder_increments_per_10000us = 0;
    std::vector<int8_t> _encoder_increments;
    int _encoder_increments_pointer = 0; // indicates the insertion lacation of the new value
    int _encoder_increments_size;

    uint8_t old_enc_ab;
};

#endif