#pragma once

#include <math.h>

#define DELTA_TEMP 0
#define RESESTIVE_TEMPERATUR_SCHOULDER 10000

#define B_T_1_2 3984

#define GAIN_POWER_DISC 36000

//Внутренняя функция IAR так мы добавляем её в проект.
float log1pf(float x);

class NTC {

    //Параметры и переменные для вычисления температуры.
    float Om_float; //Переменная для вычисления сопротивления термистора.
    //float B_formula=3510;//Коэффициент для формулы.
    float R_formula; //Сопротивление в омах при 25 градусах.
    float T_formula; //25 градусов в Кельвин.
    float Temp_formula;

public:

    NTC()
    {
      R_formula = 10000;
      T_formula  =298.15;
    }

    uint16_t operator() (uint16_t adc)
    {
      //Вычисление температуры в К датчика
        Om_float = (float)4095 - adc;
        Om_float = adc / Om_float;
        Om_float = Om_float * RESESTIVE_TEMPERATUR_SCHOULDER;
        //Вычисляем по формуле T1 = 1 / ((ln(R1) – ln(R2)) / B + 1 / T2) температуру.
        Temp_formula = (1 / ( (log1pf(Om_float) - log1pf(R_formula)) / B_T_1_2 + 1 / T_formula));
        return (int16_t)Temp_formula - 273;
    }


};
