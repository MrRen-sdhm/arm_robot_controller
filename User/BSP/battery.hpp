#pragma once

#include "stm32f4xx_hal_adc.h"

#include "utils.hpp"
#include "dmabuffer_uart.hpp"

namespace hustac {
    
class Battery {
public:
    Battery(const char* name, ADC_HandleTypeDef* adc1, ADC_HandleTypeDef* adc2) 
        : name_(name)
        , adc1_(adc1)
        , adc2_(adc2) {
    }
        
    int init() {
        if (HAL_ADC_Start(adc1_) != HAL_OK || HAL_ADC_Start(adc2_) != HAL_OK) {
            printf("%s: ADC start failed\n", name_);
            return -1;
        } else {
            return 0;
        }
    }
    
    int spin_once() {
        if (HAL_ADC_PollForConversion(adc1_, 0) == HAL_OK) {
            adc1_value_ = HAL_ADC_GetValue(adc1_) / 4096.0f * 3.3f;
//            printf("%s: adc1_value_=%.2f V\n", name_, adc1_value_);
        }
        if (HAL_ADC_PollForConversion(adc2_, 0) == HAL_OK) {
            adc2_value_ = HAL_ADC_GetValue(adc2_) / 4096.0f * 3.3f;
//            printf("%s: adc2_value_=%.2f V\n", name_, adc2_value_);
        }
        if (adc1_value_ >= 0 && adc2_value_ >= 0 && HAL_GetTick() - last_sample >= sample_interval) {
            last_sample += sample_interval;
            
            // ADC差分输入电压
            float input_valtage = adc1_value_ - adc2_value_;
            adc1_value_ = -1;
            adc2_value_ = -1;
            
            // 运放输入端电压
            float sample_valtage = input_valtage / 8.0f;
            // 电池电压
            float battery_valtage = sample_valtage / 100.0f * (100.0f + 10000.0f);
            
            // 补偿
            battery_valtage = offset_k * battery_valtage + offset_b;
            
            if (samples.size() == samples.max_size()) {
                float pop_value;
                if (samples.pop(pop_value) >= 0) {
                    samples_sum -= pop_value;
                }
            }
            if (samples.push(battery_valtage) >= 0) {
                samples_sum += battery_valtage;
            }
            count_sample++;
            if (HAL_ADC_Start(adc1_) != HAL_OK || HAL_ADC_Start(adc2_) != HAL_OK) {
                printf("%s: ADC start failed\n", name_);
            }
            
//            float valtage = samples_sum / samples.size();
//            printf("%s: valtage = %.2f V\n", name_, valtage);
            return 0;
        } else {
            return -1;
        }
    }
    
    float voltage() const {
        return samples_sum / samples.size();
    }
    
private:
    static const int sample_interval = 1;  // 采样间隔
    static const int average_count = 1000;    // 采样均值次数

    // 补偿系数
    const float offset_k = 1.000f;  // 实际值=测量值*offset_k+offset_b
    const float offset_b = 0;

    const char* name_;
    ADC_HandleTypeDef* adc1_;
    ADC_HandleTypeDef* adc2_;

    float adc1_value_ = -1;
    float adc2_value_ = -1;
    uint32_t last_sample = 0;

    QueueUnsafe<float, average_count> samples;
    float samples_sum = 0;
    uint32_t count_sample = 0;
};

//extern Battery battery;
    
}
