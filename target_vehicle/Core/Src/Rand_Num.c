#include "Rand_Num.h"

int16_t RandRPM = 0;

int32_t generate_random_change(void) {
    return (rand() % 61) - 30;  // 生成 [-10, 10] 的随机数
}

int32_t abs_ChangeNum_value(int current_value) {
    int change = generate_random_change();  // 生成随机变化值
    int new_value = current_value + change; // 变化后的值

    if (new_value < 0) {                    // 限制变化范围
        new_value = 0;
    } else if (new_value > MAX_absRPM) {
        new_value = MAX_absRPM;
    }

    return new_value;
}
