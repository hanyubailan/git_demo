#include "Rand_Num.h"

int16_t RandRPM = 0;

int32_t generate_random_change(void) {
    return (rand() % 61) - 30;  // ���� [-10, 10] �������
}

int32_t abs_ChangeNum_value(int current_value) {
    int change = generate_random_change();  // ��������仯ֵ
    int new_value = current_value + change; // �仯���ֵ

    if (new_value < 0) {                    // ���Ʊ仯��Χ
        new_value = 0;
    } else if (new_value > MAX_absRPM) {
        new_value = MAX_absRPM;
    }

    return new_value;
}
