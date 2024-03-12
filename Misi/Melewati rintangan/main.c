
#include "stm32f4xx.h"

#define LEFT_SENSOR_PIN  GPIO_PIN_0  // Pin untuk sensor ultrasonik kiri
#define RIGHT_SENSOR_PIN GPIO_PIN_1  // Pin untuk sensor ultrasonik kanan
#define LEFT_MOTOR_PIN   GPIO_PIN_2  // Pin untuk motor kiri
#define RIGHT_MOTOR_PIN  GPIO_PIN_3  // Pin untuk motor kanan

void GPIO_Init(void) {
    // Aktifkan clock untuk port A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Konfigurasi pin sebagai input untuk sensor ultrasonik
    GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk);

    // Konfigurasi pin sebagai output untuk mengendalikan motor
    GPIOA->MODER |= GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0;
}

void MoveForward(void) {
    // Aktifkan motor untuk bergerak maju
    GPIOA->BSRR = LEFT_MOTOR_PIN;
    GPIOA->BSRR = RIGHT_MOTOR_PIN << 16;
}

void TurnLeft(void) {
    // Aktifkan motor untuk berbelok ke kiri
    GPIOA->BSRR = LEFT_MOTOR_PIN << 16;
    GPIOA->BSRR = RIGHT_MOTOR_PIN;
}

void TurnRight(void) {
    // Aktifkan motor untuk berbelok ke kanan
    GPIOA->BSRR = LEFT_MOTOR_PIN;
    GPIOA->BSRR = RIGHT_MOTOR_PIN << 16;
}

int main(void) {
    GPIO_Init(); // Inisialisasi pin GPIO

    while (1) {
        // Baca kondisi dari sensor ultrasonik
        uint16_t left_sensor = GPIOA->IDR & LEFT_SENSOR_PIN;
        uint16_t right_sensor = GPIOA->IDR & RIGHT_SENSOR_PIN;

        // Jika kedua sensor tidak mendeteksi rintangan, maka maju
        if (!left_sensor && !right_sensor) {
            MoveForward();
        }
        // Jika hanya sensor kiri yang mendeteksi rintangan, maka belok ke kanan
        else if (left_sensor) {
            TurnRight();
        }
        // Jika hanya sensor kanan yang mendeteksi rintangan, maka belok ke kiri
        else if (right_sensor) {
            TurnLeft();
        }
    }
}


//Pastikan untuk menghubungkan sensor ultrasonik ke pin yang sesuai pada mikrokontroler dan motor ke pin yang benar juga.
//Program ini memberikan contoh sederhana tentang bagaimana mengendalikan gerakan robot untuk melewati jalur yang kompleks dan menghindari rintangan menggunakan sensor ultrasonik.
