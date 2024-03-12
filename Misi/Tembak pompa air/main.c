#include "stm32f4xx.h"
#include "stdio.h"

#define TRIGGER_DISTANCE 50 // Jarak dalam sentimeter untuk menembak
#define PUMP_PIN GPIO_BSRR_BS0 // Pin untuk mengontrol pompa air (contoh: PA0)

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Aktifkan clock untuk port A

    GPIOA->MODER |= GPIO_MODER_MODER0_0; // Set pin PA0 sebagai output (pompa air)
}

void Delay(uint32_t milliseconds) {
    uint32_t i;
    for (i = 0; i < milliseconds * 4000; i++) {} // Delay kasar
}

int main(void) {
    GPIO_Init(); // Inisialisasi pin GPIO

    while (1) {
        // Baca sensor jarak (ultrasonik HC-SR04)
        // Implementasi baca sensor jarak di sini (misalnya, menggunakan delay untuk tujuan demo)

        Delay(1000); // Delay 1 detik

        // Cek apakah jarak memenuhi syarat untuk menembak
        if (/* Jarak <= TRIGGER_DISTANCE */) {
            GPIOA->BSRR = PUMP_PIN; // Aktifkan pompa air
            Delay(1000); // Aktifkan pompa air selama 1 detik (contoh)
            GPIOA->BSRR = PUMP_PIN << 16; // Matikan pompa air
        }
    }
}



//Kode ini lebih sederhana dan tidak menggunakan interrupt atau timer.
//Ini hanya menunggu selama 1 detik setelah setiap pembacaan sensor jarak (yang disimulasikan dengan fungsi Delay).
//Jika jarak yang dibaca sesuai dengan batas yang ditentukan oleh TRIGGER_DISTANCE, pompa air diaktifkan selama 1 detik.
//Pastikan untuk mengganti bagian "Implementasi baca sensor jarak di sini" dengan kode yang sesuai untuk membaca sensor jarak HC-SR04.

