#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

#define PACKET_SIZE 2054
#define SAMPLES 1024

uint16_t adc_data[SAMPLES];
uint8_t packet[PACKET_SIZE];

uint16_t gain_value = 10;     // Replace with actual gain input
uint16_t trigger_value = 2500; // Replace with actual trigger logic

void SystemClock_Config(void);
void Error_Handler(void);

// Simulated parallel ADC read from AD9215
void read_adc_samples() {
    for (int i = 0; i < SAMPLES; i++) {
        // Replace this with actual parallel data read (e.g., from GPIO)
        adc_data[i] = simulate_adc_read();
    }
}

uint16_t simulate_adc_read() {
    // Dummy function to simulate ADC value
    static uint16_t value = 0;
    value += 100;
    return value & 0x7FFF;
}

void build_packet() {
    packet[0] = 0xAA;
    packet[1] = 0x55;

    for (int i = 0; i < SAMPLES; i++) {
        packet[2 + 2*i] = adc_data[i] & 0xFF;
        packet[3 + 2*i] = (adc_data[i] >> 8) & 0xFF;
    }

    packet[2050] = gain_value & 0xFF;
    packet[2051] = (gain_value >> 8) & 0xFF;

    packet[2052] = trigger_value & 0xFF;
    packet[2053] = (trigger_value >> 8) & 0xFF;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();  // Replace with your UART port

    while (1) {
        read_adc_samples();
        build_packet();
        HAL_UART_Transmit(&huart2, packet, PACKET_SIZE, HAL_MAX_DELAY);
        HAL_Delay(100); // Adjust to your sample rate
    }
}

"""
uint16_t read_adc_channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return val;
}

void update_gain_and_trigger() {
    gain_value = read_adc_channel(ADC_CHANNEL_5);     // e.g., potentiometer for gain
    trigger_value = read_adc_channel(ADC_CHANNEL_6);  // e.g., potentiometer for trigger
}
"""
