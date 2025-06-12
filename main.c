#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22           // pino SCL
#define I2C_MASTER_SDA_IO 21           // pino SDA
#define I2C_MASTER_NUM I2C_NUM_0       // número do barramento I2C
#define I2C_MASTER_FREQ_HZ 100000      // frequência 100kHz
#define BMP280_ADDR 0x76               // endereço I2C do BMP280 (pode ser 0x76 ou 0x77)

static const char *TAG = "BMP280";

// Estrutura para dados de calibração
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

bmp280_calib_data_t calib_data;
int32_t t_fine;

// Inicializa I2C
esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Escreve um byte em um registrador do BMP280
esp_err_t bmp280_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_ADDR, write_buf, 2, 1000 / portTICK_PERIOD_MS);
}

// Lê bytes de um registrador do BMP280
esp_err_t bmp280_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// Inicializa o BMP280
esp_err_t bmp280_init() {
    uint8_t id = 0;
    esp_err_t err = bmp280_read_bytes(0xD0, &id, 1); // registrador ID
    if (err != ESP_OK) return err;
    if (id != 0x58) {
        ESP_LOGE(TAG, "Sensor BMP280 nao encontrado. ID lido: 0x%02X", id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP280 encontrado! ID: 0x%02X", id);

    // Configura sensor: modo normal, oversampling x1
    err = bmp280_write_byte(0xF4, 0x27); // ctrl_meas
    if (err != ESP_OK) return err;
    err = bmp280_write_byte(0xF5, 0xA0); // config
    return err;
}

// Lê os dados de calibração do sensor
esp_err_t bmp280_read_calib_data() {
    uint8_t calib[24];
    esp_err_t err = bmp280_read_bytes(0x88, calib, 24);
    if (err != ESP_OK) return err;

    calib_data.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    calib_data.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    calib_data.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    calib_data.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    calib_data.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    calib_data.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    calib_data.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    calib_data.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    calib_data.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    calib_data.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    calib_data.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    calib_data.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    return ESP_OK;
}

// Compensação da temperatura
int32_t bmp280_compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Compensação da pressão
uint32_t bmp280_compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (uint32_t)p;
}

// Lê os dados do sensor e imprime temperatura e pressão
void bmp280_read_and_print() {
    uint8_t data[6];
    esp_err_t err = bmp280_read_bytes(0xF7, data, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro lendo dados BMP280");
        return;
    }

    int32_t adc_P = ((int32_t)(data[0]) << 12) | ((int32_t)(data[1]) << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)(data[3]) << 12) | ((int32_t)(data[4]) << 4) | (data[5] >> 4);

    int32_t temp = bmp280_compensate_temp(adc_T);
    uint32_t press = bmp280_compensate_pressure(adc_P);

    ESP_LOGI(TAG, "Temperatura: %.2f °C", temp / 100.0);
    ESP_LOGI(TAG, "Pressao: %.2f hPa", press / 25600.0);
}

// Função principal
void app_main() {
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar I2C");
        return;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    err = bmp280_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar BMP280");
        return;
    }

    err = bmp280_read_calib_data();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler dados de calibracao");
        return;
    }

    while (1) {
        bmp280_read_and_print();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
