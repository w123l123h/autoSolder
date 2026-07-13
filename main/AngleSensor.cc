
#include "AngleSensor.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "AS5048A";

static uint8_t as5048aParity(uint16_t value)
{
	uint8_t count = 0;
	for (int i = 0; i < 16; i++)
	{
		if (value & 0x1)
			count++;
		value >>= 1;
	}
	return count & 0x1;
}

AngleSensor::AngleSensor(gpio_num_t cs_pin, gpio_num_t sck_pin, gpio_num_t miso_pin, gpio_num_t mosi_pin)
	: cs_pin_(cs_pin), sck_pin_(sck_pin), miso_pin_(miso_pin), mosi_pin_(mosi_pin)
{
	spi_bus_config_t bus_cfg = {};
	bus_cfg.mosi_io_num = mosi_pin_;
	bus_cfg.miso_io_num = miso_pin_;
	bus_cfg.sclk_io_num = sck_pin_;
	bus_cfg.quadwp_io_num = -1;
	bus_cfg.quadhd_io_num = -1;
	bus_cfg.max_transfer_sz = 16;
	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

	spi_device_interface_config_t dev_cfg = {};
	dev_cfg.mode = 1;						   // CPOL=0, CPHA=1
	dev_cfg.clock_speed_hz = 10 * 1000 * 1000; // 10 MHz
	dev_cfg.spics_io_num = cs_pin_;
	dev_cfg.queue_size = 1;
	dev_cfg.cs_ena_pretrans = 1; // CS setup time
	dev_cfg.command_bits = 0;
	dev_cfg.address_bits = 0;
	dev_cfg.dummy_bits = 0;

	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev_));
	ESP_LOGI(TAG, "SPI initialized");
}

float AngleSensor::getAngle()
{
    // 启用只读模式：发送 0xFFFF，让 MOSI 的所有位都为高电平
    uint16_t tx_data = 0xFFFF;  // 原来发送的是命令 0x3FFF
    uint16_t rx_data = 0;

    spi_transaction_t t = {};
    t.length = 16;              // 只需要 16 位（原来需要两次，现在一次）
    t.tx_buffer = (uint8_t*)&tx_data;
    t.rx_buffer = (uint8_t*)&rx_data;
    t.rxlength = 16;

    esp_err_t ret = spi_device_polling_transmit(spi_dev_, &t);
    if (ret != ESP_OK)
        return -1.0f;

    // 注意：rx_data 是大端序（MSB first），如果你的 SPI 配置为 MSB first，直接使用即可
    // 但为了安全，还是用字节数组组合（参考之前的方法）
    // 这里用字节数组方式：
    uint8_t* rx_bytes = (uint8_t*)&rx_data;
    uint16_t raw = ((uint16_t)rx_bytes[0] << 8) | rx_bytes[1];

    // 奇偶校验（与之前相同）
    if (((raw >> 15) & 1) != as5048aParity(raw & 0x7FFF))
        return -1.0f;

    static const float ANGLE_SCALE = (2.0f * M_PI) / 16384.0f;
    return (float)(raw & 0x3FFF) * ANGLE_SCALE;
}