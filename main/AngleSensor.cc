
#include "AngleSensor.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "AS5048A";

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
	:cs_pin_(cs_pin), sck_pin_(sck_pin), miso_pin_(miso_pin), mosi_pin_(mosi_pin)
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
	dev_cfg.mode = 1;                          // CPOL=0, CPHA=1
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
	// 读命令：地址 0x3FFF，R/W=0，带偶校验
	uint16_t cmd = 0x3FFF; // bit14=0 表示读
	uint8_t parity = as5048aParity(cmd);
	uint16_t tx_cmd = cmd | ((uint16_t)parity << 15);

	uint8_t tx_buf[2] = { (uint8_t)(tx_cmd >> 8), (uint8_t)(tx_cmd & 0xFF) };
	uint8_t rx_buf[2] = { 0 };

	// 第一帧：发送命令，同时收到的是无效数据，可丢弃
	spi_transaction_t t1 = {};
	t1.length = 16;
	t1.tx_buffer = tx_buf;
	t1.rx_buffer = rx_buf;
	t1.rxlength = 16;
	esp_err_t ret = spi_device_transmit(spi_dev_, &t1);
	if (ret != ESP_OK)
		return -1.0f;

	// 第二帧：发送任意值（如0x0000），接收真实角度
	uint16_t dummy = 0x0000;
	uint8_t tx_dummy[2] = { 0, 0 };
	uint8_t rx_data[2] = { 0 };
	spi_transaction_t t2 = {};
	t2.length = 16;
	t2.tx_buffer = tx_dummy;
	t2.rx_buffer = rx_data;
	t2.rxlength = 16;
	ret = spi_device_transmit(spi_dev_, &t2);
	if (ret != ESP_OK)
		return -1.0f;

	// 大端组合 16 位
	uint16_t raw = ((uint16_t)rx_data[0] << 8) | rx_data[1];

	// 校验奇偶（可选）
	uint8_t p = (raw >> 15) & 1;
	if (p != as5048aParity(raw & 0x7FFF))
	{
		return -1.0f;
	}

	uint16_t angle_raw = raw & 0x3FFF;
	return (float)angle_raw / 16384.0f * 2.0f * M_PI;
}