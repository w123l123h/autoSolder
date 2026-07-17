set(CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE SOURCES "./*.cc" "./*.c" "./*.cpp")

idf_component_register(SRCS ${SOURCES}
    PRIV_REQUIRES
    spi_flash driver freertos esp_adc esp_driver_mcpwm esp_driver_gpio esp_driver_spi
    esp_hw_support
    esp_timer
    esp_driver_touch_sens
    INCLUDE_DIRS "")
