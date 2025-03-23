set(TARGET drivers)

# IDEA: Componentize this further into LL, HAL, CMSIS and BSP.

set(SOURCES
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dac.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smbus_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lptim.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hcd.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_adc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pssi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smbus.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_usart_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
#    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_msp_template.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_crc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pka.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_opamp.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rng.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ltdc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gfxmmu.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma2d.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_fmc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_comp.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sai.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sram.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_mmc_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_i2c.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_mmc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_wwdg.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tsc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dcmi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lcd.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ospi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dsi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_ltdc_ex.c
    ./STM32L4xx_HAL_Driver/Src/Legacy/stm32l4xx_hal_can.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_comp.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_nand.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng_ex.c
#    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_timebase_tim_template.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lpuart.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_sdmmc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rtc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smartcard_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_sd_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_irda.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_crs.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_smartcard.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_firewall.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pka.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_swpmi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_swpmi.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dfsdm_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma2d.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_nor.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    ./STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    ./BSP/B-L475E-IOT01/stm32l475e_iot01_bus.c
    ./BSP/B-L475E-IOT01/stm32l475e_iot01.c
    ./BSP/B-L475E-IOT01/stm32l475e_iot01_env_sensors.c
    ./BSP/B-L475E-IOT01/stm32l475e_iot01_motion_sensors.c
    ./BSP/B-L475E-IOT01/stm32l475e_iot01_audio.c
#    ./BSP/B-L475E-IOT01/stm32l475e_iot01_qspi.c
    ./BSP/Components/lps22hb/lps22hb_reg.c
    ./BSP/Components/lps22hb/lps22hb.c
#    ./BSP/Components/es_wifi/es_wifi.c
    ./BSP/Components/hts221/hts221.c
    ./BSP/Components/hts221/hts221_reg.c
    ./BSP/Components/lis3mdl/lis3mdl.c
    ./BSP/Components/lis3mdl/lis3mdl_reg.c
    ./BSP/Components/lsm6dsl/lsm6dsl.c
    ./BSP/Components/lsm6dsl/lsm6dsl_reg.c
# Time of Flight sensor is not well componentized (parts of it are in the demo app)
#    ./BSP/Components/vl53l0x/vl53l0x_api.c
#    ./BSP/Components/vl53l0x/vl53l0x_api_calibration.c
#    ./BSP/Components/vl53l0x/vl53l0x_api_strings.c
#    ./BSP/Components/vl53l0x/vl53l0x_platform_log.c
#    ./BSP/Components/vl53l0x/vl53l0x_api_ranging.c
#    ./BSP/Components/vl53l0x/vl53l0x_api_core.c   
)

add_subdirectory(./Adafruit/Adafruit-GFX)       # Generic Graphics Interface
add_subdirectory(./Adafruit/Adafruit_ILI9341)   # LCD
add_subdirectory(./Adafruit/Adafruit_FT6206)    # Touch-screen
add_subdirectory(./Arduino/SD)
add_subdirectory(./Adafruit/Adafruit_VS1053)    # Audio Interface. Requires SD card.

add_library(${TARGET}
    ${SOURCES}
)

target_include_directories(${TARGET} 
    PUBLIC 
        ./CMSIS/Include
        ./CMSIS/Device/ST/STM32L4xx/Include
        ./CMSIS/RTOS/Template
        ./STM32L4xx_HAL_Driver/Inc
        ./BSP/B-L475E-IOT01
        ./BSP/Components/Common
        ./BSP/Components/es_wifi
        ../Configuration
)

target_compile_options(${TARGET} PRIVATE -Wno-sign-compare)
