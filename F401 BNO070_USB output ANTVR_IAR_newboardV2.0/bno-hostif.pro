#-------------------------------------------------
#
# Project created by QtCreator 2014-10-09T19:27:44
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = bno-hostif
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += \
    sensorhub_hid.c \
    sensorhub.c \
    src/sensorhub_hid.c \
    src/sensorhub.c \
    examples/linux/main.c \
    firmware/1000-3251_1.2.0.c \
    examples/linux/bnodfu.c \
    examples/linux/bnotest.c \
    examples/linux/common.c \
    examples/nucleo-f401re/Src/bno_init.c \
    examples/nucleo-f401re/Src/freertos.c \
    examples/nucleo-f401re/Src/i2c_master_transfer.c \
    examples/nucleo-f401re/Src/main.c \
    examples/nucleo-f401re/Src/stm32f4xx_hal_msp.c \
    examples/nucleo-f401re/Src/stm32f4xx_it.c \
    examples/stm32f205/Src/adc.c \
    examples/stm32f205/Src/bno_callbacks.c \
    examples/stm32f205/Src/freertos.c \
    examples/stm32f205/Src/gpio.c \
    examples/stm32f205/Src/i2c.c \
    examples/stm32f205/Src/i2c_master_transfer.c \
    examples/stm32f205/Src/iwdg.c \
    examples/stm32f205/Src/main.c \
    examples/stm32f205/Src/stm32f2xx_it.c \
    examples/stm32f205/Src/tim.c \
    examples/stm32f205/Src/usart.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_irda.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_msp_template.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nand.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nor.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pccard.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_smartcard.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c \
    examples/nucleo-f401re/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

OTHER_FILES += \
    README.md \
    examples/nucleo-f401re/README.md \
    examples/linux/README.md

HEADERS += \
    sensorhub.h \
    sensorhub_hid.h \
    sensorhub_platform.h \
    src/sensorhub_hid.h \
    src/sensorhub_platform.h \
    src/sensorhub.h \
    examples/linux/common.h
