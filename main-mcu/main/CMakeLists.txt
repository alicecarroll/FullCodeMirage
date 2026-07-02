idf_component_register(
    SRCS
        "main.cpp"
        "ConnectionLoss.cpp"
        "EthernetCom.cpp"
        #"Humidity.cpp"
        "Initialize.cpp"
        "Multiplexer.cpp"
        "Neopixel.cpp"
        "read_sensors.cpp"
        "SDCard.cpp"
        # "Slaves.cpp"
        "Uart.cpp"
        "watchdog.cpp"
        

    INCLUDE_DIRS
        "."

    REQUIRES
        # Logging
        log

        # Timers
        esp_timer

        # GPIO + SPI master driver
        driver

        esp_driver_uart
        esp_driver_gpio 
        esp_driver_spi
        esp_driver_ledc

        # SD card + FAT filesystem
        fatfs
        sdmmc

        # FreeRTOS (vTaskDelay etc.)
        freertos

        # WIZnet ioLibrary_Driver — clone into components/ and name the folder ioLibrary_Driver.
        # Add the folder name here exactly as it appears on disk.
        ioLibrary_Driver
)
