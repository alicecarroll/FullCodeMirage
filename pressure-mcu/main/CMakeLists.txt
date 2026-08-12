# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

#FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(
    SRCS "main.cpp" "pressure_control.cpp" "pressure_hardware.cpp"
    INCLUDE_DIRS 
        "." 
        "../../protocol"
    REQUIRES 
        # Logging
        log
        
        #Drivers
        driver

        #esp_driver_uart
        esp_driver_gpio 
        #esp_driver_spi
        esp_driver_ledc

        # FreeRTOS (vTaskDelay etc.)
        freertos)
