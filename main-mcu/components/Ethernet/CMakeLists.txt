idf_component_register(
    SRCS 
        "wizsocket.c"
        "wizchip_conf.c"
        "w5500.c"

    INCLUDE_DIRS "."
    REQUIRES 
        
        log 

        freertos 

        #ioLibrary_Driver 

        #driver 
        #esp_driver_spi 
        #esp_driver_gpio 
    # deliberately NOT requiring lwip / esp_netif / esp_wifi
)