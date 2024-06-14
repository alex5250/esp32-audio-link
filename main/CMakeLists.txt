



idf_component_register(SRCS "main.c"
                    REQUIRES    fec  nvs_flash   app_update freertos freertos newlib  heap log  soc  esp_rom esp_common xtensa cxx
                    INCLUDE_DIRS ".")
