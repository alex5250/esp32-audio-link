



idf_component_register(SRCS "main.c"
                    REQUIRES esp32-wifi-raw   gsm_codec fec microphone nvs_flash   app_update freertos freertos newlib  heap log  soc  esp_rom esp_common xtensa cxx
                    INCLUDE_DIRS ".")
