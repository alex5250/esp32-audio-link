idf_component_register(SRCS "audio-link.c"
                    REQUIRES esp_actions audio_sal audio_stream audio_hal audio_board audio_recorder input_key_service esp-idf-speex esp_peripherals audio_flash_tone  esp_wifi
                    INCLUDE_DIRS "include")
