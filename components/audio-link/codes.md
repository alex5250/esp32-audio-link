void speex_encode_decode_task(audio_element_handle_t raw_read) {
    //ESP_LOGI(TAG, "================ Start encode =================");

    short in[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized input buffer");

    float input[FRAME_SIZE];
    ESP_LOGI(TAG, "Initialized float input buffer");

    char cbits[200];
    ESP_LOGI(TAG, "Initialized cbits buffer");

    int nbBytes;
    ESP_LOGI(TAG, "Initialized nbBytes variable");

    // Encoder state
    void *encoder_state = speex_encoder_init(&speex_nb_mode);
    if (!encoder_state) {
        ESP_LOGE(TAG, "Failed to initialize encoder");
        return;
    }
    ESP_LOGI(TAG, "Encoder initialized");


        // Set the quality to 8 (15 kbps)
    int tmp = 8;
    speex_encoder_ctl(encoder_state, SPEEX_SET_QUALITY, &tmp);
    ESP_LOGI(TAG, "Encoder quality set to 8");


   // Bit-stream structure
    SpeexBits enc_bits;
    speex_bits_init(&enc_bits);
    ESP_LOGI(TAG, "Encoder bits initialized");

// Buffer to store encoded data
    char encoded_data_buffer[PACKAGE_SIZE * 100]; // Adjust size as needed
    int encoded_data_offset = 0;
    ESP_LOGI(TAG, "Initialized encoded data buffer");

ESP_LOGI(TAG, "================ Encoding loop start =================");

    // Encoding loop
    while (1) {
        // Read raw audio data
        raw_stream_read(raw_read, (char *)in, FRAME_SIZE * sizeof(short));
        ESP_LOGI(TAG, "Read raw audio data");

       
        ESP_LOGI(TAG, "Converted input to float for Speex encoder");

        // Encode audio data
        speex_bits_reset(&enc_bits);
        ESP_LOGI(TAG, "Reset encoder bits");

        speex_encode(encoder_state, input, &enc_bits);
        ESP_LOGI(TAG, "Encoded audio data");

        // Write encoded bits to buffer
        nbBytes = speex_bits_write(&enc_bits, cbits, sizeof(cbits));
        ESP_LOGI(TAG, "Wrote encoded bits to buffer: %d bytes", nbBytes);


    

        // Optional: Break condition or sleep to prevent infinite looping
        if (continue_recording) break;
    }

       // Destroy encoder state
    speex_encoder_destroy(encoder_state);
    ESP_LOGI(TAG, "Destroyed encoder state");

    // Destroy bit-stream structure
    speex_bits_destroy(&enc_bits);
    ESP_LOGI(TAG, "Destroyed encoder bits");

    ESP_LOGI(TAG, "Encode done, total encoded size: %d bytes", encoded_data_offset);

    ESP_LOGI(TAG, "================ Encoding loop end =================");
}