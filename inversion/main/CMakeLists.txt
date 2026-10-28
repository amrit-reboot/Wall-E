idf_component_register(SRCS "main.c" "inversion_controller.c" "inversion_board.c"
    INCLUDE_DIRS "." REQUIRES esp_adc driver esp_timer)
