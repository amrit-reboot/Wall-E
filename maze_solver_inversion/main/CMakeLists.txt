idf_component_register(
    SRCS "main.c" "maze_controller.c" "maze_board.c" "maze_web.c" "maze_oled.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc esp_driver_gpio esp_driver_ledc esp_driver_i2c
             esp_wifi esp_event esp_netif esp_http_server nvs_flash json esp_timer
    EMBED_TXTFILES "../frontend/index.html")
