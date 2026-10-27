idf_component_register(
    SRCS "maze_solving.c" "state_follow.c" "state_junction.c" "state_turns.c"
         "turn_90.c" "turn_120.c" "junction.c" "node_memory.c" "display.c"
         "line_sensor.c" "motion.c" "pid.c" "tuning_http_server.c" "wifi_handler.c"
    INCLUDE_DIRS "include"
    REQUIRES sra-board-component esp_wifi esp_event esp_netif nvs_flash
             esp_http_server spiffs json mdns lwip esp_timer driver)
spiffs_create_partition_image(www "${CMAKE_CURRENT_SOURCE_DIR}/../frontend" FLASH_IN_PROJECT)
