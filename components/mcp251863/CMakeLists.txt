idf_component_register(SRCS "mcp251863.c" "MCP251XFD.c"
                    REQUIRES esp_timer driver esp_driver_spi
                    INCLUDE_DIRS "include")
