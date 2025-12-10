idf_component_register(SRCS "main.cpp" "Ili9341Display.cpp"
                    INCLUDE_DIRS "."
					REQUIRES esp_lcd_ili9341 )

littlefs_create_partition_image(storage ../roms FLASH_IN_PROJECT)
