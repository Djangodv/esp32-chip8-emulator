idf_component_register(SRCS "main.cpp"
                  INCLUDE_DIRS "."
                  REQUIRES )

littlefs_create_partition_image(storage ../roms FLASH_IN_PROJECT)
