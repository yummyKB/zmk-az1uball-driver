zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_AZ1UBALL src/az1uball.c)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)