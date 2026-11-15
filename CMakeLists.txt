add_subdirectory(src/drivers)

target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_MOUSE_SETTING app PRIVATE src/behaviors/behavior_mouse_setting.c)

zephyr_include_directories(include)
