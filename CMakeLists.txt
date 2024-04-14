add_subdirectory(src/drivers)

target_sources_ifdef(CONFIG_ZMK_INPUT_MOUSE_PS2 app PRIVATE src/mouse/input_listener_ps2.c)
target_sources_ifdef(CONFIG_ZMK_INPUT_MOUSE_PS2 app PRIVATE src/behaviors/behavior_mouse_setting.c)

zephyr_include_directories(include)
