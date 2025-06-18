# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_TLX493D input_tlx493d.c)

target_sources_ifdef(CONFIG_ZMK_INPUT_TLX493D_IDLE_SLEEPER app PRIVATE zmk_tlx493d_idle_sleeper.c)
