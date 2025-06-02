# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_INPUT_TLX493D tlx493d.c)
