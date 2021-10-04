# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(mis2dh.c)
zephyr_library_sources(mis2dh_i2c.c)
zephyr_library_sources(mis2dh_spi.c)
zephyr_library_sources_ifdef(CONFIG_MIS2DH_TRIGGER    mis2dh_trigger.c)
