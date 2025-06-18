/**
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#if DT_HAS_COMPAT_STATUS_OKAY(infineon_tlx493d)

LOG_MODULE_DECLARE(tlx493d);

static const struct device *tlx493d_dev = DEVICE_DT_GET_ANY(infineon_tlx493d);

static int tlx493d_idle_sleeper_listener(const zmk_event_t *eh)
{
    if (as_zmk_activity_state_changed(eh)) {
        const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
        
        if (!device_is_ready(tlx493d_dev)) {
            LOG_WRN("TLX493D device not ready for power management");
            return 0;
        }
        
        switch (ev->state) {
        case ZMK_ACTIVITY_ACTIVE:
            LOG_DBG("Activity state: ACTIVE - waking TLX493D");
            pm_device_action_run(tlx493d_dev, PM_DEVICE_ACTION_RESUME);
            break;
            
        case ZMK_ACTIVITY_IDLE:
            LOG_DBG("Activity state: IDLE - keeping TLX493D awake");
            break;
            
        case ZMK_ACTIVITY_SLEEP:
            LOG_DBG("Activity state: SLEEP - putting TLX493D to sleep");
            pm_device_action_run(tlx493d_dev, PM_DEVICE_ACTION_SUSPEND);
            break;
            
        default:
            LOG_WRN("Unknown activity state: %d", ev->state);
            break;
        }
    }
    
    return 0;
}

ZMK_LISTENER(tlx493d_idle_sleeper, tlx493d_idle_sleeper_listener)
ZMK_SUBSCRIPTION(tlx493d_idle_sleeper, zmk_activity_state_changed)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(infineon_tlx493d) */