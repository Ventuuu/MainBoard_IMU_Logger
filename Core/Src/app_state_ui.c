#include "app_state_ui.h"
#include "main.h"

static void UpdateStateLed(AppState state)
{
    uint32_t now = HAL_GetTick();
    uint32_t blink_interval_ms = 0U;

    if ((storage_full_latched != 0U) && (state != STATE_FACTORY_ERASE))
    {
        LED_On(LED_GREEN);
        LED_On(LED_RED);
        return;
    }

    if ((ble_sync_abort_led_until_ms != 0U) &&
        ((int32_t)(now - ble_sync_abort_led_until_ms) < 0))
    {
        LED_Off(LED_GREEN);
        if ((now - ble_sync_abort_led_last_toggle_ms) >= 150U)
        {
            ble_sync_abort_led_last_toggle_ms = now;
            LED_Toggle(LED_RED);
        }
        return;
    }
    else if (ble_sync_abort_led_until_ms != 0U)
    {
        ble_sync_abort_led_until_ms = 0U;
        LED_Off(LED_RED);
        state_led_initialized = 0U;
    }

    if ((state_led_initialized == 0U) || (state != previous_state_led))
    {
        state_led_initialized = 1U;
        previous_state_led = state;
        state_led_last_toggle_ms = now;

        switch (state)
        {
            case STATE_IDLE:
                LED_Off(LED_GREEN);
                return;

            case STATE_ACQUISITION:
                LED_On(LED_GREEN);
                return;

            case STATE_USB_CONNECTED:
            case STATE_DOWNLOAD:
                LED_On(LED_GREEN);
                return;

            case STATE_BLE_SYNC:
                LED_Off(LED_RED);
                LED_On(LED_GREEN);
                return;

            case STATE_FACTORY_ERASE:
                LED_Off(LED_RED);
                LED_On(LED_GREEN);
                return;

            default:
                LED_Off(LED_GREEN);
                return;
        }
    }

    switch (state)
    {
        case STATE_IDLE:
            LED_Off(LED_GREEN);
            break;

        case STATE_ACQUISITION:
            LED_On(LED_GREEN);
            break;

        case STATE_USB_CONNECTED:
            blink_interval_ms = 500U;
            break;

        case STATE_DOWNLOAD:
            blink_interval_ms = 125U;
            break;

        case STATE_BLE_SYNC:
            blink_interval_ms = 250U;
            break;

        case STATE_FACTORY_ERASE:
            blink_interval_ms = 250U;
            break;

        default:
            LED_Off(LED_GREEN);
            break;
    }

    if ((blink_interval_ms != 0U) &&
        ((now - state_led_last_toggle_ms) >= blink_interval_ms))
    {
        state_led_last_toggle_ms = now;
        LED_Toggle(LED_GREEN);
    }
}

static void UserButton_Process(uint32_t now_ms)
{
    GPIO_PinState pin_state;

    if ((factory_erase_in_progress != 0U) ||
        (user_button_pressed == 0U))
    {
        return;
    }

    if (user_button_press_state == STATE_IDLE)
    {
        start_acquisition_requested = 0U;
        ble_sync_requested = 0U;
    }

    pin_state = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    if ((pin_state == GPIO_PIN_RESET) &&
        ((now_ms - user_button_last_event_ms) >= USER_BUTTON_DEBOUNCE_MS))
    {
        if ((user_button_long_press_triggered == 0U) &&
            ((now_ms - user_button_press_start_ms) >= USER_BUTTON_LONG_PRESS_MS) &&
            (user_button_press_state == STATE_IDLE) &&
            (current_state == STATE_IDLE) &&
            (usb_flag == 0U) &&
            (ble_sync_active == 0U) &&
            (microphone_active == 0U))
        {
            user_button_long_press_triggered = 1U;
            factory_erase_requested = 1U;
            button_long_press_count++;
            factory_erase_request_count++;
        }

        AppState pressed_state = user_button_press_state;
        uint8_t long_press_triggered = user_button_long_press_triggered;

        user_button_pressed = 0U;
        user_button_release_pending = 0U;
        user_button_long_press_triggered = 0U;
        user_button_press_start_ms = 0U;
        user_button_press_state = STATE_IDLE;
        user_button_last_event_ms = now_ms;

        if (long_press_triggered == 0U)
        {
            UserButton_HandleShortPress(pressed_state);
        }
        return;
    }

    if ((pin_state == GPIO_PIN_SET) &&
        (user_button_release_pending != 0U))
    {
        user_button_release_pending = 0U;
    }

    if ((user_button_long_press_triggered == 0U) &&
        ((now_ms - user_button_press_start_ms) >= USER_BUTTON_LONG_PRESS_MS) &&
        (user_button_press_state == STATE_IDLE) &&
        (current_state == STATE_IDLE) &&
        (usb_flag == 0U) &&
        (ble_sync_active == 0U) &&
        (microphone_active == 0U))
    {
        user_button_long_press_triggered = 1U;
        factory_erase_requested = 1U;
        button_long_press_count++;
        factory_erase_request_count++;
    }
}

static void UserButton_HandleShortPress(AppState pressed_state)
{
    button_short_press_count++;

    switch (pressed_state)
    {
        case STATE_IDLE:
        case STATE_ACQUISITION:
            ble_sync_requested = 1U;
            break;

        case STATE_BLE_SYNC:
            if (ble_sync_active != 0U)
            {
                ble_sync_abort_requested = 1U;
            }
            break;

        case STATE_USB_CONNECTED:
            exit_flag = 0;
            download_requested = 1U;
            download_request_count++;
            break;

        default:
            break;
    }
}

