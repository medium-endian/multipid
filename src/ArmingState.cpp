#include "../include/ArmingState.h"

ArmingState *arming_state_instance = nullptr;

static const inline bool state_transition_triggered(channels_t *input) {
    if (*input[THROTTLE_CHANNEL] >   25) return false;
    if (*input[ROLL_CHANNEL]     <  475) return false;
    /* No typo - roll is different from the other channels when not inverted */
    if (*input[PITCH_CHANNEL]    > -475) return false;
    if (*input[YAW_CHANNEL]      > -475) return false;
    return true;
}

void update_state() {
    bool triggered = state_transition_triggered(arming_state_instance->channels);
    switch (arming_state_instance->internal_state) {
        case ARMED:
            if (triggered) {
                arming_state_instance->internal_state = DISARMING;
                arming_state_instance->state_change_time = millis();
            }
            break;

        case DISARMED:
            if (triggered) {
                arming_state_instance->internal_state = ARMING;
                arming_state_instance->state_change_time = millis();
            }
            break;

        case DISARMING:
            if (triggered) {
                if ((millis() - arming_state_instance->state_change_time)
                        > arming_state_instance->DISARM_TIMEOUT_MS) {
                    arming_state_instance->internal_state = DISARMED;
                } else break;
            } else {
                arming_state_instance->internal_state = ARMED;
            }
            break;

        case ARMING:
            if (triggered) {
                if ((millis() - arming_state_instance->state_change_time)
                        > arming_state_instance->ARM_TIMEOUT_MS) {
                    arming_state_instance->internal_state = ARMED;
                } else break;
            } else {
                arming_state_instance->internal_state = DISARMED;
            }
            break;
    }
}

ArmingState::ArmingState(channels_t *channels)
    : channels(channels) {
        if (!state_change_timer.begin(update_state, INTERVAL_US)) {
            error_blink(STATE_TIMER_HARDWARE_BUSY,
                    "Could not set up interval timer for arming state update!");
        }
        arming_state_instance = this;
}

state_t ArmingState::get_state() {
    switch (internal_state) {
        case ARMED:
        case DISARMING:
            return ARMED;
        case DISARMED:
        case ARMING:
            return DISARMED;
    }
    return DISARMED;
}
