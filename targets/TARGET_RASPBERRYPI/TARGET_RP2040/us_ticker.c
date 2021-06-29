/*
 * Copyright (c) 2013 Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list
 *      of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form, except as embedded into a Nordic Semiconductor ASA
 *      integrated circuit in a product or a software update for such product, must reproduce
 *      the above copyright notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without specific prior
 *      written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Nordic Semiconductor ASA integrated circuit.
 *
 *   5. Any software provided in binary or object form under this license must not be reverse
 *      engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "us_ticker_api.h"
#include "mbed_critical.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

#define US_TICKER_COUNTER_BITS        32u
#define US_TICKER_FREQ                1000000

/* us ticker is driven by 1MHz clock and counter length is 32 bits */
const ticker_info_t* us_ticker_get_info()
{
    static const ticker_info_t info = {
        US_TICKER_FREQ,
        US_TICKER_COUNTER_BITS
    };
    return &info;
}

static const uint8_t alarm_num = 0;

static timestamp_t last_timestamp = 0;

static void us_ticker_irq_handler_internal(uint alarm_src) {
    if (alarm_num == alarm_src) {
        us_ticker_irq_handler();
    }
}

static void hardware_alarm_irq_handler_internal()
{
    // Determine which timer this IRQ is for
    uint32_t ipsr;
    __asm volatile ("mrs %0, ipsr" : "=r" (ipsr)::);
    uint alarm_id = (ipsr & 0x3fu) - 16;
    check_hardware_alarm_num_param(alarm_id);

    spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
    uint32_t save = spin_lock_blocking(lock);
    // Clear the timer IRQ (inside lock, because we check whether we have handled the IRQ yet in alarm_set by looking at the interrupt status
    timer_hw->intr = 1u << alarm_id;

    spin_unlock(lock, save);

    us_ticker_irq_handler_internal(alarm_num);
}

static void hardware_alarm_set_callback_internal()
{
    spin_lock_t *lock = spin_lock_instance(PICO_SPINLOCK_ID_TIMER);
    uint32_t save = spin_lock_blocking(lock);
    irq_set_exclusive_handler(alarm_num, hardware_alarm_irq_handler_internal);
    irq_set_enabled(alarm_num, true);
    hw_set_bits(&timer_hw->inte, 1u << alarm_num);
    spin_unlock(lock, save);
}

void us_ticker_init(void)
{
    hardware_alarm_claim(alarm_num);
    hardware_alarm_set_callback_internal();
}

uint32_t us_ticker_read()
{
    return time_us_32();
}

void wrap_alarm(void)
{
    uint64_t current_time = time_us_64();
    uint64_t wrapped_time = current_time - 0xFFFFFFFF;
    timer_hw->timelw = (uint32_t)wrapped_time;
    timer_hw->timehw = 0;
}

void us_ticker_set_interrupt(timestamp_t timestamp)
{
    core_util_critical_section_enter();
    // update the current timestamp
    if (timestamp <= last_timestamp) {
        //If we are here, it means that the 32 bit timer already wrapped
        //Now we need to wrap the 64 bit timer, as it was a 32 bit one.

        //First we set the maximum target to let the SDK always enable the alarm.
        //Providing the maximum possible target will make the condition (now >= t)
        //inside hardware_alarm_set_target() function evaluate false and will always allow to set the alarm.
        absolute_time_t max_target = { 0xFFFFFFFFFFFFFFFF };

        if (last_timestamp != 0) {
            //Wrap the 64 bit alarm to follow the 32 bit one
            wrap_alarm();
        }

        hardware_alarm_set_target(alarm_num, max_target);

        //Now set the proper target in the alarm register after the 2 timers have been wrapped
        timer_hw->alarm[alarm_num] = timestamp;
    } else {
        absolute_time_t target = { timestamp };
        hardware_alarm_set_target(alarm_num, target);
    }
    last_timestamp = timestamp;
    core_util_critical_section_exit();
}

void us_ticker_fire_interrupt(void)
{
    us_ticker_irq_handler();
}

void us_ticker_disable_interrupt(void)
{
    hardware_alarm_cancel(alarm_num);
}

void us_ticker_clear_interrupt(void)
{
    hardware_alarm_cancel(alarm_num);
}

void us_ticker_free(void)
{
    hardware_alarm_unclaim(alarm_num);
}
