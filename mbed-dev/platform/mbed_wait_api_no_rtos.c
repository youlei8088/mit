/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This implementation of the wait functions will be compiled only
// if the RTOS is not present.
#ifndef MBED_CONF_RTOS_PRESENT

#include "platform/mbed_wait_api.h"
#include "hal/us_ticker_api.h"

void wait(float s) {
    wait_us(s * 1000000.0f);
}

void wait_ms(int ms) {
    wait_us(ms * 1000);
}

void wait_us(int us)
{
    const ticker_data_t *const ticker = get_us_ticker_data();
    uint32_t start = ticker_read(ticker);
    while ((ticker_read(ticker) - start) < (uint32_t)us);
}

#endif // #ifndef MBED_CONF_RTOS_PRESENT

