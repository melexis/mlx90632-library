/**
 * @file
 * @brief Unit tests for reading data with virtual i2c communication from sensor
 * @internal
 *
 * @copyright (C) 2017 Melexis N.V.
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
 *
 * @endinternal
 *
 * @addtogroup mlx90632_unit_tests
 * @ingroup mlx90632
 * @{
 *
 * @details
 */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>

#include "mlx90632.h"
#include "mlx90632_extended_meas.h"

#include "mock_mlx90632_depends.h"

// Pointers should point here
int16_t ambient_new_raw = 0;
int16_t ambient_old_raw = 0;
int16_t object_new_raw = 0;
int16_t object_old_raw = 0;

void setUp(void)
{
    ambient_new_raw = 0;
    ambient_old_raw = 0;
    object_new_raw = 0;
    object_old_raw = 0;
}

void tearDown(void)
{
}

/** Test trigger measurement.
 */
void test_trigger_measurement_success(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready

    // Trigger measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_trigger_measurement());
}

/** Test different failure paths when triggering measurement.
 */
void test_trigger_measurement_errors(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready

    // Trigger measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement());

    // Trigger measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement());
}

/** Test wait for measurement with data ready.
 */
void test_wait_for_measurement_success(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready

    // Wait for measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT32(1, mlx90632_wait_for_measurement());
}

/** Test wait for measurement function, if data is ready and if it is not it waits one usleep period before retry.
 *
 * First we simulate data not ready with bit0 not set. After one period we flip the bit0 to 1 to indicate
 * data ready and wait for measurement should complete with success.
 */
void test_wait_for_measurement_one_wait(void)
{
    uint16_t reg_status_mock = 0x0C86; // cycle position 1 & data not ready
    uint16_t reg_status_mock1 = 0x0087; // cycle position 1 & data ready

    // Wait for measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    usleep_Expect(10000, 11000);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock1);

    TEST_ASSERT_EQUAL_INT32(1, mlx90632_wait_for_measurement());
}

/** Test failure path when waiting for measurement.
 */
void test_wait_for_measurement_error(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready

    // Wait for measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_wait_for_measurement());
}

/** Test sensor timeouts while waiting for measurement.
 *
 * If this happens in real life it means that after 100 tries sensor still did not indicate data ready, which probably
 * points to much larger problem than a simple timeout. Timeout is only valid if usleep is a lot shorter than default
 * values
 */
void test_wait_for_measurement_timeout(void)
{
    uint16_t reg_status_mock = 0x0C86; // cycle position 1 & data not ready
    int i;

    for (i = 0; i < MLX90632_MAX_NUMBER_MESUREMENT_READ_TRIES; ++i)
    {
        // Wait for measurement expectations
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

        usleep_Expect(10000, 11000);
    }

    TEST_ASSERT_EQUAL_INT32(-ETIMEDOUT, mlx90632_wait_for_measurement());
}

/** Test start measurement with data ready.
 */
void test_start_measurement_success(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT32(1, mlx90632_start_measurement());
}

/** Test start measurement function, if data is ready and if it is not it waits one usleep period before retry.
 *
 * First we simulate data not ready with bit0 not set. After one period we flip the bit0 to 1 to indicate
 * data ready and start measurement should complete with success.
 */
void test_start_measurement_one_wait(void)
{
    uint16_t reg_status_mock = 0x0C86; // cycle position 1 & data not reaady
    uint16_t reg_status_mock1 = 0x0087; // cycle position 1 & data ready

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    usleep_Expect(10000, 11000);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock1);

    TEST_ASSERT_EQUAL_INT32(1, mlx90632_start_measurement());
}

/** Test different failure paths in start_measurement */
void test_start_measurement_errors(void)
{
    uint16_t reg_status_mock = 0x0C86; // cycle position 1 & data not reaady

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement());

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement());

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement());
}

/** Test sensor timeouts while start_measure.
 *
 * If this happens in real life it means that after 100 tries sensor still did not indicate data ready, which probably
 * points to much larger problem than a simple timeout. Timeout is only valid if usleep is a lot shorter than default
 * values
 */
void test_start_measurement_timeout(void)
{
    uint16_t reg_status_mock = 0x0C06; // cycle position 1 & data not ready
    int i;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    for (i = 0; i < MLX90632_MAX_NUMBER_MESUREMENT_READ_TRIES; ++i)
    {
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

        usleep_Expect(10000, 11000);
    }

    TEST_ASSERT_EQUAL_INT32(-ETIMEDOUT, mlx90632_start_measurement());
}


/** Test read temperature from sensor without waiting procedure.
 *
 * Read ambient and object raw values considering channel 1 as new. Channel 2 values are read as old,
 * while channel 1 values are read as new.
 */
void test_read_temp_raw_wo_wait_ch1_success(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // Read Ambient raw expectations (based on cycle position 1)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations (based on cycle position 1)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_wo_wait(1, &ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

/** Test read temperature from sensor without waiting procedure.
 *
 * Read ambient and object raw values considering channel 2 as new. Channel 1 values are read as old,
 * while channel 2 values are read as new.
 */
void test_read_temp_raw_wo_wait_ch2_success(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // Read Ambient raw expectations (based on cycle position 2)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations (based on cycle position 2)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_wo_wait(2, &ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

/** Test failure path when reading temperature from sensor without waiting procedure.
 */
void test_read_temp_raw_wo_wait_error(void)
{
    int16_t ambient_new_mock = 22454;

    // Read Ambient raw expectations (based on cycle position 1)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_wo_wait(1, &ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));
}

/** Test whole start and read temperature from sensor procedure.
 *
 * Start measurement which returns channel 1 as new, then read ambient and object raw values.
 * Channel 2 values are read as old, while channel 1 values are read as new. Start measurement is triggered
 * and channel selection is done based on the register values in start measurement.
 */
void test_read_temp_raw_ch1_success(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

/** Test whole start and read temperature from sensor procedure.
 *
 * Start measurement which returns channel 2 as new, then read ambient and object raw values.
 * Channel 1 values are read as old, while channel 2 values are read as new. Start measurement is triggered
 * and channel selection is done based on the register values in start measurement.
 */
void test_read_temp_raw_ch2_success(void)
{
    uint16_t reg_status_mock = 0x008B; // cycle position 2 & data ready
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

/** Test Error output on whole start and read temperature from sensor procedure. */
void test_read_temp_raw_errors(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready
    int16_t ambient_new_mock = 22454;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));
}

/** Test reading channel 1 ambient values from sensor.
 *
 * Channel 2 values are read into old, while channel 1 values are read into new.
 */
void test_read_ambient_values_ch1_success(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
}

/** Test reading channel 2 ambient values from sensor.
 *
 * Channel 1 values are read into old, while channel 2 values are read into new.
 */
void test_read_ambient_values_ch2_success(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
}

/** Test error outputs when reading ambient values from sensor. */
void test_read_ambient_values_errors(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;

    // First read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));

    // Second read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));
}

/** Test reading channel 1 object values from sensor.
 *
 * Channel 2 values are read into old, while channel 1 values are read into new.
 */
void test_read_object_values_ch1_success(void)
{
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // Read Object raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

/** Test reading channel 2 object values from sensor.
 *
 * Channel 1 values are read into old, while channel 1 values are read into new
 */
void test_read_object_values_ch2_success(void)
{
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // Read Object raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_object_raw(2, &object_new_raw, &object_old_raw));

    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

/** Test error outputs when reading object values. */
void test_read_object_values_errors(void)
{
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // First read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    // Second read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    // Third read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_old_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    // Forth and last read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_old_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));
}

/** Test error outputs when reading object values. */
void test_read_object_error_ch(void)
{
    // Input retval is invalid
    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_read_temp_object_raw(3, &object_new_raw, &object_old_raw));
}

/** Test reading ambient values from sensor in extended range measurements.
 *
 */
void test_read_ambient_values_extended_success(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_ambient_raw_extended(&ambient_new_raw, &ambient_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
}

/** Test error outputs when reading ambient values from sensor. */
void test_read_ambient_values_extended_errors(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;

    // First read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_ambient_raw_extended(&ambient_new_raw, &ambient_old_raw));

    // Second read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_ambient_raw_extended(&ambient_new_raw, &ambient_old_raw));
}

void test_read_object_values_extended_success(void)
{
    int16_t object_mock_l1 = 250;
    int16_t object_mock_l2 = 260;
    int16_t object_mock_b1 = -25;
    int16_t object_mock_b2 = -35;
    int16_t object_mock_v1 = 4;
    int16_t object_mock_v2 = -2;

    // Read object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(287, object_new_raw);
}

void test_read_object_values_extended_errors(void)
{
    int16_t object_mock_l1 = 25000;
    int16_t object_mock_l2 = 26000;
    int16_t object_mock_b1 = -2500;
    int16_t object_mock_b2 = -3500;
    int16_t object_mock_v1 = 4000;
    int16_t object_mock_v2 = 2000;

    // Read object raw expectations

    // First read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // Second read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // Third read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // 4th read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // 5th read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // 6th read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_object_raw_extended(&object_new_raw));

    // Data overflow
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_read_temp_object_raw_extended(&object_new_raw));
}

/** Test read extended temperature from sensor without waiting procedure.
 */
void test_read_temp_raw_extended_wo_wait_success(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_mock_l1 = 250;
    int16_t object_mock_l2 = 260;
    int16_t object_mock_b1 = -25;
    int16_t object_mock_b2 = -35;
    int16_t object_mock_v1 = 4;
    int16_t object_mock_v2 = -2;

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_extended_wo_wait(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(287, object_new_raw);
}

/** Test different failure paths when reading extended temperature from sensor without waiting procedure.
 */
void test_read_temp_raw_extended_wo_wait_errors(void)
{
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_mock_l1 = 250;

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_extended_wo_wait(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_extended_wo_wait(&ambient_new_raw, &ambient_old_raw, &object_new_raw));
}

/** Test whole start and read extended temperature from sensor procedure.
 *
 * Start measurement which returns channel 19 as the last measurement, then read ambient and object raw values.
 */
void test_read_temp_raw_extended_success(void)
{
    uint16_t reg_status_mock1 = 0x00C5; // cycle position 17 & data ready
    uint16_t reg_status_mock2 = 0x00C9; // cycle position 18 & data ready
    uint16_t reg_status_mock3 = 0x00CF; // cycle position 19 & data ready
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_mock_l1 = 250;
    int16_t object_mock_l2 = 260;
    int16_t object_mock_b1 = -25;
    int16_t object_mock_b2 = -35;
    int16_t object_mock_v1 = 4;
    int16_t object_mock_v2 = -2;

    // 1st read is end of meas
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock3);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock3 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock3);

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    // Trigger the read_temp_raw_extended function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(287, object_new_raw);

    // 2nd read is end of meas
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock2);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock2 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock3);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock3 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock3);

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    // Trigger the read_temp_raw_extended function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(287, object_new_raw);

    // 3rd read is end of meas
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock1);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock1 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock2);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock2 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock3);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock3 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock3);

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    // Trigger the read_temp_raw_extended function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(287, object_new_raw);
}

/** Test whole start and read extended temperature from sensor procedure.
 *
 * Start measurement which returns channel 19 as the last measurement, then read ambient and object raw values.
 */
void test_read_temp_raw_extended_errors(void)
{
    uint16_t reg_status_mock1 = 0x00C5; // cycle position 17 & data ready
    uint16_t reg_status_mock2 = 0x00CF; // cycle position 19 & data ready
    int16_t ambient_new_mock = 22454;

    // 1st read returns an error
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock1, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // read_ambient_raw_extended returns an error
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock2);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock2 & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock2);

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw));
}

/** Test retries exhaust on whole start and read extended temperature from sensor procedure.
 */
void test_wait_for_measurement_retries_exhaust(void)
{
    uint16_t reg_status_mock = 0x0CC9; // cycle position 18 & data ready

    // 3 reads are not enough to reach the end of meas
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT32(-ETIMEDOUT, mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw));
}

/** Test trigger burst measurement.
 */
void test_trigger_measurement_burst_success(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected

    // Trigger burst measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_trigger_measurement_burst());
}

/** Test different failure paths when triggering burst measurement.
 */
void test_trigger_measurement_burst_errors(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected

    // Trigger burst measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement_burst());

    // Trigger burst measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement_burst());
}

/** Test wait for burst measurement with data ready.
 */
void test_wait_for_measurement_burst_success(void)
{
    uint16_t reg_status_mock = 0x000B; // cycle position 2 & data ready & device not busy

    // Wait for burst measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_wait_for_measurement_burst());
}

/** Test failure path when waiting for burst measurement.
 */
void test_wait_for_measurement_burst_error(void)
{
    uint16_t reg_status_mock = 0x000B; // cycle position 2 & data ready & device not busy

    // Wait for burst measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_wait_for_measurement_burst());
}

/** Test sensor timeouts while while waiting for burst measurement.
 *
 * If this happens in real life it means that after some tries sensor still did not indicate data ready, which probably
 * points to much larger problem than a simple timeout. Timeout is only valid if usleep is a lot shorter than default
 * values
 */
void test_wait_for_measurement_burst_timeout(void)
{
    uint16_t reg_status_mock = 0x0C06; // cycle position 1 & device busy
    int i;

    for (i = 0; i < MLX90632_MAX_NUMBER_MESUREMENT_READ_TRIES; ++i)
    {
        // Wait for measurement expectations
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

        usleep_Expect(10000, 11000);
    }

    TEST_ASSERT_EQUAL_INT32(-ETIMEDOUT, mlx90632_wait_for_measurement_burst());
}

/** Test start sleeping step measurement.
 */
void test_start_measurement_burst_success(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected
    uint16_t reg_status_mock = 0x000B; // cycle position 2 & data ready & device not busy
    uint16_t meas1_mock = 0x820D;
    uint16_t meas2_mock = 0x821D;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    msleep_Expect(1000);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_start_measurement_burst());
}

void test_start_measurement_burst_errors(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected
    uint16_t reg_status_mock = 0x000B; // cycle position 2 & data ready & device not busy
    uint16_t meas1_mock = 0x820D;
    uint16_t meas2_mock = 0x821D;

    //mlx90632_reg_ctrl read error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement_burst());

    //mlx90632_reg_ctrl write error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement_burst());

    //calculate measurement time error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement_burst());

    //mlx90632_reg_status read error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    msleep_Expect(1000);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_start_measurement_burst());
}

/** Test sensor timeouts while start_measure.
 *
 * If this happens in real life it means that after some tries sensor still did not indicate data ready, which probably
 * points to much larger problem than a simple timeout. Timeout is only valid if usleep is a lot shorter than default
 * values
 */
void test_start_measurement_burst_timeout(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected
    uint16_t reg_status_mock = 0x0C06; // cycle position 1 & device busy
    uint16_t meas1_mock = 0x820D;
    uint16_t meas2_mock = 0x821D;
    int i;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    msleep_Expect(1000);

    for (i = 0; i < MLX90632_MAX_NUMBER_MESUREMENT_READ_TRIES; ++i)
    {
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

        usleep_Expect(10000, 11000);
    }

    TEST_ASSERT_EQUAL_INT32(-ETIMEDOUT, mlx90632_start_measurement_burst());
}

/** Test trigger single measurement.
 */
void test_trigger_measurement_single_success(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected

    // Trigger single measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_SINGLE_MEAS, 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_trigger_measurement_single());
}

/** Test failure path when triggering single measurement.
 */
void test_trigger_measurement_single_errors(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected

    // Trigger single measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement_single());

    // Trigger single measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement_single());

    // Trigger single measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement_single());

    // Trigger single measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_SINGLE_MEAS, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_trigger_measurement_single());
}

/** Test whole start and read extended temperature in sleeping step mode from sensor procedure.
 *
 * Start measurement which returns channel 0 on success, then read ambient and object raw values.
 */
void test_read_temp_raw_extended_burst_success(void)
{
    uint16_t reg_ctrl_mock = 0x0112; // extended sleeping step meas selected
    uint16_t reg_status_mock = 0x01CF; // cycle position 19 & data ready & device not busy
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_mock_l1 = 250;
    int16_t object_mock_l2 = 260;
    int16_t object_mock_b1 = -25;
    int16_t object_mock_b2 = -35;
    int16_t object_mock_v1 = 4;
    int16_t object_mock_v2 = -2;
    uint16_t meas1_mock = 0x8300;
    uint16_t meas2_mock = 0x8312;
    uint16_t meas3_mock = 0x830C;

    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS3, &meas3_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas3_mock);

    msleep_Expect(750);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(18), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(17), (uint16_t*)&object_mock_l1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(17), (uint16_t*)&object_mock_b1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(18), (uint16_t*)&object_mock_b2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_b2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(18), (uint16_t*)&object_mock_l2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_l2);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(19), (uint16_t*)&object_mock_v1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v1);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(19), (uint16_t*)&object_mock_v2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_mock_v2);

    // Trigger the read_temp_raw_extended function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_extended_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(287, object_new_raw);
}


/** Test whole start and read extended temperature in sleeping step mode from sensor procedure.
 *
 * Start measurement which returns channel 0 on success, then read ambient and object raw values.
 */
void test_read_temp_raw_extended_burst_errors(void)
{
    uint16_t reg_ctrl_mock = 0x0112; // extended sleeping step meas selected
    uint16_t reg_status_mock = 0x01CF; // cycle position 19 & data ready & device not busy
    int16_t ambient_new_mock = 22454;
    uint16_t meas1_mock = 0x8300;
    uint16_t meas2_mock = 0x8312;
    uint16_t meas3_mock = 0x830C;

    // 1st read returns an error
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_extended_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw));

    // read_ambient_raw_extended returns an error
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS3, &meas3_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas3_mock);

    msleep_Expect(750);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(17), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_extended_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw));
}

void test_read_temp_raw_burst_success(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected
    uint16_t reg_status_mock = 0x010B; // cycle position 2 & data ready & device not busy
    int16_t ambient_new_mock = 22454;
    int16_t ambient_old_mock = 23030;
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;
    uint16_t meas1_mock = 0x820D;
    uint16_t meas2_mock = 0x821D;

    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    msleep_Expect(1000);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_old_mock);

    // Read Object raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(2), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_old_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_old_mock);

    // Trigger the read_temp_raw function
    TEST_ASSERT_EQUAL_INT32(0, mlx90632_read_temp_raw_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_raw);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_raw);
}

void test_read_temp_raw_burst_errors(void)
{
    uint16_t reg_ctrl_mock = 0x0002; // medical sleeping step meas selected
    uint16_t reg_status_mock = 0x010B; // cycle position 2 & data ready & device not busy
    int16_t ambient_new_mock = 22454;
    uint16_t meas1_mock = 0x820D;
    uint16_t meas2_mock = 0x821D;

    // 1st read returns an error
    //Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock | MLX90632_START_BURST_MEAS, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&meas2_mock);

    msleep_Expect(1000);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Read Ambient raw expectations (based on above cycle position)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_read_temp_raw_burst(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));
}

void test_calculate_dataset_ready_time_medical_success(void)
{
    uint16_t reg_ctrl_medb_mock = 0x0002; // medical sleeping step meas selected
    uint16_t med_meas1_mock[] = { 0x800D, 0x810D, 0x820D, 0x830D, 0x840D, 0x850D, 0x860D, 0x870D };
    uint16_t med_meas2_mock[] = { 0x801D, 0x811D, 0x821D, 0x831D, 0x841D, 0x851D, 0x861D, 0x871D };
    int med_waiting_time[] = { 4000, 2000, 1000, 500, 250, 124, 62, 30 };
    int i;

    for (i = 0; i < (sizeof(med_meas1_mock) / sizeof(med_meas1_mock[0])); i++)
    {
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_medb_mock, 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_medb_mock);

        mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &med_meas1_mock[i], 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&med_meas1_mock[i]);

        mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &med_meas2_mock[i], 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&med_meas2_mock[i]);

        TEST_ASSERT_EQUAL_INT32(med_waiting_time[i], mlx90632_calculate_dataset_ready_time());
    }
}

void test_calculate_dataset_ready_time_extended_success(void)
{
    uint16_t reg_ctrl_extb_mock = 0x0112; // extended sleeping step meas selected
    uint16_t ext_meas1_mock[] = { 0x8000, 0x8100, 0x8200, 0x8300, 0x8400, 0x8500, 0x8600, 0x8700 };
    uint16_t ext_meas2_mock[] = { 0x8012, 0x8112, 0x8212, 0x8312, 0x8412, 0x8512, 0x8612, 0x8712 };
    uint16_t ext_meas3_mock[] = { 0x800C, 0x810C, 0x820C, 0x830C, 0x840C, 0x850C, 0x860C, 0x870C };
    int ext_waiting_time[] = { 6000, 3000, 1500, 750, 375, 186, 93, 45 };
    int i;

    for (i = 0; i < (sizeof(ext_meas1_mock) / sizeof(ext_meas1_mock[0])); i++)
    {
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_extb_mock, 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_extb_mock);

        mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS1, &ext_meas1_mock[i], 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&ext_meas1_mock[i]);

        mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS2, &ext_meas2_mock[i], 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&ext_meas2_mock[i]);

        mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS3, &ext_meas3_mock[i], 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&ext_meas3_mock[i]);

        TEST_ASSERT_EQUAL_INT32(ext_waiting_time[i], mlx90632_calculate_dataset_ready_time());
    }
}

void test_calculate_dataset_ready_time_medical_errors(void)
{
    uint16_t reg_ctrl_medb_mock = 0x0002; // medical sleeping step meas selected
    uint16_t reg_ctrl_med_mock = 0x0006; // medical continuous meas selected
    uint16_t med_meas1_mock = 0x820D;
    uint16_t med_meas2_mock = 0x821D;

    //get_meas_type error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_medb_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());

    //invalid meas type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_med_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_med_mock);

    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_calculate_dataset_ready_time());

    //medical meas 1 error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_medb_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_medb_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &med_meas1_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());

    //medical meas 2 error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_medb_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_medb_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS1, &med_meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&med_meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEDICAL_MEAS2, &med_meas2_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());
}

void test_calculate_dataset_ready_time_extended_errors(void)
{
    uint16_t reg_ctrl_extb_mock = 0x0112; // extended sleeping step meas selected
    uint16_t reg_ctrl_ext_mock = 0x0116; // extended continuous meas selected
    uint16_t ext_meas1_mock = 0x8200;
    uint16_t ext_meas2_mock = 0x8212;
    uint16_t ext_meas3_mock = 0x820C;

    //get_meas_type error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_extb_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());

    //invalid meas type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_ext_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_ext_mock);

    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_calculate_dataset_ready_time());

    //extended meas 1 error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_extb_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_extb_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS1, &ext_meas1_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());

    //extended meas 2 error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_extb_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_extb_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS1, &ext_meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&ext_meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS2, &ext_meas2_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());

    //extended meas 3 error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_extb_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_extb_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS1, &ext_meas1_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&ext_meas1_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS2, &ext_meas2_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&ext_meas2_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_EXTENDED_MEAS3, &ext_meas3_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_calculate_dataset_ready_time());
}

/** Test get channel position.
 */
void test_get_channel_position_success(void)
{
    uint16_t reg_status_mock_med1 = 0x0087; // cycle position 1 & data ready
    uint16_t reg_status_mock_med2 = 0x008B; // cycle position 2 & data ready
    uint16_t reg_status_mock_ext1 = 0x00C5; // cycle position 17 & data ready
    uint16_t reg_status_mock_ext2 = 0x00C9; // cycle position 18 & data ready
    uint16_t reg_status_mock_ext3 = 0x00CF; // cycle position 19 & data ready

    // Read medical measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock_med1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock_med1);

    TEST_ASSERT_EQUAL_INT32(1, mlx90632_get_channel_position());

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock_med2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock_med2);

    TEST_ASSERT_EQUAL_INT32(2, mlx90632_get_channel_position());

    // Read extended measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock_ext1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock_ext1);

    TEST_ASSERT_EQUAL_INT32(17, mlx90632_get_channel_position());

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock_ext2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock_ext2);

    TEST_ASSERT_EQUAL_INT32(18, mlx90632_get_channel_position());

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock_ext3, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock_ext3);

    TEST_ASSERT_EQUAL_INT32(19, mlx90632_get_channel_position());
}

/** Test failure path when getting channel position.
 */
void test_get_channel_position_error(void)
{
    uint16_t reg_status_mock = 0x0087; // cycle position 1 & data ready

    // Get channel position expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_get_channel_position());
}

void test_set_meas_type_success(void)
{
    uint16_t reg_ctrl_mock_med = 0xFE0F;
    uint16_t reg_ctrl_mock_med1 = 0xFE09;
    uint16_t reg_ctrl_mock_med2 = 0xFE0B;
    uint16_t reg_ctrl_mock_med3 = 0xFE0D;
    uint16_t reg_ctrl_mock_ext = 0xFF1F;
    uint16_t reg_ctrl_mock_ext1 = 0xFF19;
    uint16_t reg_ctrl_mock_ext2 = 0xFF1B;
    uint16_t reg_ctrl_mock_ext3 = 0xFF1D;

    // Switch from medical to extended measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med3, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext1, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_ext1);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext, 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // Switch from extended to medical measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_ext);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext3, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_ext);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med1, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med1);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_set_meas_type(MLX90632_MTYP_MEDICAL));

    // Switch from medical to sleeping step medical measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med3, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med1, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med1);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med2, 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_set_meas_type(MLX90632_MTYP_MEDICAL_BURST));

    // Switch from medical sleeping step to extended sleeping step measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med2);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med3, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med2, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med2, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med2);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext1, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_ext1);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext2, 0);

    TEST_ASSERT_EQUAL_INT32(0, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED_BURST));
}

void test_set_meas_type_errors(void)
{
    uint16_t reg_ctrl_mock_med = 0xFE0F;
    uint16_t reg_ctrl_mock_ext1 = 0xFF19;
    uint16_t reg_ctrl_mock_step = 0xFE0D;

    // Invalid input parameter
    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_set_meas_type(9));

    // Addressed reset read current reg value error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // Addressed reset write step value error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_step, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // Addressed reset command error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_step, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // Addressed reset restore value error
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_step, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // First read fail
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_step, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // First write fail
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_step, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext1, -EPERM);

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));

    // Second read fail
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_step, 0);

    mlx90632_i2c_write_ExpectAndReturn(0x3005, MLX90632_RESET_CMD, 0);
    usleep_Expect(150, 200);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_med, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_CTRL, reg_ctrl_mock_ext1, 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext1, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_set_meas_type(MLX90632_MTYP_EXTENDED));
}

void test_get_meas_type_success(void)
{
    uint16_t reg_ctrl_mock_med = 0xFE0F;
    uint16_t reg_ctrl_mock_ext = 0xFF1F;
    uint16_t reg_ctrl_mock_med_burst = 0xFE02;
    uint16_t reg_ctrl_mock_ext_burst = 0xFF12;

    // Read medical measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med);

    TEST_ASSERT_EQUAL_INT32(MLX90632_MTYP_MEDICAL, mlx90632_get_meas_type());

    // Read extended measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_ext);

    TEST_ASSERT_EQUAL_INT32(MLX90632_MTYP_EXTENDED, mlx90632_get_meas_type());

    // Read medical sleeping step measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_med_burst, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_med_burst);

    TEST_ASSERT_EQUAL_INT32(MLX90632_MTYP_MEDICAL_BURST, mlx90632_get_meas_type());

    // Read extended measurement type
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_ext_burst, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_ext_burst);

    TEST_ASSERT_EQUAL_INT32(MLX90632_MTYP_EXTENDED_BURST, mlx90632_get_meas_type());
}

void test_get_meas_type_errors(void)
{
    uint16_t reg_ctrl_mock_inval = 0xFE9F;
    uint16_t reg_ctrl_mock_inval1 = 0xFE04;

    // Error reading the register
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_inval, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT32(-EPERM, mlx90632_get_meas_type());

    // Invalid measurement type data (measurement type)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_inval, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_inval);

    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_get_meas_type());

    // Invalid measurement type data (operating mode)
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_CTRL, &reg_ctrl_mock_inval1, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_ctrl_mock_inval1);

    TEST_ASSERT_EQUAL_INT32(-EINVAL, mlx90632_get_meas_type());
}

///@}
