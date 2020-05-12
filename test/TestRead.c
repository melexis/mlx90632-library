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

void SetUp(void)
{
    ambient_new_raw = 0;
    ambient_old_raw = 0;
    object_new_raw = 0;
    object_old_raw = 0;
}

/** Test start measurement with data ready.
 */
void test_start_measurement_success(void)
{
    uint16_t reg_status_mock = 0x87; // cycle position 1 & data ready

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT(1, mlx90632_start_measurement());
}

/** Test start measurement function, if data is ready and if it is not it waits one usleep period before retry.
 *
 * First we simulate data not ready with bit0 not set. After one period we flip the bit0 to 1 to indicate
 * data ready and start measurement should complete with success.
 */
void test_start_measurement_one_wait(void)
{
    uint16_t reg_status_mock = 0x86; // cycle position 1 & data NOT READY
    uint16_t reg_status_mock1 = 0x87; // cycle position 1 & data READY

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

    TEST_ASSERT_EQUAL_INT(1, mlx90632_start_measurement());
}

/** Test different failure paths in start_measurement */
void test_start_measurement_busy_i2c(void)
{
    uint16_t reg_status_mock = 0x06;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_start_measurement());

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), -EPERM);

    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_start_measurement());

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & (~MLX90632_STAT_DATA_RDY), 0);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_start_measurement());
}

/** Test sensor timeouts while start_measure.
 *
 * If this happens in real life it means that after 100 tries sensor still did not indicate data ready, which probably
 * points to much larger problem than a simple timeout. Timeout is only valid if usleep is a lot shorter than default
 * values
 */
void test_start_measurement_timeout(void)
{
    uint16_t reg_status_first_mock = 0x06;
    uint16_t reg_status_mock[100];
    int i;

    for (i = 0; i < 100; ++i)
        reg_status_mock[i] = 0x0006; // cycle position 1 & data NOT READY through whole array!

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_first_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_first_mock);

    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_first_mock & (~MLX90632_STAT_DATA_RDY), 0);

    for (i = 0; i < 100; ++i)
    {
        mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock[i], 0);
        mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
        mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock[i]);

        usleep_Expect(10000, 11000);
    }

    TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, mlx90632_start_measurement());
}


/** Test whole start and read temperature from sensor procedure.
 *
 * Start measurement which returns channel 1 as new, then read ambient and object raw values.
 * Channel 2 values are read as old, while channel 1 values are read as new. Start measurement is triggered
 * and channel selection is done based on the register values in start measurement.
 */
void test_read_temp_raw_ch1_success(void)
{
    uint16_t reg_status_mock = 0x87; // cycle position 1 & data ready
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
    TEST_ASSERT_EQUAL_INT(0, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_mock);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_mock);
}

/** Test whole start and read temperature from sensor procedure.
 *
 * Start measurement which returns channel 2 as new, then read ambient and object raw values.
 * Channel 1 values are read as old, while channel 2 values are read as new. Start measurement is triggered
 * and channel selection is done based on the register values in start measurement.
 */
void test_read_temp_raw_ch2_success(void)
{
    uint16_t reg_status_mock = 0x8b; // cycle position 2 & data ready
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
    TEST_ASSERT_EQUAL_INT(0, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

    // Confirm all values are as expected
    TEST_ASSERT_EQUAL_INT16(ambient_new_mock, ambient_new_raw);
    TEST_ASSERT_EQUAL_INT16(ambient_old_mock, ambient_old_raw);
    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_mock);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_mock);
}

/** Test Error output on whole start and read temperature from sensor procedure. */
void test_read_temp_raw_errors(void)
{
    uint16_t reg_status_mock = 0x87; // cycle position 1 & data ready
    int16_t ambient_new_mock = 22454;

    // Start measurement expectations
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));

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
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw));
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
    TEST_ASSERT_EQUAL_INT(0, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));

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
    TEST_ASSERT_EQUAL_INT(0, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));

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
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));

    // Second read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(1), (uint16_t*)&ambient_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&ambient_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_3(2), (uint16_t*)&ambient_old_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_ambient_raw(&ambient_new_raw, &ambient_old_raw));
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
    TEST_ASSERT_EQUAL_INT(0, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_mock);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_mock);
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
    TEST_ASSERT_EQUAL_INT(0, mlx90632_read_temp_object_raw(2, &object_new_raw, &object_old_raw));

    TEST_ASSERT_EQUAL_INT16(object_new_mock, object_new_mock);
    TEST_ASSERT_EQUAL_INT16(object_old_mock, object_old_mock);
}

/** Test error outputs when reading object values. */
void test_read_object_values_errors(void)
{
    int16_t object_new_mock = 150;
    int16_t object_old_mock = 150;

    // First read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    // Second read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

    // Third read fails
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_1(1), (uint16_t*)&object_new_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value((uint16_t*)&object_new_mock);

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_RAM_2(2), (uint16_t*)&object_old_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));

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
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_read_temp_object_raw(1, &object_new_raw, &object_old_raw));
}

/** Test error outputs when reading object values. */
void test_read_object_error_ch(void)
{
    // Input retval is invalid
    TEST_ASSERT_EQUAL_INT(-EINVAL, mlx90632_read_temp_object_raw(3, &object_new_raw, &object_old_raw));
}

///@}

