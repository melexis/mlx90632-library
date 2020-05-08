/**
 * @file
 * @brief Unit tests for initialization with virtual i2c communication from sensor
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

void SetUp(void)
{
}

/** Test initialization process */
void test_init_success(void)
{
    uint16_t eeprom_version_mock = 0x105;
    uint16_t reg_status_mock = 0x47; // cycle position 1 & data ready

    // Confirm EEPROM version
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    // Read REG_STATUS
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Reset EOC and NewData
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & ~0x01, 0);

    TEST_ASSERT_EQUAL_INT(0, mlx90632_init());

    // test also ID_CONSUMER
    eeprom_version_mock = 0x205;

    // Confirm EEPROM version
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    // Read REG_STATUS
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Reset EOC and NewData
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & ~0x01, 0);

    TEST_ASSERT_EQUAL_INT(0, mlx90632_init());

    // test also another calibration id
    eeprom_version_mock = 0x305;

    // Confirm EEPROM version
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    // Read REG_STATUS
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Reset EOC and NewData
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & ~0x01, 0);

    TEST_ASSERT_EQUAL_INT(0, mlx90632_init());

    // test extended range
    eeprom_version_mock = 0x505;

    // Confirm EEPROM version
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    // Read REG_STATUS
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    // Reset EOC and NewData
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & ~0x01, 0);

    TEST_ASSERT_EQUAL_INT(ERANGE, mlx90632_init());
}

void test_init_wrong_eeprom_version(void)
{
    uint16_t eeprom_version_mock = 0x103;

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    TEST_ASSERT_EQUAL_INT(-EPROTONOSUPPORT, mlx90632_init());
}

void test_init_i2c_read_fails(void)
{
    uint16_t eeprom_version_mock = 0x103;

    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, -EBUSY);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output

    TEST_ASSERT_EQUAL_INT(-EBUSY, mlx90632_init());
}

void test_init_i2c_read_fails2(void)
{
    uint16_t eeprom_version_mock = 0x105;
    uint16_t reg_status_mock = 0x47; // cycle position 1 & data ready

    // Confirm EEPROM version
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    // Read REG_STATUS
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);

    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_init());
}

void test_init_i2c_read_fails3(void)
{
    uint16_t eeprom_version_mock = 0x105;
    uint16_t reg_status_mock = 0x47; // cycle position 1 & data ready

    // Confirm EEPROM version
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_VERSION, &eeprom_version_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&eeprom_version_mock);

    // Read REG_STATUS
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, &reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_status_mock);


    // Reset NewData
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock & ~0x01, -EPERM);

    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_init());
}

///@}

