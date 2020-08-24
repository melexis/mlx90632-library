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

void setUp(void)
{
}

void tearDown(void)
{
}

void expect_read_meas1_success ()
{
    static uint16_t reg_meas1_mock = 0x820D;
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEAS_1, &reg_meas1_mock, 0);
	mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_meas1_mock);
}

void expect_read_meas2_success ()
{
    static uint16_t reg_meas2_mock = 0x821D;
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEAS_2, &reg_meas2_mock, 0);
	mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_meas2_mock);
}

void expect_unlockEEPROM_success ()
{
    mlx90632_i2c_write_ExpectAndReturn(0x3005, 0x554C, 0);
}

void expect_write_meas1_success (uint16_t data)
{
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_EE_MEAS_1, data, 0);
}

void expect_write_meas2_success (uint16_t data)
{
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_EE_MEAS_2, data, 0);
}

void expect_read_status_success (uint16_t* reg_status_mock)
{
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock, 0);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(reg_status_mock);
}

void expect_read_status_success_eepromBusy ()
{
	static uint16_t readBusy = 0x0200;
	expect_read_status_success (&readBusy);
}

void expect_read_status_success_eepromNotBusy ()
{
	static uint16_t readNotBusy = 0xFDFF;
	expect_read_status_success (&readNotBusy);
}
	
void assertSetRefreshRate (MLX90632_MEAS meas, uint16_t reg_meas1, uint16_t reg_meas2)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (reg_meas1);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_success ();
	expect_write_meas2_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas2_success (reg_meas2);
	expect_read_status_success_eepromNotBusy ();
	
    TEST_ASSERT_EQUAL_INT(0, mlx90632_set_refreshRate(meas));
}

void test_set_refresh_rate_success(void)
{
	assertSetRefreshRate(MLX90632_MEAS_HZ_HALF, 0x800D, 0x801D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_1, 0x810D, 0x811D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_2, 0x820D, 0x821D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_4, 0x830D, 0x831D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_8, 0x840D, 0x841D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_16, 0x850D, 0x851D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_32, 0x860D, 0x861D);
	assertSetRefreshRate(MLX90632_MEAS_HZ_64, 0x870D, 0x871D);
}

void expect_read_meas1_error ()
{
    static uint16_t reg_meas1_mock = 0x820D;
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEAS_1, &reg_meas1_mock,  -EPERM);
	mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_meas1_mock);
}

void expect_unlockEEPROM_error ()
{
    mlx90632_i2c_write_ExpectAndReturn(0x3005, 0x554C, -EPERM);
}

void expect_write_meas1_error (uint16_t data)
{
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_EE_MEAS_1, data, -EPERM);
}

void expect_read_status_error (uint16_t* reg_status_mock)
{
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_REG_STATUS, reg_status_mock, -EPERM);
    mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(reg_status_mock);
}

void test_set_refresh_rate_error_first_read_fails(void)
{
	expect_read_meas1_error ();
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_first_unlock_erase_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_error ();
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_first_erase_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_error (0x00);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_first_read_erase_status_fails(void)
{
	uint16_t error = 0x00;
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_error (&error);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_first_unlock_write_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_error ();
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_first_write_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_error (0x870D);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_first_read_write_status_fails(void)
{
	uint16_t error = 0x00;
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_error (&error);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void expect_read_meas2_error ()
{
    static uint16_t reg_meas2_mock = 0x821D;
    mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEAS_2, &reg_meas2_mock,  -EPERM);
	mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_meas2_mock);
}

void expect_write_meas2_error (uint16_t data)
{
    mlx90632_i2c_write_ExpectAndReturn(MLX90632_EE_MEAS_2, data, -EPERM);
}

void test_set_refresh_rate_error_second_read_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_error ();
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_second_unlock_erase_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_error ();
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_second_erase_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_success ();
	expect_write_meas2_error(0x00);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_second_read_erase_status_fails(void)
{
	uint16_t error = 0x00;
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_success ();
	expect_write_meas2_success(0x00);
	expect_read_status_error (&error);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_second_unlock_write_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_success ();
	expect_write_meas2_success(0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_error();
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_second_write_fails(void)
{
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_success ();
	expect_write_meas2_success(0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success();
	expect_write_meas2_error (0x871D);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void test_set_refresh_rate_error_second_read_write_status_fails(void)
{
	uint16_t error = 0x00;
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x870D);
	expect_read_status_success_eepromNotBusy ();
	
	expect_read_meas2_success();
	expect_unlockEEPROM_success ();
	expect_write_meas2_success(0x00);
	expect_read_status_success_eepromNotBusy ();
	expect_unlockEEPROM_success();
	expect_write_meas2_success (0x871D);
	expect_read_status_error(&error);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}

void assertGetRefreshRate (MLX90632_MEAS meas, uint16_t reg_meas1_mock)
{
	mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEAS_1, &reg_meas1_mock, 0);
	mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_meas1_mock);
	
	TEST_ASSERT_EQUAL_INT(meas, mlx90632_get_refreshRate());
}

void test_get_refresh_rate(void)
{
	assertGetRefreshRate (MLX90632_MEAS_HZ_HALF, 0x800D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_1, 0x810D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_2, 0x820D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_4, 0x830D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_8, 0x840D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_16, 0x850D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_32, 0x860D);
	assertGetRefreshRate (MLX90632_MEAS_HZ_64, 0x870D);
}

void test_get_refresh_rate_error(void)
{
	uint16_t reg_meas1_mock = 0x800D;
	mlx90632_i2c_read_ExpectAndReturn(MLX90632_EE_MEAS_1, &reg_meas1_mock, -EPERM);
	mlx90632_i2c_read_IgnoreArg_value(); // Ignore input of mock since we use it as output
    mlx90632_i2c_read_ReturnThruPtr_value(&reg_meas1_mock);
	
	TEST_ASSERT_EQUAL_INT(MLX90632_MEAS_HZ_ERROR, mlx90632_get_refreshRate());
}

void test_set_refresh_rate_success_status_busy_error(void)
{
	uint16_t error = 0x00;
	expect_read_meas1_success();
	expect_unlockEEPROM_success ();
	expect_write_meas1_success (0x00);
	expect_read_status_success_eepromBusy ();
	expect_read_status_error (&error);
	
    TEST_ASSERT_EQUAL_INT(-EPERM, mlx90632_set_refreshRate(MLX90632_MEAS_HZ_64));
}
///@}

