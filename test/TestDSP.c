/**
 * @file
 * @brief MLX90632 driver with virtual i2c communication
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

#include "mlx90632.h"

#include "mock_mlx90632_depends.h"

// example P_R values
int32_t P_R = 0x00587f5b;
int32_t P_G = 0x04a10289;
int32_t P_T = 0xfff966f8;
int32_t P_O = 0x00001e0f;
int32_t Ea = 4859535;
int32_t Eb = 5686508;
int32_t Fa = 53855361;
int32_t Fb = 42874149;
int32_t Ga = -14556410;
int16_t Ha = 16384;
int16_t Hb = 0;
int16_t Gb = 9728;
int16_t Ka = 10752;

double dspv5_object_helper(int16_t object_new_raw, int16_t object_old_raw, int16_t ambient_new_raw, int16_t ambient_old_raw)
{
    double object, ambient;

    ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
    object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw,
                                             Ka);
    return mlx90632_calc_temp_object(object, ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
}

double dspv5_object_reflected_helper(int16_t object_new_raw, int16_t object_old_raw, int16_t ambient_new_raw, int16_t ambient_old_raw, double reflected)
{
    double object, ambient;

    ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
    object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw,
                                             Ka);
    return mlx90632_calc_temp_object_reflected(object, ambient, reflected, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
}

double dspv5_ambient_helper(int16_t ambient_new_raw, int16_t ambient_old_raw)
{
    return mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, P_T, P_R, P_G, P_O, Gb);
}

void setUp(void)
{
    // Global variable - set it back to starting point
    mlx90632_set_emissivity(1.0);
}

void test_dsp_preprocess_ambient(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 24041.27, mlx90632_preprocess_temp_ambient(22454, 23030, Gb));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 19065.018, mlx90632_preprocess_temp_ambient(100, 150, Gb));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 24385.9, mlx90632_preprocess_temp_ambient(32767, 32766, Gb));
}

void test_dsp_preprocess_object(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 3314.89, mlx90632_preprocess_temp_object(3237, 3239, 22454, 23030, Ka));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 153.562, mlx90632_preprocess_temp_object(149, 151, 22454, 23030, Ka));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -153.562, mlx90632_preprocess_temp_object(-149, -151, 22454, 23030, Ka));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -33545.08, mlx90632_preprocess_temp_object(-32767, -32767, 22454, 23030, Ka));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 33545.08, mlx90632_preprocess_temp_object(32767, 32767, 22454, 23030, Ka));
}


void test_dsp_ambient(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 48.724, dspv5_ambient_helper(22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -18.734, dspv5_ambient_helper(100, 150));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 53.350, dspv5_ambient_helper(32767, 32766));
}

void test_dsp_object(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 55.507, dspv5_object_helper(609, 611, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 51.123, dspv5_object_helper(149, 151, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 48.171, dspv5_object_helper(-149, -151, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 212.844, dspv5_object_helper(32767, 32767, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -16.653, dspv5_object_helper(-5000, -5000, 22454, 23030));
}

void test_dsp_object_reflected(void)
{
    double ke = mlx90632_get_emissivity();

    mlx90632_set_emissivity(1.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 55.507, dspv5_object_reflected_helper(609, 611, 22454, 23030, 40.00));
    mlx90632_set_emissivity(0.1);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 98.141, dspv5_object_reflected_helper(609, 611, 22454, 23030, 49.66));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 143.956, dspv5_object_reflected_helper(609, 611, 22454, 23030, 40.00));

    mlx90632_set_emissivity(ke);
}

void test_set_get_emissivity(void)
{
    double delta = 0.00001;

    mlx90632_set_emissivity(1.0);
    TEST_ASSERT_DOUBLE_WITHIN(delta, 1.0, mlx90632_get_emissivity());

    mlx90632_set_emissivity(0.8);
    TEST_ASSERT_DOUBLE_WITHIN(delta, 0.8, mlx90632_get_emissivity());

    mlx90632_set_emissivity(0.1);
    TEST_ASSERT_DOUBLE_WITHIN(delta, 0.1, mlx90632_get_emissivity());

    mlx90632_set_emissivity(0.0);
    TEST_ASSERT_DOUBLE_WITHIN(delta, 1.0, mlx90632_get_emissivity());
}

void test_dsp_object_close(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 48.171, dspv5_object_helper(-149, -151, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 48.192, dspv5_object_helper(-147, -149, 22454, 23030));
}


void test_dsp_object_Hb_change(void)
{
    int16_t tmp_Hb = Hb;

    Hb = 10240;
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 45.495, dspv5_object_helper(609, 611, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 41.121, dspv5_object_helper(149, 151, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 38.174, dspv5_object_helper(-149, -151, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 202.628, dspv5_object_helper(32767, 32767, 22454, 23030));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -26.457, dspv5_object_helper(-5000, -5000, 22454, 23030));
    Hb = tmp_Hb;
}


///@}

