/**
 * @file mlx90632.c
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
 * @details
 *
 * @addtogroup mlx90632_private MLX90632 Internal library functions
 * @{
 *
 */
#include <stdint.h>
#include <math.h>
#include <errno.h>

#include "mlx90632.h"
#include "mlx90632_depends.h"

#define POW10 10000000000LL

#ifndef VERSION
#define VERSION "test"
#endif

static const char mlx90632version[] __attribute__((used)) = { VERSION };

#ifndef STATIC
#define STATIC static
#endif

/** Trigger start measurement for mlx90632
 *
 * Trigger measurement cycle and wait for data to be ready. It does not read anything, just triggers and completes.
 *
 * @retval <0 Something failed. Check errno.h for more information
 * @retval >=0 Channel position where new (recently updated) measurement can be found
 *
 * @note This function is using usleep so it is blocking!
 */
STATIC int mlx90632_start_measurement(void)
{
    int ret, tries = 100;
    uint16_t reg_status;

    ret = mlx90632_i2c_read(MLX90632_REG_STATUS, &reg_status);
    if (ret < 0)
        return ret;

    ret = mlx90632_i2c_write(MLX90632_REG_STATUS, reg_status & (~MLX90632_STAT_DATA_RDY));
    if (ret < 0)
        return ret;

    while (tries-- > 0)
    {
        ret = mlx90632_i2c_read(MLX90632_REG_STATUS, &reg_status);
        if (ret < 0)
            return ret;
        if (reg_status & MLX90632_STAT_DATA_RDY)
            break;
        /* minimum wait time to complete measurement
         * should be calculated according to refresh rate
         * atm 10ms - 11ms
         */
        usleep(10000, 11000);
    }

    if (tries < 0)
    {
        // data not ready
        return -ETIMEDOUT;
    }

    return (reg_status & MLX90632_STAT_CYCLE_POS) >> 2;
}

/** Based on @link mlx90632_start_measurement @endlink return value fill channel_new and channel_old
 * variables with proper values. This is to provide a bit more flexibility in case other channels are
 * returned and need a bit more mingeling than usual maths can provide.
 *
 * So far there are just two use cases. If returned value is not 1 or 2, it will leave channel_new
 * and channel_old unassigned.
 *
 * @param[in] ret @link mlx90632_start_measurement @endlink return value
 * @param[out] *channel_new Pointer to memory location where new channel value is stored
 * @param[out] *channel_old Pointer to memory location where old channel value is stored
 *
 * @retval 0 When both memory locations are updated as per ret
 * @retval -EINVAL channel_new and channel_old were not updated
 */
static int32_t mlx90632_channel_new_select(int32_t ret, uint8_t *channel_new, uint8_t *channel_old)
{
    switch (ret)
    {
    case 1:
        *channel_new = 1;
        *channel_old = 2;
        break;

    case 2:
        *channel_new = 2;
        *channel_old = 1;
        break;

    default:
        return -EINVAL;
    }
    return 0;
}

/** Read ambient raw old and new values based on @link mlx90632_start_measurement @endlink return value.
 *
 * Two i2c_reads are needed to obtain necessary raw ambient values from the sensor, as they are then
 * preprocessed before going to calculation functions. Because one value is newer than other (see @link
 * mlx90632_start_measurement @endlink) this function provides dynamics based on return value of the
 * @link mlx90632_start_measurement @endlink.
 *
 * @param[out] *ambient_new_raw Pointer to memory location where new ambient value from sensor is stored
 * @param[out] *ambient_old_raw Pointer to memory location where old ambient value from sensor is stored
 *
 * @retval 0 Successfully read both values
 * @retval <0 Something went wrong. Check errno.h for more details.
 */
STATIC int32_t mlx90632_read_temp_ambient_raw(int16_t *ambient_new_raw, int16_t *ambient_old_raw)
{
    int32_t ret;
    uint16_t read_tmp;

    ret = mlx90632_i2c_read(MLX90632_RAM_3(1), &read_tmp);
    if (ret < 0)
        return ret;
    *ambient_new_raw = (int16_t)read_tmp;

    ret = mlx90632_i2c_read(MLX90632_RAM_3(2), &read_tmp);
    if (ret < 0)
        return ret;
    *ambient_old_raw = (int16_t)read_tmp;

    return ret;
}

/** Read object raw old and new values based on @link mlx90632_start_measurement @endlink return value.
 *
 * Four i2c_reads are needed to obtain necessary raw object values from the sensor. These values are grouped per new
 * and old and then averaged before return of the function. After that they are then preprocessed before going to
 * calculation functions. Because one value is newer than other (see @link mlx90632_start_measurement @endlink) this
 * function provides dynamics based on return value of the @link mlx90632_start_measurement @endlink.
 *
 * @param[in] start_measurement_ret Return value of @link mlx90632_start_measurement @endlink
 * @param[out] *object_new_raw Pointer to memory location where average of new object values from sensor is stored
 * @param[out] *object_old_raw Pointer to memory location where average of old object values from sensor is stored
 *
 * @retval 0 Successfully read both values
 * @retval <0 Something went wrong. Check errno.h for more details.
 */
STATIC int32_t mlx90632_read_temp_object_raw(int32_t start_measurement_ret,
        int16_t *object_new_raw, int16_t *object_old_raw)
{
    int32_t ret;
    uint16_t read_tmp;
    int16_t read;
    uint8_t channel, channel_old;

    ret = mlx90632_channel_new_select(start_measurement_ret, &channel, &channel_old);
    if (ret != 0)
        return -EINVAL;

    ret = mlx90632_i2c_read(MLX90632_RAM_2(channel), &read_tmp);
    if (ret < 0)
        return ret;

    read = (int16_t)read_tmp;

    ret = mlx90632_i2c_read(MLX90632_RAM_1(channel), &read_tmp);
    if (ret < 0)
        return ret;
    *object_new_raw = (read + (int16_t)read_tmp) / 2;

    ret = mlx90632_i2c_read(MLX90632_RAM_2(channel_old), &read_tmp);
    if (ret < 0)
        return ret;
    read = (int16_t)read_tmp;

    ret = mlx90632_i2c_read(MLX90632_RAM_1(channel_old), &read_tmp);
    if (ret < 0)
        return ret;
    *object_old_raw = (read + (int16_t)read_tmp) / 2;

    return ret;
}

int32_t mlx90632_read_temp_raw(int16_t *ambient_new_raw, int16_t *ambient_old_raw,
                               int16_t *object_new_raw, int16_t *object_old_raw)
{
    int32_t ret, start_measurement_ret;

    // trigger and wait for measurement to complete
    start_measurement_ret = mlx90632_start_measurement();
    if (start_measurement_ret < 0)
        return start_measurement_ret;

    /** Read new and old **ambient** values from sensor */
    ret = mlx90632_read_temp_ambient_raw(ambient_new_raw, ambient_old_raw);
    if (ret < 0)
        return ret;

    /** Read new and old **object** values from sensor */
    ret = mlx90632_read_temp_object_raw(start_measurement_ret, object_new_raw, object_old_raw);

    return ret;
}

/* DSPv5 */
double mlx90632_preprocess_temp_ambient(int16_t ambient_new_raw, int16_t ambient_old_raw, int16_t Gb)
{
    double VR_Ta, kGb;

    kGb = ((double)Gb) / 1024.0;

    VR_Ta = ambient_old_raw + kGb * (ambient_new_raw / (MLX90632_REF_3));
    return ((ambient_new_raw / (MLX90632_REF_3)) / VR_Ta) * 524288.0;
}

double mlx90632_preprocess_temp_object(int16_t object_new_raw, int16_t object_old_raw,
                                       int16_t ambient_new_raw, int16_t ambient_old_raw,
                                       int16_t Ka)
{
    double VR_IR, kKa;

    kKa = ((double)Ka) / 1024.0;

    VR_IR = ambient_old_raw + kKa * (ambient_new_raw / (MLX90632_REF_3));
    return ((((object_new_raw + object_old_raw) / 2) / (MLX90632_REF_12)) / VR_IR) * 524288.0;
}

double mlx90632_calc_temp_ambient(int16_t ambient_new_raw, int16_t ambient_old_raw, int32_t P_T,
                                  int32_t P_R, int32_t P_G, int32_t P_O, int16_t Gb)
{
    double Asub, Bsub, Ablock, Bblock, Cblock, AMB;

    AMB = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);

    Asub = ((double)P_T) / (double)17592186044416.0;
    Bsub = (double)AMB - ((double)P_R / (double)256.0);
    Ablock = Asub * (Bsub * Bsub);
    Bblock = (Bsub / (double)P_G) * (double)1048576.0;
    Cblock = (double)P_O / (double)256.0;

    return Bblock + Ablock + Cblock;
}

/** Iterative calculation of object temperature
 *
 * DSPv5 requires 3 iterations to reduce noise for object temperature. Since
 * each iteration requires same calculations this helper function is
 * implemented.
 *
 * @param[in] prev_object_temp previously calculated object temperature. If
 *                              there is no previously calculated temperature
 *                              input 25.0
 * @param[in] object object temperature from @link mlx90632_preprocess_temp_object @endlink
 * @param[in] TAdut ambient temperature coefficient
 * @param[in] Ga Register value on @link MLX90632_EE_Ga @endlink
 * @param[in] Fa Register value on @link MLX90632_EE_Fa @endlink
 * @param[in] Fb Register value on @link MLX90632_EE_Fb @endlink
 * @param[in] Ha Register value on @link MLX90632_EE_Ha @endlink
 * @param[in] Hb Register value on @link MLX90632_EE_Hb @endlink
 * @param[in] emissivity Value provided by user of the object emissivity
 *
 * @return Calculated object temperature for current iteration in milliCelsius
 */
static double mlx90632_calc_temp_object_iteration(double prev_object_temp, int32_t object, double TAdut,
        int32_t Ga, int32_t Fa, int32_t Fb, int16_t Ha, int16_t Hb,
        double emissivity)
{
    double calcedGa, calcedGb, calcedFa, TAdut4, first_sqrt;
    // temp variables
    double KsTAtmp, Alpha_corr;
    double Ha_customer, Hb_customer;


    Ha_customer = Ha / ((double)16384.0);
    Hb_customer = Hb / ((double)1024.0);
    calcedGa = ((double)Ga * (prev_object_temp - 25)) / ((double)68719476736.0);
    KsTAtmp = (double)Fb * (TAdut - 25);
    calcedGb = KsTAtmp / ((double)68719476736.0);
    Alpha_corr = (((double)(Fa * POW10)) * Ha_customer * (double)(1 + calcedGa + calcedGb)) /
                 ((double)70368744177664.0);
    calcedFa = object / (emissivity * (Alpha_corr / POW10));
    TAdut4 = (TAdut + 273.15) * (TAdut + 273.15) * (TAdut + 273.15) * (TAdut + 273.15);

    first_sqrt = sqrt(calcedFa + TAdut4);

    return sqrt(first_sqrt) - 273.15 - Hb_customer;
}

/** Iterative calculation of object temperature  when the environment temperature differs from the sensor temperature
 *
 * DSPv5 requires 3 iterations to reduce noise for object temperature. Since
 * each iteration requires same calculations this helper function is
 * implemented.
 *
 * @param[in] prev_object_temp previously calculated object temperature. If
 *                              there is no previously calculated temperature
 *                              input 25.0
 * @param[in] object object temperature from @link mlx90632_preprocess_temp_object @endlink
 * @param[in] TAdut ambient temperature coefficient
 * @param[in] Tr reflected (environment) temperature from a sensor different than the MLX90632 or acquired by other means
 * @param[in] Ga Register value on @link MLX90632_EE_Ga @endlink
 * @param[in] Fa Register value on @link MLX90632_EE_Fa @endlink
 * @param[in] Fb Register value on @link MLX90632_EE_Fb @endlink
 * @param[in] Ha Register value on @link MLX90632_EE_Ha @endlink
 * @param[in] Hb Register value on @link MLX90632_EE_Hb @endlink
 * @param[in] emissivity Value provided by user of the object emissivity
 *
 * @return Calculated object temperature for current iteration in milliCelsius
 */
static double mlx90632_calc_temp_object_iteration_reflected(double prev_object_temp, int32_t object, double TAdut, double TaTr4,
        int32_t Ga, int32_t Fa, int32_t Fb, int16_t Ha, int16_t Hb,
        double emissivity)
{
    double calcedGa, calcedGb, calcedFa, TAdut4, first_sqrt;
    // temp variables
    double KsTAtmp, Alpha_corr;
    double Ha_customer, Hb_customer;

    Ha_customer = Ha / ((double)16384.0);
    Hb_customer = Hb / ((double)1024.0);
    calcedGa = ((double)Ga * (prev_object_temp - 25)) / ((double)68719476736.0);
    KsTAtmp = (double)Fb * (TAdut - 25);
    calcedGb = KsTAtmp / ((double)68719476736.0);
    Alpha_corr = (((double)(Fa * POW10)) * Ha_customer * (double)(1 + calcedGa + calcedGb)) /
                 ((double)70368744177664.0);
    calcedFa = object / (emissivity * (Alpha_corr / POW10));

    first_sqrt = sqrt(calcedFa + TaTr4);

    return sqrt(first_sqrt) - 273.15 - Hb_customer;
}

static double emissivity = 0.0;
void mlx90632_set_emissivity(double value)
{
    emissivity = value;
}

double mlx90632_get_emissivity(void)
{
    if (emissivity == 0.0)
    {
        return 1.0;
    }
    else
    {
        return emissivity;
    }
}

double mlx90632_calc_temp_object(int32_t object, int32_t ambient,
                                 int32_t Ea, int32_t Eb, int32_t Ga, int32_t Fa, int32_t Fb,
                                 int16_t Ha, int16_t Hb)
{
    double kEa, kEb, TAdut;
    double temp = 25.0;
    double tmp_emi = mlx90632_get_emissivity();
    int8_t i;

    kEa = ((double)Ea) / ((double)65536.0);
    kEb = ((double)Eb) / ((double)256.0);
    TAdut = (((double)ambient) - kEb) / kEa + 25;

    //iterate through calculations
    for (i = 0; i < 5; ++i)
    {
        temp = mlx90632_calc_temp_object_iteration(temp, object, TAdut, Ga, Fa, Fb, Ha, Hb, tmp_emi);
    }
    return temp;
}

double mlx90632_calc_temp_object_reflected(int32_t object, int32_t ambient, double reflected,
        int32_t Ea, int32_t Eb, int32_t Ga, int32_t Fa, int32_t Fb,
        int16_t Ha, int16_t Hb)
{
    double kEa, kEb, TAdut;
    double temp = 25.0;
    double tmp_emi = mlx90632_get_emissivity();
    double TaTr4;
    double ta4;
    int8_t i;

    kEa = ((double)Ea) / ((double)65536.0);
    kEb = ((double)Eb) / ((double)256.0);
    TAdut = (((double)ambient) - kEb) / kEa + 25;

    TaTr4 = reflected + 273.15;
    TaTr4 = TaTr4 * TaTr4;
    TaTr4 = TaTr4 * TaTr4;
    ta4 = TAdut + 273.15;
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    TaTr4 = TaTr4 - (TaTr4-ta4)/tmp_emi;

    //iterate through calculations
    for (i = 0; i < 5; ++i)
    {
        temp = mlx90632_calc_temp_object_iteration_reflected(temp, object, TAdut, TaTr4, Ga, Fa, Fb, Ha, Hb, tmp_emi);
    }
    return temp;
}

int32_t mlx90632_init(void)
{
    int32_t ret;
    uint16_t eeprom_version, reg_status;

    ret = mlx90632_i2c_read(MLX90632_EE_VERSION, &eeprom_version);
    if (ret < 0)
    {
        return ret;
    }

    if ((eeprom_version & 0x00FF) != MLX90632_DSPv5)
    {
        // this here can fail because of big/little endian of cpu/i2c
        return -EPROTONOSUPPORT;
    }

    ret = mlx90632_i2c_read(MLX90632_REG_STATUS, &reg_status);
    if (ret < 0)
        return ret;

    // Prepare a clean start with setting NEW_DATA to 0
    ret = mlx90632_i2c_write(MLX90632_REG_STATUS, reg_status & ~(MLX90632_STAT_DATA_RDY));
    if (ret < 0)
        return ret;

    return 0;
}
///@}
