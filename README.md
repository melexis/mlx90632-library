[![Build Status](https://travis-ci.org/melexis/mlx90632-library.svg?branch=master)](https://travis-ci.org/melexis/mlx90632-library)
[![Coverage Status](https://coveralls.io/repos/github/melexis/mlx90632-library/badge.svg?branch=master)](https://coveralls.io/github/melexis/mlx90632-library?branch=master)
[![Documentation](https://img.shields.io/badge/Documentation-published-brightgreen.svg)](https://melexis.github.io/mlx90632-library/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](http://www.apache.org/licenses/LICENSE-2.0)
[![Contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](https://github.com/melexis/mlx90632-library/issues)

This is 90632 example driver with virtual i2c read/write functions. There will
be some mapping needed with MCU's own i2c read/write procedures, but the core
calculations should remain the same. Functions that need to be implemented for
each individual MCU are listed in `mlx90632_depends.h` file.

Since there is one source and two header files they can be built also just as
normal source files. They are dependent on mathlib and errno so appropriate flags are
required. For ease of development and unit-testing Makefile was added with
following targets:

```
# All targets can take `CC=clang` variable, otherwise default compiler is GCC. If
# you need to cross-compile just feed `CROSS_COMPILE` variable to Makefile

make libs	# builds library with single file. Include inc/ for header definitions
make doxy	# builds doxygen documentation in build/html/
make utest	# builds and runs unit test program mlx90632 (dependent on ceedling)
make all	# builds unit tests, doxygen documentation, coverage information and library
make coverage   # builds coverage information
make clean	# cleans the crap make has made
```
# Documentation
Compiled documentation is available on [melexis.github.io/mlx90632-library](https://melexis.github.io/mlx90632-library/).
Datasheet is available in [Melexis documentation](https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90632).

# Example program flow
You can either include library directly or object file. Definitions are found
in library `inc/` folder and you need to point your compiler `-I` flag there.

After you have your environment set you need to enter below flow to your program.

```C
/* Before include, make sure you have BITS_PER_LONG defined. This is a CPU
 * specific value which is used to generate bit masks. You can also use -D
 * to input definition to compiler via command line
 */
#include "mlx90632.h"

/* Declare and implement here functions you find in mlx90632_depends.h */

/* You can use global or local storage for EEPROM register values so declare
 * them whereever you want. Do not forget to declare ambient_new_raw,
 * ambient_old_raw, object_new_raw, object_old_raw
 */
int main(void)
{
    int32_t ret = 0; /**< Variable will store return values */
    double ambient; /**< Ambient temperature in degrees Celsius */
    double object; /**< Object temperature in degrees Celsius */

    /* Read sensor EEPROM registers needed for calcualtions */

    /* Now we read current ambient and object temperature */
    ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw,
                                 &object_new_raw, &object_old_raw);
    if(ret < 0)
        /* Something went wrong - abort */
        return ret;

    /* Now start calculations (no more i2c accesses) */
    /* Calculate ambient temperature */
    ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw,
                                         P_T, P_R, P_G, P_O, Gb);

    /* Get preprocessed temperatures needed for object temperature calculation */
    double pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw,
                                                          ambient_old_raw, Gb);
    double pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,
                                                        ambient_new_raw, ambient_old_raw,
                                                        Ka);
    /* Calculate object temperature */
    object = mlx90632_calc_temp_object(object, ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);
}
```

# Dependencies for library unit-testing
Because of increased functionality and code size unit test, mocking and building
framework [Ceedling](http://www.throwtheswitch.org/ceedling/) was picked to ease
and validate development. It is an established open-source framework that brings
in additional dependency to ruby and rake (see `.gitlab-ci` file), but it allowed
faster development with automatic mocking ([CMock](http://www.throwtheswitch.org/cmock/))
and wider range of unit test macros ([Unity](http://www.throwtheswitch.org/unity/)).
Because of it, cloning repository requires adding a `--recursive` flag
(so `git clone --recursive <url> <destination>`) or initialization of submodules
afterwards using `git submodule update --init --recursive`. 


