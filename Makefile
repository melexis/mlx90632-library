# @copyright (C) 2017 Melexis N.V.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Extract current version from library git repo
VERSION = $(shell echo `git describe --always --long --abbrev=8 --dirty`)

# Include sources into compilation
SRCS +=	$(wildcard src/*.c)
UNIT_TESTS += $(wildcard test/*.c)
INCLUDE = -Iinc/
UNCRUSTIFY_FILES = $(SRCS) \
		   $(UNIT_TESTS) \
		   $(wildcard inc/*.h)

# From sources list include .h files for dependencies
DEPS +=	$(wildcard inc/*.h)

# generate object files in objdir
C_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(filter %.c, $(SRCS)))
UNIT_TEST_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(filter %.c, $(UNIT_TESTS)))

# we want same order of the object files passed to linker each
# time so sort them. This makes linking process independent on
# how object files are sorted on disk
OBJS = $(sort $(C_OBJS) $(UNIT_TEST_OBJS))

# Put tools on one spot just in case
CC := $(CROSS_COMPILE)gcc
AR := $(CORSS_COMPILE)ar
OBJDIR := build
TARGET = mlx90632

# Flags
CFLAGS += -Wall -Wpedantic     # just something for warnings...
CFLAGS += -Werror              # Make sure we dont have any warnings
ARFLAGS = rcs
# optimization levels
# -------------------
CFLAGS += -fmessage-length=0 -fno-builtin \
			-ffunction-sections -fdata-sections
CFLAGS += -std=c99 -fshort-enums

# Put dependencies in case we change header file we want to recompile
# (most bottom line does that with help of compiler creating .d files)
ifeq ($(CC),clang)
CFLAGS += -MD -MP -MT $@ -MF $(@:.o=.d)
else
CFLAGS += -Wp,-MM,-MP,-MT,$@,-MF,$(@:.o=.d)
endif

# Add version to source file
CFLAGS += '-DVERSION="VERSION=$(VERSION)"'
CFLAGS += -DSTATIC=static

# Include mathlib at linking
DLIB += -lm
#LFLAGS += -lm

# ==========================
# Test coverage (lcov) flags
# ==========================
LCOVFLAGS  = --no-external --directory src/ --capture
LCOVCONFIG = --config-file .lcovrc
ifeq ($(CC),clang)
LCOVFLAGS += --gcov-tool './llvm-gcov.sh'
endif

.PHONY: all
.PHONY: clean
.PHONY: libs
.PHONY: utest
.PHONY: doxy
.PHONY: coverage
.PHONY: cscope
.PHONY: ctags

all: utest libs coverage doxy

# build object files just for fun of it with dependencies on .h files
$(C_OBJS): $(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling $< -> $@"
	@$(CC) $(INCLUDE) -c $< -o $@ $(CFLAGS)
$(UNIT_TEST_OBJS): $(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "Compiling unit $< -> $@"
	@$(CC) $(INCLUDE) -c $< -o $@ $(CFLAGS)

# build stuff together and link it to .elf
$(TARGET): $(OBJS)
	@$(CC) $(LFLAGS) -o $@ $^ $(DLIB)
	@echo "Linked everything to $@"

libs: lib$(TARGET).a

# build stuff together as library
lib$(TARGET).a: $(C_OBJS)
	@$(AR) $(ARFLAGS) $@$(C_OBJS)
	@echo "Packed into archive $@"

utest:
	@echo "Building and executing unit tests as executable on PC"
	@mkdir -p build
	@rake -m -j 4 options:$(CC) clobber test:all

coverage:
	@echo "Produce coverage information"
	@mkdir -p build
	@rm -f tools/ceedling/plugins/gcov/config/defaults.yml # we delete these because settings are merged
	@rake -m -j 4 options:$(CC) clobber gcov:all
	@lcov --directory $(OBJDIR)/gcov/out/ --output-file $(OBJDIR)/lcov.info $(LCOVFLAGS) $(LCOVCONFIG)
	@genhtml $(OBJDIR)/lcov.info -o $(OBJDIR)/coverage/ $(LCOVCONFIG)

uncrustify:
	uncrustify -c tools/uncrustify.cfg --replace --no-backup -l C $(UNCRUSTIFY_FILES)

ci_uncrustify:
	uncrustify -c tools/uncrustify.cfg --check -l C $(UNCRUSTIFY_FILES)



# Doxygen documentation that is stored in the source files is referenced from the
# # rest documentation, we need to make sure we publish it at the same time
doxy:
	@echo "** Running doxygen on source code, destination of doxygen build is the final html folder."
	@mkdir -p $(OBJDIR)/html/doxygen
	@export VERSION=$(VERSION) && doxygen doxygen/doxyfile > /dev/null
clean:
	@rm -rf $(OBJDIR)
	@rm -f $(TARGET)
	@rm -f lib$(TARGET).a
	@echo "Deleted $(OBJDIR)/ and $(TARGET)"

ctags: cscope

cscope:
	@echo "Building cscope database and ctags"
	@rm -f cscope.files cscope.out ctags
	@find . -name "*.c" >> cscope.files
	@find . -name "*.h" >> cscope.files
	@cscope -b
	@ctags --fields=+l --langmap=c:.c.h $(SRCS) inc/* tools/ceedling/vendor/unity/src/*

# ==============================
# Include automatic dependencies
# ==============================
DEPS = $(OBJS:%.o=%.d)
ifneq (${MAKECMDGOALS},clean)
	-include $(DEPS)
endif
