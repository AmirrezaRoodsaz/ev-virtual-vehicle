# Builds the C fast domain into a host shared library for the Python co-sim.
#
# The same sources compile unchanged for an embedded target: C11 + libm only,
# no HAL, no OS, no malloc. `firmware/` mixes vendored control code (read-only
# here, see firmware/VENDORED.md) with the plant model owned by this repo.

CC      := cc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 -fPIC
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    LIBEXT := dylib
else
    LIBEXT := so
endif

BUILD   := build
LIB     := $(BUILD)/libevtwin.$(LIBEXT)
SRCS    := $(wildcard firmware/*.c)
HDRS    := $(wildcard firmware/*.h)
VENV    := .venv
PY      := $(VENV)/bin/python

.PHONY: all venv test clean

all: $(LIB)

$(LIB): $(SRCS) $(HDRS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -shared -o $@ $(SRCS) -lm

# Python 3.11 is the portfolio convention; CI provides it directly.
venv: requirements.txt
	python3.11 -m venv $(VENV)
	$(PY) -m pip install -q --upgrade pip
	$(PY) -m pip install -q -r requirements.txt

test: $(LIB)
	$(PY) -m pytest tests/ -q

clean:
	rm -rf $(BUILD) .pytest_cache
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
