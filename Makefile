# SPDX-FileCopyrightText: 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0

# RVVI bridge for the CV32E40P model of GVSOC.
#
#   make GVSOC_HOME=<gvsoc source tree> GVSOC_INSTALL=<gvsoc install dir>
#
# GVSOC must be built with this repository as a module
# (make build MODULES=$(CURDIR)/gvsoc) so that the cv32e40p_cosim* targets exist.

GVSOC_HOME    ?= $(error GVSOC_HOME is not set)
GVSOC_INSTALL ?= $(error GVSOC_INSTALL is not set)
BUILDDIR      ?= build

CXX      ?= g++
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -fPIC -Wall -Werror -MMD -MP \
            -IRVVI/include/host/rvvi \
            -I$(GVSOC_HOME)/engine/engine/include \
            -I$(GVSOC_HOME)/pulp
LDLIBS   := -L$(GVSOC_INSTALL)/lib -lpulpvp -Wl,-rpath,$(GVSOC_INSTALL)/lib -ldl

LIB  := $(BUILDDIR)/libcv32e40p_rvvi.so
OBJS := $(BUILDDIR)/rvvi_api.o $(BUILDDIR)/cosim_client.o

all: $(LIB) $(BUILDDIR)/cosim_run

$(LIB): $(OBJS)
	$(CXX) -shared -o $@ $^ $(LDLIBS)

$(BUILDDIR)/cosim_run: $(BUILDDIR)/cosim_run.o $(BUILDDIR)/cosim_client.o
	$(CXX) -o $@ $^ $(LDLIBS)

$(BUILDDIR)/%.o: bridge/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: test/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -Ibridge -c $< -o $@

$(BUILDDIR):
	mkdir -p $@

clean:
	rm -rf $(BUILDDIR)

-include $(wildcard $(BUILDDIR)/*.d)

.PHONY: all clean
