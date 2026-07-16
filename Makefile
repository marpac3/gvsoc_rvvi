# Makefile — GVSOC-RVVI bridge library for CV32E40P DPI co-simulation.
# Pattern: gvsoc/core/docs/.../tutorials/17_how_to_control_gvsoc_from_an_external_simulator
#
# Layout:  build/ (cmake GVSOC) · install/ (lib,bin,python,models) · work/ (config gvrun)
#
# Target:
#   make gvsoc               build GVSOC + the CV32E40P ISA models (6 variants)
#   make            (= all)  build libgvsoc_rvvi.so / _zfinx.so / librvvi_text.so
#   make config BINARY=...   generate gvsoc_config.json (gvrun prepare)
#   make trace  BINARY=...   standalone GVSOC with instruction trace → $(TRACE)
#   make clean / distclean
# Knobs: DEBUG=1 (-g -O0 for gdb/VS Code) · COREV_PULP / FPU / ZFINX / COREV_CLUSTER
#
# Requires the Python env (gvrun/config-gen): micromamba -n gvsoc_env_3_12 (see README).
# The Python generators (cv32e40p-standalone.py, cv32e40p_exit/) live in the
# gvsoc/pulp/ submodule and are installed by 'make gvsoc'.

QUESTA_HOME ?= /tools/siemens/questa_2025.3/questasim
GVSOC_HOME  := $(abspath gvsoc)
RVVI_DIR    := $(abspath RVVI)

CURDIR      := $(abspath .)
BUILDDIR    ?= $(CURDIR)/build
INSTALLDIR  ?= $(CURDIR)/install
WORK_DIR    ?= $(CURDIR)/work

# ELF to simulate — required by 'make trace'/'make config'. In DPI the bridge
# injects the binary at runtime, so this stays a placeholder.
BINARY           ?= __BINARY_NOT_SET__
COREV_PULP       ?= 0
FPU              ?= 0
ZFINX            ?= 0
COREV_CLUSTER    ?= 0
NUM_MHPMCOUNTERS ?= 1
TRACE            ?= gvsoc_trace.log

GVSOC_LIB := $(INSTALLDIR)/lib
CXX       := g++

CXXFLAGS_BASE := -std=c++17 -fPIC -O2 \
                 -I$(RVVI_DIR)/include/host/rvvi

# ISS defines — they MUST match the ISS model .so build (extracted from the
# compile_commands.json of gen_isa_cv32e40p_core_v2_nof_noz).
ISS_DEFINES := -DNDEBUG -D__GVSOC__ \
               -DCONFIG_GVSOC_ISS_FP_WIDTH=32 \
               -DRISCV=1 -DRISCY -DISS_WORD_32 \
               -DCONFIG_GVSOC_ISS_RISCV=1 \
               -DCONFIG_GVSOC_ISS_SCOREBOARD=1 \
               -DCONFIG_GVSOC_ISS_TIMED=1 \
               -DCONFIG_GVSOC_ISS_FLOAT_USE_FLEXFLOAT=1 \
               -DCONFIG_GVSOC_ISS_RISCV_EXCEPTIONS=1 \
               -DCONFIG_GVSOC_ISS_HANDLE_MISALIGNED=1 \
               -DPIPELINE_STALL_THRESHOLD=1 \
               -DCONFIG_ISS_CORE=cv32e40p \
               -DCONFIG_GVSOC_CORE_VERSION=2 \
               -DCONFIG_GVSOC_ISS_HWLOOP=1 \
               -DCONFIG_GVSOC_ISS_CV32E40P=1

# Engine includes, from the source tree (the GVSOC cmake does not stage them
# in install/include). Only two paths are needed: engine/engine/include
# (gv/gvsoc.hpp, vp/*) and core/models (cpu/iss/include/iss.hpp, cores/cv32e40p/*).
ISS_INCLUDES := -I$(GVSOC_HOME)/engine/engine/include \
                -I$(GVSOC_HOME)/core/models

# ZFINX: the ISS .so is built with ISS_SINGLE_REGFILE=1, which removes fregs[]
# and the two float-scoreboard arrays from the Regfile (regfile.hpp):
# sizeof(Regfile) 1592 -> 944 bytes. gvsoc_engine.cpp reads the Regfile by
# offset (iss.regfile.regs[]/fregs[]), so it MUST use the same flag: with a
# different layout the downstream fields (PC, CSR) sit at wrong offsets and
# fregs[] does not exist. Hence two distinct .so builds.
ISS_DEFINES_ZFINX := $(ISS_DEFINES) -DISS_SINGLE_REGFILE=1 -DCONFIG_GVSOC_ISS_ZFINX=1

# DEBUG=1 → -g -O0 (gdb/VS Code). -O0 wins over -O2 because it comes later.
# It does not change the ABI (which depends on the -D CONFIG_*, not on -O):
# the .so stays compatible with libpulpvp.so.
DEBUG ?= 0
ifeq ($(DEBUG),1)
CXXFLAGS_BASE += -g -O0
endif

CXXFLAGS              := $(CXXFLAGS_BASE)                                  # rvvi_api2gvsoc.cpp
CXXFLAGS_ENGINE       := $(CXXFLAGS_BASE) $(ISS_DEFINES) $(ISS_INCLUDES)   # gvsoc_engine.cpp
CXXFLAGS_ENGINE_ZFINX := $(CXXFLAGS_BASE) $(ISS_DEFINES_ZFINX) $(ISS_INCLUDES)

# Library options only: at link time the objects MUST precede -lpulpvp.
LDFLAGS := -L$(GVSOC_LIB) -lpulpvp -Wl,-rpath,$(GVSOC_LIB)

ifdef QUESTA_HOME
    CXXFLAGS              += -I$(QUESTA_HOME)/include
    CXXFLAGS_ENGINE       += -I$(QUESTA_HOME)/include
    CXXFLAGS_ENGINE_ZFINX += -I$(QUESTA_HOME)/include
else ifdef VCS_HOME
    CXXFLAGS              += -I$(VCS_HOME)/include
    CXXFLAGS_ENGINE       += -I$(VCS_HOME)/include
    CXXFLAGS_ENGINE_ZFINX += -I$(VCS_HOME)/include
endif

TARGET       := libgvsoc_rvvi.so
TARGET_ZFINX := libgvsoc_rvvi_zfinx.so
TARGET_TEXT  := librvvi_text.so
OBJS         := rvvi_api2gvsoc.o gvsoc_engine.o rvvi_text_writer.o
OBJS_ZFINX   := rvvi_api2gvsoc.o gvsoc_engine_zfinx.o rvvi_text_writer.o
OBJS_TEXT    := rvvi_text_dpi.o rvvi_text_writer.o

# Installed gvrun: it sets LD_LIBRARY_PATH/PATH/PYTHONPATH/USE_GVRUN/--platform by itself.
# gvrun needs the Python env (see README): wrap it in 'micromamba run' when
# micromamba is available, so the caller's shell does not have to activate it.
GVSOC_PY_ENV ?= gvsoc_env_3_12
MICROMAMBA   := $(shell command -v micromamba 2>/dev/null)
ifneq ($(MICROMAMBA),)
  GVRUN_ENV := $(MICROMAMBA) run -n $(GVSOC_PY_ENV)
endif
GVRUN := $(GVRUN_ENV) timeout 10s $(INSTALLDIR)/bin/gvrun

.PHONY: all gvsoc config trace clean distclean test

all: $(TARGET) $(TARGET_ZFINX) $(TARGET_TEXT)

# Separate compilation: rvvi_api2gvsoc.cpp with the base flags, gvsoc_engine.cpp with the ISS flags.
rvvi_api2gvsoc.o: rvvi_api2gvsoc.cpp gvsoc_engine.hpp rvvi_text_writer.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# rvvi_text_writer.cpp — standalone RVVI-TEXT formatter: pure C++, no ISS
# define/include, so it builds with the base flags only. -fvisibility=hidden:
# the formatter is internal to each .so, and dual-trace loads both libraries
# in one simulation — exported copies could interpose on a partial rebuild.
rvvi_text_writer.o: rvvi_text_writer.cpp rvvi_text_writer.hpp
	$(CXX) $(CXXFLAGS) -fvisibility=hidden -c $< -o $@

# rvvi_text_dpi.cpp — DPI shim around the formatter (RTL-only tracer entry points).
# Pure C++, base flags, no ISS define/include.
rvvi_text_dpi.o: rvvi_text_dpi.cpp rvvi_text_dpi.hpp rvvi_text_writer.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

gvsoc_engine.o: gvsoc_engine.cpp gvsoc_engine.hpp
	$(CXX) $(CXXFLAGS_ENGINE) -c $< -o $@

gvsoc_engine_zfinx.o: gvsoc_engine.cpp gvsoc_engine.hpp
	$(CXX) $(CXXFLAGS_ENGINE_ZFINX) -c $< -o $@

$(TARGET): $(OBJS)
	$(CXX) -shared $(OBJS) $(LDFLAGS) -o $@
	@echo "OK → $(TARGET)"

$(TARGET_ZFINX): $(OBJS_ZFINX)
	$(CXX) -shared $(OBJS_ZFINX) $(LDFLAGS) -o $@
	@echo "OK → $(TARGET_ZFINX)"

# librvvi_text.so — standalone RVVI-TEXT writer for RTL-only sims (no GVSOC dep,
# so it links without libpulpvp / the embedded engine).
$(TARGET_TEXT): $(OBJS_TEXT)
	$(CXX) -shared $(OBJS_TEXT) -o $@
	@echo "OK → $(TARGET_TEXT)"

# make test — deterministic unit test of the RVVI-TEXT formatter.
# Pure C++, no GVSOC/ISS, no Questa license: checks the RET/TRAP/X/F/C/MODE line
# grammar byte-for-byte. Reuses rvvi_text_writer.o (base flags).
test: rvvi_text_writer.o rvvi_text_dpi.o
	$(CXX) $(CXXFLAGS_BASE) -I$(CURDIR) test/test_rvvi_text_writer.cpp rvvi_text_writer.o rvvi_text_dpi.o -o test/test_rvvi_text_writer
	./test/test_rvvi_text_writer

# make gvsoc — build GVSOC + the CV32E40P ISA models. The target:param=val
# syntax is handled by gvrun; USE_GVRUN=1 is forced by the GVSOC CMake (with
# gapy the inline parameters would not be injected).
CV32E40P_TARGETS := cv32e40p-standalone \
                    cv32e40p-standalone:corev_pulp=True \
                    cv32e40p-standalone:fpu=True \
                    cv32e40p-standalone:fpu=True,zfinx=True \
                    cv32e40p-standalone:corev_pulp=True,fpu=True \
                    cv32e40p-standalone:corev_pulp=True,fpu=True,zfinx=True \
                    cv32e40p-v2-spike

gvsoc:
	$(MAKE) -C $(GVSOC_HOME) \
		TARGETS="$(CV32E40P_TARGETS)" \
		BUILDDIR=$(BUILDDIR) \
		INSTALLDIR=$(INSTALLDIR) \
		build

# make config — generate gvsoc_config.json (gvrun prepare, without running the sim).
config:
ifeq ($(BINARY),__BINARY_NOT_SET__)
	$(error BINARY not set — use: make config BINARY=/path/to/test.elf)
endif
	mkdir -p $(WORK_DIR)
	$(GVRUN) \
		--target=cv32e40p-standalone \
		--work-dir=$(WORK_DIR) \
		--parameter binary=$(BINARY) \
		--parameter corev_pulp=$(COREV_PULP) \
		--parameter fpu=$(FPU) \
		--parameter zfinx=$(ZFINX) \
		--parameter corev_cluster=$(COREV_CLUSTER) \
		--parameter num_mhpmcounters=$(NUM_MHPMCOUNTERS) \
		prepare
	cp $(WORK_DIR)/gvsoc_config.json $(if $(GVSOC_CONFIG),$(GVSOC_CONFIG),gvsoc_config.json)
	@echo "OK → $(if $(GVSOC_CONFIG),$(GVSOC_CONFIG),gvsoc_config.json) (binary=$(BINARY), pulp=$(COREV_PULP), fpu=$(FPU), zfinx=$(ZFINX), num_mhpmcounters=$(NUM_MHPMCOUNTERS))"

# make trace — standalone GVSOC with instruction trace → $(TRACE).
trace:
ifeq ($(BINARY),__BINARY_NOT_SET__)
	$(error BINARY not set — use: make trace BINARY=/path/to/test.elf)
endif
	-$(GVRUN) \
		--target=cv32e40p-standalone \
		--work-dir=$(WORK_DIR) \
		--trace=insn \
		--trace-level=trace \
		--trace-format=short \
		--parameter binary=$(BINARY) \
		--parameter corev_pulp=$(COREV_PULP) \
		--parameter fpu=$(FPU) \
		--parameter zfinx=$(ZFINX) \
		--parameter corev_cluster=$(COREV_CLUSTER) \
		run > $(TRACE) 2>&1

clean:
	rm -f $(TARGET) $(TARGET_ZFINX) $(TARGET_TEXT) $(OBJS) gvsoc_engine_zfinx.o rvvi_text_dpi.o test/test_rvvi_text_writer
	rm -rf $(WORK_DIR)

distclean: clean
	rm -rf $(BUILDDIR) $(INSTALLDIR)
