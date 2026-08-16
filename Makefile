# Makefile — GVSOC-RVVI bridge library for CV32E40P DPI co-simulation.
# Pattern: gvsoc/core/docs/.../tutorials/17_how_to_control_gvsoc_from_an_external_simulator
#
# Layout:  build/ (cmake GVSOC) · install/ (lib,bin,python,models) · work/ (config gvrun)
#
# Target:
#   make gvsoc                  build GVSOC + the CV32E40P iss_v2 platforms
#   make            (= all)     build libgvsoc_rvvi_v2.so / _v2_zfinx.so / librvvi_text.so
#   make config-v2 BINARY=...   generate gvsoc_config.json (gvrun prepare)
#   make trace  BINARY=...      standalone GVSOC with instruction trace → $(TRACE)
#   make clean / distclean
# Knobs: DEBUG=1 (-g -O0 for gdb/VS Code) · V2_TARGET (core config, baked per target)
#
# Requires the Python env (gvrun/config-gen): micromamba -n gvsoc_env_3_12 (see README).
# The Python generators (cv32e40p-v2-standalone*.py, cv32e40p_exit/) live in the
# gvsoc/pulp/ submodule and are installed by 'make gvsoc'.

QUESTA_HOME ?= /tools/siemens/questa_2025.3/questasim
GVSOC_HOME  := $(abspath gvsoc)
RVVI_DIR    := $(abspath RVVI)

CURDIR      := $(abspath .)
BUILDDIR    ?= $(CURDIR)/build
INSTALLDIR  ?= $(CURDIR)/install
WORK_DIR    ?= $(CURDIR)/work

# ELF to simulate — required by 'make trace'/'make config-v2'. In DPI the
# bridge injects the binary at runtime, so this stays a placeholder.
BINARY           ?= __BINARY_NOT_SET__
TRACE            ?= gvsoc_trace.log

GVSOC_LIB := $(INSTALLDIR)/lib
CXX       := g++

# -MMD: emit a .d per object so GVSOC/personality header edits retrigger the
# bridge compile (the hand-written prerequisites below only cover local headers).
CXXFLAGS_BASE := -std=c++17 -fPIC -O2 -MMD -MP \
                 -I$(RVVI_DIR)/include/host/rvvi

# iss_v2 bridge (gvsoc_engine_v2.cpp). The module layout is fixed by the
# generated ISA header (personality types + layout-affecting defines), so it
# is force-included: run `make gvsoc` first, the header lives in the GVSOC
# build tree. Base and FPU share one layout (verified by static_assert +
# runtime canary); ZFINX drops the 32 float registers, hence a second build.
# -DRISCV selects the RISC-V (not legacy RISCY) SIMD operand order in shared
# int.h. The recipe also emits it into the generated ISA header; kept here so
# the operand order does not silently flip on a stale header.
ISS_DEFINES_V2 := -DNDEBUG -D__GVSOC__ -DISS_WORD_32 \
                  -DRISCV=1 \
                  -DCONFIG_GVSOC_ISS_V2=1 \
                  -DCONFIG_GVSOC_ISS_TIMED=1 \
                  -DCONFIG_GVSOC_ISS_HTIF=1 \
                  -DCONFIG_GVSOC_ISS_FLOAT_USE_FLEXFLOAT=1 \
                  -DCONFIG_GVSOC_ISS_FP_WIDTH=32
ISS_DEFINES_V2_ZFINX := $(ISS_DEFINES_V2) -DCONFIG_GVSOC_ISS_ZFINX=1
ISA_HDR_V2       := $(shell ls -t $(BUILDDIR)/engine/isa_cv32e40p_v2_rv32imfc_pulp_[0-9]*.hpp 2>/dev/null | head -1)
ISA_HDR_V2_ZFINX := $(shell ls -t $(BUILDDIR)/engine/isa_cv32e40p_v2_rv32imfc_pulp_zfinx_*.hpp 2>/dev/null | head -1)
ISS_INCLUDES_V2  := -I$(BUILDDIR)/engine \
                    -I$(GVSOC_HOME)/config_tree \
                    -I$(GVSOC_HOME)/engine/engine/include \
                    -I$(GVSOC_HOME)/core/models \
                    -I$(GVSOC_HOME)/pulp

# DEBUG=1 → -g -O0 (gdb/VS Code). -O0 wins over -O2 because it comes later.
# It does not change the ABI (which depends on the -D CONFIG_*, not on -O):
# the .so stays compatible with libpulpvp.so.
DEBUG ?= 0
ifeq ($(DEBUG),1)
CXXFLAGS_BASE += -g -O0
endif

CXXFLAGS              := $(CXXFLAGS_BASE)                                  # rvvi_api2gvsoc.cpp
CXXFLAGS_ENGINE_V2       := $(CXXFLAGS_BASE) $(ISS_DEFINES_V2) -include $(ISA_HDR_V2) $(ISS_INCLUDES_V2)
CXXFLAGS_ENGINE_V2_ZFINX := $(CXXFLAGS_BASE) $(ISS_DEFINES_V2_ZFINX) -include $(ISA_HDR_V2_ZFINX) $(ISS_INCLUDES_V2)

# Library options only: at link time the objects MUST precede -lpulpvp.
LDFLAGS := -L$(GVSOC_LIB) -lpulpvp -Wl,-rpath,$(GVSOC_LIB)

ifdef QUESTA_HOME
    CXXFLAGS                 += -I$(QUESTA_HOME)/include
    CXXFLAGS_ENGINE_V2       += -I$(QUESTA_HOME)/include
    CXXFLAGS_ENGINE_V2_ZFINX += -I$(QUESTA_HOME)/include
else ifdef VCS_HOME
    CXXFLAGS                 += -I$(VCS_HOME)/include
    CXXFLAGS_ENGINE_V2       += -I$(VCS_HOME)/include
    CXXFLAGS_ENGINE_V2_ZFINX += -I$(VCS_HOME)/include
endif

TARGET_V2       := libgvsoc_rvvi_v2.so
TARGET_V2_ZFINX := libgvsoc_rvvi_v2_zfinx.so
TARGET_TEXT     := librvvi_text.so
OBJS_V2         := rvvi_api2gvsoc.o gvsoc_engine_v2.o rvvi_text_writer.o
OBJS_V2_ZFINX   := rvvi_api2gvsoc.o gvsoc_engine_v2_zfinx.o rvvi_text_writer.o
OBJS_TEXT       := rvvi_text_dpi.o rvvi_text_writer.o

# Installed gvrun: it sets LD_LIBRARY_PATH/PATH/PYTHONPATH/USE_GVRUN/--platform by itself.
# gvrun needs the Python env (see README): wrap it in 'micromamba run' when
# micromamba is available, so the caller's shell does not have to activate it.
GVSOC_PY_ENV ?= gvsoc_env_3_12
MICROMAMBA   := $(shell command -v micromamba 2>/dev/null)
ifneq ($(MICROMAMBA),)
  GVRUN_ENV := $(MICROMAMBA) run -n $(GVSOC_PY_ENV)
endif
GVRUN := $(GVRUN_ENV) timeout 120s $(INSTALLDIR)/bin/gvrun

.PHONY: all gvsoc config-v2 trace clean distclean test

all: $(TARGET_V2) $(TARGET_V2_ZFINX) $(TARGET_TEXT)

# Separate compilation: rvvi_api2gvsoc.cpp with the base flags, gvsoc_engine_v2.cpp with the ISS flags.
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

gvsoc_engine_v2.o: gvsoc_engine_v2.cpp gvsoc_engine.hpp $(ISA_HDR_V2)
	@test -n "$(ISA_HDR_V2)" || { echo "generated ISA header not found - run 'make gvsoc' first"; exit 1; }
	$(CXX) $(CXXFLAGS_ENGINE_V2) -c $< -o $@

gvsoc_engine_v2_zfinx.o: gvsoc_engine_v2.cpp gvsoc_engine.hpp $(ISA_HDR_V2_ZFINX)
	@test -n "$(ISA_HDR_V2_ZFINX)" || { echo "generated ISA header not found - run 'make gvsoc' first"; exit 1; }
	$(CXX) $(CXXFLAGS_ENGINE_V2_ZFINX) -c $< -o $@

$(TARGET_V2): $(OBJS_V2)
	$(CXX) -shared $(OBJS_V2) $(LDFLAGS) -o $@
	@echo "OK → $(TARGET_V2)"

$(TARGET_V2_ZFINX): $(OBJS_V2_ZFINX)
	$(CXX) -shared $(OBJS_V2_ZFINX) $(LDFLAGS) -o $@
	@echo "OK → $(TARGET_V2_ZFINX)"

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
CV32E40P_TARGETS := cv32e40p-v2-spike \
                    cv32e40p-v2-spike-fpu \
                    cv32e40p-v2-spike-zfinx \
                    cv32e40p-v2-standalone \
                    cv32e40p-v2-standalone-nopulp \
                    cv32e40p-v2-standalone-fpu \
                    cv32e40p-v2-standalone-zfinx

gvsoc:
	$(MAKE) -C $(GVSOC_HOME) \
		TARGETS="$(CV32E40P_TARGETS)" \
		BUILDDIR=$(BUILDDIR) \
		INSTALLDIR=$(INSTALLDIR) \
		build

# make config-v2 — generate gvsoc_config.json (gvrun prepare, without running
# the sim). The core configuration is baked into the target name, so there
# are no --parameter knobs besides the binary.
#
# Parallel-safe: every `make test` lane regenerates its per-CFG json through
# this target. The historical SHARED work dir + plain `cp` raced across
# concurrent lanes — a pulp lane could publish the gvsoc_config.json a
# pulp_fpu lane's prepare had just written into the same work/ directory,
# and the bridge then booted the ISS with the wrong misa/fpu (CSR[0x301]
# mismatch at retire #1, F-bit delta). A unique mktemp work dir per
# invocation plus an atomic publish (tmp + mv on the same filesystem)
# close both the cross-CFG corruption and torn reads.
V2_TARGET ?= cv32e40p-v2-standalone
config-v2:
ifeq ($(BINARY),__BINARY_NOT_SET__)
	$(error BINARY not set — use: make config-v2 BINARY=/path/to/test.elf [V2_TARGET=cv32e40p-v2-standalone-fpu])
endif
	set -e; \
	wd=$$(mktemp -d $(CURDIR)/work_cfg.XXXXXX); \
	trap 'rm -rf "$$wd"' EXIT; \
	$(GVRUN) \
		--target=$(V2_TARGET) \
		--work-dir=$$wd \
		--parameter binary=$(BINARY) \
		prepare; \
	out="$(if $(GVSOC_CONFIG),$(GVSOC_CONFIG),gvsoc_config.json)"; \
	cp $$wd/gvsoc_config.json "$$out.tmp.$$$$"; \
	mv "$$out.tmp.$$$$" "$$out"; \
	echo "OK → $$out (target=$(V2_TARGET), binary=$(BINARY))"

# make trace — standalone GVSOC with instruction trace → $(TRACE), a
# diagnostic tool (behavioural ISS analysis without the DPI co-simulation).
# Core configuration via V2_TARGET, as for config-v2.
trace:
ifeq ($(BINARY),__BINARY_NOT_SET__)
	$(error BINARY not set — use: make trace BINARY=/path/to/test.elf [V2_TARGET=...])
endif
	-$(GVRUN) \
		--target=$(V2_TARGET) \
		--work-dir=$(WORK_DIR) \
		--trace=insn \
		--trace-level=trace \
		--trace-format=short \
		--parameter binary=$(BINARY) \
		run > $(TRACE) 2>&1

clean:
	rm -f $(TARGET_V2) $(TARGET_V2_ZFINX) $(TARGET_TEXT) $(OBJS_V2) gvsoc_engine_v2_zfinx.o rvvi_text_dpi.o test/test_rvvi_text_writer *.d
	rm -rf $(WORK_DIR)

distclean: clean
	rm -rf $(BUILDDIR) $(INSTALLDIR)

-include $(wildcard *.d)
