# Quanton build is via external build system, and very similar to PX4/Pixhawk builds, using NuttX

ifneq ($(PX4_ROOT),)

# cope with relative paths
ifeq ($(wildcard $(PX4_ROOT)/nuttx-configs),)
PX4_ROOT := $(shell cd $(SKETCHBOOK)/$(PX4_ROOT) && pwd)
endif

# check it is a valid PX4Firmware tree
ifeq ($(wildcard $(PX4_ROOT)/nuttx-configs),)
$(error ERROR: PX4_ROOT not set correctly - no nuttx-configs directory found)
endif

# default to PX4NuttX above the PX4Firmware tree
ifeq ($(NUTTX_SRC),)
NUTTX_SRC := $(shell cd $(PX4_ROOT)/../PX4NuttX/nuttx && pwd)/
endif

# cope with relative paths for NUTTX_SRC
ifeq ($(wildcard $(NUTTX_SRC)/configs),)
NUTTX_SRC := $(shell cd $(SKETCHBOOK)/$(NUTTX_SRC) && pwd)/
endif

ifeq ($(wildcard $(NUTTX_SRC)configs),)
$(error ERROR: NUTTX_SRC not set correctly - no configs directory found)
endif

# we have different config files for V1 and V2
QUANTON_CONFIG_FILE=$(MK_DIR)/PX4/config_quanton_APM.mk

SKETCHFLAGS=$(SKETCHLIBINCLUDES) -I$(PWD) -DARDUPILOT_BUILD -DCONFIG_HAL_BOARD=HAL_BOARD_QUANTON -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=ArduPilot_main -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)

WARNFLAGS = -Wno-psabi -Wno-packed

PX4_MAKE = $(v) make -C $(SKETCHBOOK) -f $(PX4_ROOT)/Makefile EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) "'$(EXTRAFLAGS)' APM_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) PX4_ROOT=$(PX4_ROOT) NUTTX_SRC=$(NUTTX_SRC) MAXOPTIMIZATION="-Os"
PX4_MAKE_ARCHIVES = make -C $(PX4_ROOT) NUTTX_SRC=$(NUTTX_SRC) archives MAXOPTIMIZATION="-Os"

.PHONY: module_mk
module_mk:
	$(RULEHDR)
	$(v) echo "# Auto-generated file - do not edit" > $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_COMMAND = ArduPilot" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "SRCS = Build.$(SKETCH)/$(SKETCH).cpp $(SKETCHLIBSRCSRELATIVE)" >> $(SKETCHBOOK)/module.mk.new
	$(v) echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	$(v) cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	$(v) rm -f $(SKETCHBOOK)/module.mk.new

quanton: showflags $(PX4_ROOT)/Archives/quanton.export $(SKETCHCPP) module_mk
	$(RULEHDR)
	$(v) rm -f $(PX4_ROOT)/makefiles/$(QUANTON_CONFIG_FILE)
	$(v) cp $(PWD)/$(QUANTON_CONFIG_FILE) $(PX4_ROOT)/makefiles/
	$(v) $(PX4_MAKE) quanton_APM
	$(v) /bin/rm -f $(SKETCH).px4
	$(v) cp $(PX4_ROOT)/Images/quanton_APM.px4 $(SKETCH).px4
	$(v) echo "PX4 $(SKETCH) Firmware is in $(SKETCH).px4"


quanton-clean: clean px4-archives-clean
	$(v) /bin/rm -rf $(PX4_ROOT)/makefiles/build $(PX4_ROOT)/Build

quanton-upload: quanton
	$(RULEHDR)
	$(v) $(PX4_MAKE) quanton_APM upload

px4-archives-clean:
	$(v) /bin/rm -rf $(PX4_ROOT)/Archives

$(PX4_ROOT)/Archives/quanton.export:
	$(v) $(PX4_MAKE_ARCHIVES)

px4-archives:
	$(v) $(PX4_MAKE_ARCHIVES)

else

px4:
	$(error ERROR: You need to add PX4_ROOT to your config.mk)

quanton-clean: quanton

quanton-upload: quanton

endif
