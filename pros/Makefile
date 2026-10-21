################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
# RW draws the Brain screen itself, so this project doesn't install liblvgl for LLEMU (pros::lcd)
EXTRA_CXXFLAGS=-D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
# RW's library uses the robot's devices and settings by name, so it links with them (see firmware/rw-template.mk)
EXCLUDE_COLD_LIBRARIES:= $(BINDIR)/RW-Template.a

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
# RW-Template builds as a PROS template: "pros make template" (the release Action passes VERSION)
LIBNAME:=RW-Template
VERSION:=0.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))
# The robot's own files, which the template adds to a project as starting points (user files)
RW_USER_FILES:=$(SRCDIR)/rw-config.cpp $(SRCDIR)/rw-autons.cpp $(SRCDIR)/rw-user.cpp $(INCDIR)/rw-config.h
EXCLUDE_SRC_FROM_LIB+=$(filter %.cpp,$(RW_USER_FILES))

# RW's headers, the make settings that link RW with the robot's code, and the simulator (sim/rw has
# RW's code for it, since the library only runs on the robot)
TEMPLATE_FILES=$(INCDIR)/rw/*.h $(FWDIR)/rw-template.mk
TEMPLATE_FILES+=$(ROOT)/sim/*.sh $(ROOT)/sim/*.cpp $(ROOT)/sim/*.h $(ROOT)/sim/*.html $(ROOT)/sim/.gitignore $(ROOT)/sim/pros/*.h $(ROOT)/sim/rw/*.cpp
CREATE_TEMPLATE_FLAGS=$(foreach file,$(RW_USER_FILES),--user "$(file)") --kernels "^4.2.1"

# RW's shared code lives in ../src and ../include, next to the VEXcode version. rw/sync.sh copies it
# (and the PROS-only files in rw/) into src/rw and include/rw before every build. Git ignores the copies
RW_SYNC:=$(shell sh rw/sync.sh)
ifneq ($(RW_SYNC),ok)
$(error Couldn't copy RW's shared code into this project: $(RW_SYNC))
endif

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

# The cold package's link lists every library, so RW's has to exist first, even though it links with the hot one
$(COLD_ELF): | $(LIBAR)
