################################################################################
# Tools
#

#
# Decide where we are going to look for tools
#
ifeq ($(SYSTYPE),Darwin)
  # use the tools that come with Arduino
  # use BWK awk
  AWK =  awk
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
endif
ifeq ($(SYSTYPE),Linux)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
endif
ifeq ($(findstring CYGWIN, $(SYSTYPE)),CYGWIN)
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL = $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
endif
ifeq ($(findstring MINGW, $(SYSTYPE)),MINGW)
  # expect that tools are on the path
  TOOLPATH :=  $(subst :, ,$(PATH))
  FIND_TOOL    =  $(firstword $(wildcard $(addsuffix /$(1).exe,$(TOOLPATH))))
endif

NATIVE_CXX     :=  g++
NATIVE_CC      :=  gcc
NATIVE_AS      :=  gcc
NATIVE_AR      :=  ar
NATIVE_LD      :=  g++
NATIVE_GDB     :=  gdb
NATIVE_OBJCOPY :=  objcopy

AVARICE      :=  $(call FIND_TOOL,avarice)

# toolchains for beagleboneblack
BBONE_CXX     :=  arm-linux-gnueabihf-g++
BBONE_CC      :=  arm-linux-gnueabihf-gcc
BBONE_AS      :=  arm-linux-gnueabihf-gcc
BBONE_AR      :=  ar
BBONE_LD      :=  arm-linux-gnueabihf-g++
BBONE_GDB     :=  gdb
BBONE_OBJCOPY :=  objcopy

# toolchains for Raspberry Pi
RPI_CXX     :=  arm-linux-gnueabihf-g++
RPI_CC      :=  arm-linux-gnueabihf-gcc
RPI_AS      :=  arm-linux-gnueabihf-gcc
RPI_AR      :=  arm-linux-gnueabihf-ar
RPI_LD      :=  arm-linux-gnueabihf-g++
RPI_GDB     :=  arm-linux-gnueabihf-gdb
RPI_OBJCOPY :=  arm-linux-gnueabihf-obj

# toolchains for zynq
ZYNQ_CXX     :=  arm-xilinx-linux-gnueabi-g++
ZYNQ_CC      :=  arm-xilinx-linux-gnueabi-gcc
ZYNQ_AS      :=  arm-xilinx-linux-gnueabi-gcc
ZYNQ_AR      :=  arm-xilinx-linux-gnueabi-ar
ZYNQ_LD      :=  arm-xilinx-linux-gnueabi-g++
ZYNQ_GDB     :=  arm-xilinx-linux-gnueabi-gdb
ZYNQ_OBJCOPY :=  arm-xilinx-linux-gnueabi-objcopy

#toolchain for altera
ALTERA_CXX     :=  arm-linux-gnueabihf-g++
ALTERA_CC      :=  arm-linux-gnueabihf-gcc
ALTERA_AS      :=  arm-linux-gnueabihf-gcc
ALTERA_AR      :=  arm-linux-gnueabihf-ar
ALTERA_LD      :=  arm-linux-gnueabihf-g++
ALTERA_GDB     :=  arm-linux-gnueabihf-gdb
ALTERA_OBJCOPY :=  arm-linux-gnueabihf-objcopy

# enable ccache if installed
CCACHE :=  $(call FIND_TOOL,ccache)
export CCACHE

CXX = $(CCACHE) $($(TOOLCHAIN)_CXX)
CC = $(CCACHE) $($(TOOLCHAIN)_CC)
AS = $($(TOOLCHAIN)_AS)
AR = $($(TOOLCHAIN)_AR)
LD = $($(TOOLCHAIN)_LD)
GDB = $($(TOOLCHAIN)_GDB)
OBJCOPY = $($(TOOLCHAIN)_OBJCOPY)

ifeq ($(CXX),)
$(error ERROR: cannot find the compiler tools for $(TOOLCHAIN) anywhere on the path $(TOOLPATH))
endif

# Find awk
AWK			?=	gawk
ifeq ($(shell which $(AWK)),)
$(error ERROR: cannot find $(AWK) - you may need to install GNU awk)
endif
