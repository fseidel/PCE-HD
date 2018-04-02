# Target executable
TARGET = simv

# Source files
SRC := $(wildcard *.vh) $(wildcard *.sv) $(wildcard ../HuC6260/*.sv) \
	   $(wildcard ../HuC6270/vdc_HuC6270.sv) $(wildcard ../HuC6270/VRAM.sv) \
	   $(wildcard ../HuC6280/*.v) $(wildcard ../synth/*_synth.v)\
	   $(wildcard ../synth/MAIN_RAM.v) $(wildcard ../HuC6280/MMU.sv)
      #RAMTest.v
SRC := $(filter-out ../HuC6270/gfx_tb.sv, $(SRC))
#SRC := $(filter-out ../HuC6270/DataBusBuffer.sv, $(SRC))
SRC := $(filter-out ../HuC6280/testbench.v, $(SRC))
SRC := $(filter-out ../HuC6280/PSG.v, $(SRC))
SRC := $(filter-out ../HuC6280/cpu.v, $(SRC))
SRC := $(filter-out ../HuC6280/cpu_65c02.v, $(SRC))
SRC := $(filter-out ../HuC6280/memory.v, $(SRC))

# Set the number of threads to use for parallel compilation (2 * cores)
CORES = $(shell getconf _NPROCESSORS_ONLN)
THREADS = $(shell echo $$((2 * $(CORES))))

# VCS flags #+lint=all
VCSFLAGS = -full64 -sverilog -debug_all +warn=all -j$(THREADS) \
			-timescale=1ns/1ps +v2k +incdir+../HuC6260+../HuC6270 \

#			+vcs+vcdpluson

COMMON_FLAGS +=

# Simulator
SIM = vcs

ifndef foo
# Altera FPGA library files (for simulation)
INC_V = /afs/ece/support/altera/release/12.1/quartus/eda/sim_lib/altera_primitives.v \
				/afs/ece/support/altera/release/12.1/quartus/eda/sim_lib/220model.v \
				/afs/ece/support/altera/release/12.1/quartus/eda/sim_lib/sgate.v \
				/afs/ece/support/altera/release/12.1/quartus/eda/sim_lib/altera_mf.v \
				/afs/ece/support/altera/release/12.1/quartus/eda/sim_lib/cycloneiv_atoms.v
INC_V_FLAGS = $(addprefix -v , $(INC_V))
INC_SV =
INC_SV_FLAGS = $(addprefix -v , $(INC_SV))
else
INC_V = 
INC_V_FLAGS = 
INC_SV =
INC_SV_FLAGS =
endif

# Copy common flags
VCSFLAGS += $(COMMON_FLAGS)

vdc: 
	$(SIM) $(VCSFLAGS) $(INC_V_FLAGS) $(INC_SV_FLAGS) -o $(TARGET) $(SRC)

clean :
	-rm -r csrc
	-rm -r DVEfiles
	-rm $(TARGET)
	-rm -r $(TARGET).daidir
	-rm ucli.key

