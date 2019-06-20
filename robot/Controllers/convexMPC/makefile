appname := mpc_test


CXX := gcc
CPP := g++
CXXFLAGS := -O3 -Wall -g -std=gnu99 -march=native -I. -I_sharedutils/  #show warnings, include debugging symbols
CPPFLAGS := -O3 -Wall -g  -march=native #show warnings, include debugging symbols
#CXXFLAGS :=   -DMKL_ILP64 -m64 -I${MKLROOT}/include -O3 -Wall -g -std=gnu99 -march=native -I. -I_sharedutils/  -DEIGEN_USE_MKL_ALL#show warnings, include debugging symbols
#CPPFLAGS :=   -DMKL_ILP64 -m64 -I${MKLROOT}/include -O3 -Wall -g  -march=native -DEIGEN_USE_MKL_ALL#show warnings, include debugging symbols
INCLUDES := -I.


csrcfiles := $(shell find . -maxdepth 1 -name "*.c") #find all .c files
cppsrcfiles := $(shell find . -maxdepth 1 -name "*.cpp") #find all .c f

THIRD_PARTY_PATH = $(CHEETAH_SOFTWARE_PATH)/Third-Party


	LINKER_FLAGS := -L . -lqpOASES  -lpthread -llcm -lm -std=c++11 \
	-L$(THIRD_PARTY_PATH)/qpOASES-3.2.0/bin \
	-Wl,-rpath,$(THIRD_PARTY_PATH)/qpOASES-3.2.0/bin 
	#-Wl,-rpath,$(THIRD_PARTY_PATH)/qpOASES-3.2.0/bin,--start-group ${MKLROOT}/lib/intel64/libmkl_intel_ilp64.a 
	INCLUDE_FLAGS := -I$(THIRD_PARTY_PATH)/eigen \
	-I$(THIRD_PARTY_PATH)/qpOASES-3.2.0/include \
	#LINKER_FLAGS := -L . -lsoem -lpthread -llcm -lm #static soem library, pthread, lcm, and math


cobjects  := $(patsubst %.c, %.o, $(csrcfiles)) #name object files with .o suffix
cppobjects  := $(patsubst %.cpp, %.o, $(cppsrcfiles)) #name object files with .o suffix

.PHONY: depend clean

all: $(appname)

%.o: %.cpp	
	$(CPP)  -c $(CPPFLAGS) $(INCLUDE_FLAGS) $< -o $@
	#$(CPP) -MM $(CPPFLAGS) $(INCLUDE_FLAGS) $*.cpp > $*.d

%.o: %.c	
	$(CXX)  -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
	#$(CXX) -MM $(CXXFLAGS) $(INCLUDE_FLAGS) $*.c > $*.d


$(appname): $(cobjects) $(cppobjects)	
	echo $(csrcfiles)
	echo $(cppsrcfiles)

	$(CPP) $(CPPFLAGS) -o $(appname) $(cobjects) $(cppobjects) $(LINKER_FLAGS) $(INCLUDE_FLAGS)

clean:
	rm -f $(cobjects) $(cppobjects) *.d

	depend: $(csrcfiles) $(cppsrcfiles)
	makedepend $(INCLUDES) $^
