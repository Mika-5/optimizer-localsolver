SYSTEM = x86-64_linux
LIBFORMAT = static_pic
LOCALSOLVER_TOP = ../localsolver_8_0
EXDIR = ../optimizer-localsolver
PROTOBUF_TAG = 3.3.0
GLOG_TAG = 0.3.5
GFLAGS_TAG = 2.2.0
MKDIR = mkdir
UNIX_PROTOBUF_DIR = dependencies/install
CMAKE = cmake
PROTOBUF_INC = -I$(UNIX_PROTOBUF_DIR)/include
CCC = g++ -fPIC -std=c++0x -fwrapv
GFLAGS_INC = -I$(EXDIR)/dependencies/install/include -I$(LOCALSOLVER_TOP)/include 
#-I$(EXDIR)/dependencies/install/include -I$(LOCALSOLVER_TOP)/cplex/include -I$(LOCALSOLVER_TOP)/cpoptimizer/include -I$(LOCALSOLVER_TOP)/concert/include
# g++ cvrptw.cpp -I/opt/localsolver_XXX/include -llocalsolver -lpthread -o cvrp

CFLAGS = -Wno-deprecated $(PROTOBUF_INC) 
#------------------------------------------------------------
#
# When you adapt this makefile to compile your LOCALSOLVER programs
# please copy this makefile and set LOCALSOLVERDIR and CONCERTDIR to
# the directories where LOCALSOLVER and CONCERT are installed.
#
#------------------------------------------------------------
export ILOG_HOME= ../localsolver_8_0

export LD_LIBRARY_PATH=$ILOG_HOME/bin

export ILOG_LICENSE_FILE=$ILOG_HOME/license

# ILOGSTUDIODIR =/home/hb/ILOG/LOCALSOLVER_Studio_AcademicResearch122
LOCALSOLVERDIR = $(ILOG_HOME)/cplex
CONCERTDIR = $(ILOG_HOME)/concert

# ---------------------------------------------------------------------
# Compiler selection
# ---------------------------------------------------------------------

# CCC = g++
CC = gcc
JAVAC = javac

# ---------------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------------

CCOPT = -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

LOCALSOLVERBINDIR = $(LOCALSOLVERDIR)/bin/x86-64_linux
CONCERTXBINDIR = $(CONCERTDIR)/include
LOCALSOLVERLIBDIR = $(LOCALSOLVERDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNFLAGS = $(ILOG_HOME)/cpoptimizer/lib/x86-64_linux/static_pic -lcp -L $(LOCALSOLVERLIBDIR) -lilocplex -lcplex -L $(CONCERTLIBDIR) -lconcert -m64 -lm -pthread
LDFLAGS = -I$(EXDIR)/dependencies/install/include -L$(LOCALSOLVER_TOP)/include -llocalsolver -ldl



CCFLAGS = $(CCOPT) -I $(LOCALSOLVERBINDIR) -I $(CONCERTXBINDIR) -w

# include $(LOCALSOLVER_TOP)/Makefile

.PHONY: all local_clean
third_party: \
	install_protobuf \
	install_gflags \
	install_glog \

# Install GFLAGS
install_gflags: dependencies/install/include/gflags/gflags.h

CMAKE_MISSING = "cmake not found in /Applications, nor in the PATH. Install the official version, or from brew"

dependencies/install/include/gflags/gflags.h: dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake/Makefile
	cd dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake && \
	$(SET_COMPILER) make -j 4 && make install
	touch $@

dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake/Makefile: dependencies/sources/gflags-$(GFLAGS_TAG)/CMakeLists.txt
	-mkdir dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake
	cd dependencies/sources/gflags-$(GFLAGS_TAG)/build_cmake && $(SET_COMPILER) \
	$(CMAKE) -D BUILD_SHARED_LIBS=OFF \
		 -D BUILD_STATIC_LIBS=ON \
	         -D CMAKE_INSTALL_PREFIX=../../../install \
		 -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
	         ..

dependencies/sources/gflags-$(GFLAGS_TAG)/CMakeLists.txt:
	git clone -b v$(GFLAGS_TAG) https://github.com/gflags/gflags.git dependencies/sources/gflags-$(GFLAGS_TAG)


# Install GLOG.
install_glog: dependencies/install/include/glog/logging.h

dependencies/install/include/glog/logging.h: dependencies/sources/glog-$(GLOG_TAG)/build_cmake/Makefile
	cd dependencies/sources/glog-$(GLOG_TAG)/build_cmake && $(SET_COMPILER) make -j 4 && make install
	touch $@

dependencies/sources/glog-$(GLOG_TAG)/build_cmake/Makefile: dependencies/sources/glog-$(GLOG_TAG)/CMakeLists.txt install_gflags
	-$(MKDIR) dependencies/sources/glog-$(GLOG_TAG)/build_cmake
	cd dependencies/sources/glog-$(GLOG_TAG)/build_cmake && \
	  $(CMAKE) -D CMAKE_INSTALL_PREFIX=../../../install \
                   -D BUILD_SHARED_LIBS=OFF \
                   -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
                   -D CMAKE_PREFIX_PATH="$(OR_TOOLS_TOP)/dependencies/install" \
	           ..

dependencies/sources/glog-$(GLOG_TAG)/CMakeLists.txt:
	git clone -b v$(GLOG_TAG) https://github.com/google/glog.git dependencies/sources/glog-$(GLOG_TAG)


#Install PROTOBUF
install_protobuf: dependencies/install/bin/protoc

dependencies/install/bin/protoc: dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build/Makefile
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build && $(SET_COMPILER) make -j 4 && make install

dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build/Makefile: dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/CMakeLists.txt
	-$(MKDIR) dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build
	cd dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/build && \
	  $(CMAKE) -D CMAKE_INSTALL_PREFIX=../../../../install \
		   -D protobuf_BUILD_TESTS=OFF \
                   -D BUILD_SHARED_LIBS=OFF \
                   -D CMAKE_CXX_FLAGS="-fPIC $(MAC_VERSION)" \
	           ..

dependencies/sources/protobuf-$(PROTOBUF_TAG)/cmake/CMakeLists.txt:
	git clone https://github.com/google/protobuf.git dependencies/sources/protobuf-$(PROTOBUF_TAG) && cd dependencies/sources/protobuf-$(PROTOBUF_TAG) && git checkout 3d9d1a1




# all: tsp_localsolver

%.pb.cc: %.proto
	$(EXDIR)/dependencies/install/bin/protoc --cpp_out . $<

%.o: %.cc %.h
	$(CCC) $(CFLAGS) -c $< -o $@

localsolver_vrp.pb.h: localsolver_vrp.pb.cc

localsolver_result.pb.h: localsolver_result.pb.cc

tsp_localsolver.o: tsp_localsolver.cc \
	localsolver_vrp.pb.h \
	localsolver_result.pb.h \
	tsptw_data_dt.h
	$(CCC) $(CFLAGS) $(GFLAGS_INC) -c $(LDFLAGS) tsp_localsolver.cc -o tsp_localsolver.o

tsp_localsolver: tsp_localsolver.o localsolver_vrp.pb.o localsolver_result.pb.o
	$(CCC) $(CFLAGS) -g tsp_localsolver.o localsolver_vrp.pb.o localsolver_result.pb.o -lz -lrt -lpthread \
	-L$(LOCALSOLVER_TOP)/include -llocalsolver -L dependencies/install/lib -lprotobuf -L dependencies/install/lib -lgflags -lpthread \
	-o tsp_localsolver


local_clean:
	rm -f *.pb.cc *.pb.h
	rm *.o

mrproper: local_clean
	rm tsp_localsolver
