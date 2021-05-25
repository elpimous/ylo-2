
CXX = $(CROSS)g++

TARGET = elpimous_test_myahrs+
 
CFLAGS   = -O2
CXXFLAGS = $(CFLAGS) -std=c++11

CPP_SRCS = $(wildcard *.cpp) 
C_SRCS   = $(wildcard *.c)

SRCS = $(CPP_SRCS) $(C_SRCS)
OBJS = $(CPP_SRCS:.cpp=.o) $(C_SRCS:.c=.o)


all : $(TARGET)

$(TARGET) : $(OBJS) myahrs_plus.hpp
	$(CXX) $(CFLAGS) -o $@ $(OBJS) -lm -lpthread 



