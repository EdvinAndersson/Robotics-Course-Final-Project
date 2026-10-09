#all: 
#	g++ -std=c++11 -o a.out ina219.so main.cpp lidar.cpp scan_match.cpp odometry.cpp -lrobotic_gcc -lwiringPi -lpthread -I /usr/include/eigen3

# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++11 -I /usr/include/eigen3
LDFLAGS := -lrobotic_gcc -lwiringPi -lpthread `pkg-config --libs opencv`

# Sources and target
SRC := main.cpp lidar.cpp scan_match.cpp odometry.cpp camera.cpp servo.cpp
TARGET := a.out
SHARED_LIBS := ina219.so

# Object files
OBJ := $(SRC:.cpp=.o)

# Default target
all: $(TARGET)

# Link object files and shared libraries into executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(SHARED_LIBS) $(LDFLAGS)

# Compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean