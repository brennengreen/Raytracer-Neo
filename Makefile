CC=g++
TARGET=raytrace
CFLAGS=-std=c++17

all: $(TARGET)

$(TARGET): main.cpp
			$(CC) $(CFLAGS) -o $(TARGET)  main.cpp

clean:
			$(RM) $(TARGET)