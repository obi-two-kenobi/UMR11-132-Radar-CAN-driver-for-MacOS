# Variables
CXX := /usr/bin/clang++
CXXFLAGS := -std=gnu++14 -fcolor-diagnostics -fansi-escape-codes
LDFLAGS := -lpthread -Wl,-framework -Wl,IOKit -Wl,-framework -Wl,CoreFoundation
DEFINES := -DCHANNEL=0 -DBAUDRATE=CANBTR_INDEX_500K
INCLUDES := -IIncludes
LIBS := Binaries/libKvaserCAN.a `pkg-config --cflags --libs opencv4`
TARGET := umr11
SRC := main.cpp UMR11132.cpp

# Build target
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) $(SRC) -o $(TARGET) $(LIBS) $(LDFLAGS)

# Clean target
.PHONY: clean
clean:
	rm -f $(TARGET)