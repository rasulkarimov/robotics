SDK := /home/astra/tools/deptrum-aurora930/sdk
CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -I$(SDK)/include
LDFLAGS := -L$(SDK)/lib -Wl,-rpath,$(SDK)/lib
LDLIBS := -ldeptrum_stream_aurora900 -lturbojpeg -lpthread

all: aurora_streamer

aurora_streamer: aurora_streamer.cc
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) $(LDLIBS)

probe: probe.cc
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) -lpthread

clean:
	rm -f aurora_streamer probe

.PHONY: all clean
