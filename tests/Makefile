LDFLAGS = -lgtest -lgtest_main -pthread
CXXFLAGS = -std=c++11

all: test

clean:
	rm test

test:
	g++ -o test test.cpp $(LDFLAGS) $(CXXFLAGS)
