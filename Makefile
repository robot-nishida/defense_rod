TARGET = name

CXXFLAGS = -O2 -g -Wall

LIBS = -lboost_thread -lboost_filesystem ¥
       -I/usr/include/octave-`octave-config -v` ¥
       -L/usr/lib/octave-`octave-config -v` -loctave -lcruft ¥
       -I/usr/local/include -L/usr/local/lib -lode -ldrawstuff -lglut -l3ds

SRCS = $(shell ls *.cpp)
OBJS = $(SRCS:.cpp=.o)
HEADS = $(shell ls *.hpp)

$(TARGET): $(OBJS) $(HEADS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all: $(TARGET)

run: all
	./$(TARGET)

depend:
	$(CXX) -MM -MG $(SRCS) > Makefile.depend
	cat Makefile.depend

clean:
	rm -f $(OBJS) $(TARGET) *~ ¥#*¥#