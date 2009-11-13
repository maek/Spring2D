LIB = Spring2D

SRCDIR = src
OBJDIR = obj
LIBDIR = lib
INCDIR = include

SOURCES = s2Vector.cc
OBJECTS = $(SOURCES:.cc=.o)

CXX = g++
CXXFLAGS = -Wall -fPIC
CXXOBJECTS = $(patsubst %,$(OBJDIR)/%,$(OBJECTS))


.PHONY: all
all: $(LIB)

$(LIB): $(CXXOBJECTS)
	$(CXX) -Wall -shared -o $(LIBDIR)/lib$@.so $^


$(OBJDIR)/s2Vector.o: $(SRCDIR)/s2Vector.cc\
 											$(INCDIR)/s2Vector.h\
										 	$(INCDIR)/s2Common.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<


.PHONY: clean
clean:
	$(RM) $(CXXOBJECTS) $(LIBDIR)/lib$(LIB).so
