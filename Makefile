LIB = Spring2D

SRCDIR = src
OBJDIR = obj
LIBDIR = lib
INCDIR = include

OBJECTS = s2Engine.o s2Vector.o

CXX = g++
CXXFLAGS = -Wall -fPIC
CXXOBJECTS = $(patsubst %,$(OBJDIR)/%,$(OBJECTS))


.PHONY: all
all: $(LIB)

$(LIB): $(CXXOBJECTS)
	$(CXX) -Wall -shared -o $(LIBDIR)/lib$@.so $^


$(OBJDIR)/s2Vector.o: $(SRCDIR)/s2Vector.cc\
											$(INCDIR)/s2Vector.h\
											$(INCDIR)/s2Settings.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Engine.o: $(SRCDIR)/s2Engine.cc\
											$(INCDIR)/s2Engine.h\
											$(INCDIR)/s2Settings.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<


.PHONY: clean
clean:
	$(RM) $(CXXOBJECTS) $(LIBDIR)/lib$(LIB).so
