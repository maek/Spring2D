LIB = Spring2D

SRCDIR = src
OBJDIR = obj
LIBDIR = lib
INCDIR = include

OBJECTS = s2Engine.o\
					s2Body.o\
				 	s2Environment.o\
				 	s2Vector2.o

CXX = g++
CXXFLAGS = -Wall -fPIC
CXXOBJECTS = $(patsubst %,$(OBJDIR)/%,$(OBJECTS))


.PHONY: all
all: $(LIB)


$(LIB): $(CXXOBJECTS)
	$(CXX) -Wall -shared -o $(LIBDIR)/lib$@.so $^


$(OBJDIR)/s2Engine.o:							$(SRCDIR)/s2Engine.cc\
																	$(INCDIR)/s2Engine.h\
																	$(INCDIR)/s2Settings.h\
																	$(INCDIR)/s2Environment.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Body.o:								$(SRCDIR)/s2Body.cc\
																	$(INCDIR)/s2Body.h\
																	$(INCDIR)/s2Settings.h\
																	$(INCDIR)/s2Vector2.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Environment.o:				$(SRCDIR)/s2Environment.cc\
																	$(INCDIR)/s2Environment.h\
																	$(INCDIR)/s2Settings.h\
																	$(INCDIR)/s2Body.h\
																	$(INCDIR)/s2Vector2.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Vector2.o:							$(SRCDIR)/s2Vector2.cc\
																	$(INCDIR)/s2Vector2.h\
																	$(INCDIR)/s2Settings.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<


.PHONY: clean
clean:
	@$(RM) -v $(OBJDIR)/*.o $(LIBDIR)/*.so
