LIB = Spring2D

SRCDIR = src
OBJDIR = obj
LIBDIR = lib
INCDIR = include

OBJECTS = s2Engine.o\
				 	s2Math.o\
					s2Body.o\
					s2Environment.o\
					s2AABB.o\
					s2BroadPhaseDetector.o\
					s2Simplex.o\
					s2NarrowPhaseDetector.o\
					s2CollisionDetector.o


CXX = g++
CXXFLAGS = -Wall -fPIC
CXXOBJECTS = $(patsubst %,$(OBJDIR)/%,$(OBJECTS))


.PHONY: all
all: $(LIB)


$(LIB): $(CXXOBJECTS)
	$(CXX) -Wall -shared -o $(LIBDIR)/lib$@.so $^


$(OBJDIR)/s2Engine.o:								$(SRCDIR)/s2Engine.cc\
																		$(INCDIR)/s2Engine.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Environment.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Math.o:									$(SRCDIR)/s2Math.cc\
																		$(INCDIR)/s2Math.h\
																		$(INCDIR)/s2Settings.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Body.o:									$(SRCDIR)/s2Body.cc\
																		$(INCDIR)/s2Body.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Environment.o:					$(SRCDIR)/s2Environment.cc\
																		$(INCDIR)/s2Environment.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h\
																		$(INCDIR)/s2Body.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2AABB.o:									$(SRCDIR)/s2AABB.cc\
																		$(INCDIR)/s2AABB.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2BroadPhaseDetector.o:		$(SRCDIR)/s2BroadPhaseDetector.cc\
																		$(INCDIR)/s2BroadPhaseDetector.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2Simplex.o:							$(SRCDIR)/s2Simplex.cc\
																		$(INCDIR)/s2Simplex.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2NarrowPhaseDetector.o:	$(SRCDIR)/s2NarrowPhaseDetector.cc\
																		$(INCDIR)/s2NarrowPhaseDetector.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(OBJDIR)/s2CollisionDetector.o:		$(SRCDIR)/s2CollisionDetector.cc\
																		$(INCDIR)/s2CollisionDetector.h\
																		$(INCDIR)/s2Settings.h\
																		$(INCDIR)/s2Math.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<


.PHONY: clean
clean:
	@$(RM) -v $(OBJDIR)/*.o $(LIBDIR)/*.so


.PHONY: todo
todo:
	@grep -n --color 'TODO' $(INCDIR)/* $(SRCDIR)/*
