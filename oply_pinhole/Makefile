#
# Makefile
#
.SUFFIXES:
.SUFFIXES: .cpp .c .h .o

.cpp.o:
	$(COMPILE) $<

.c.o:
	$(CCOMPILE) $<

RPLYDIR  = rply-1.1.4
VPATH    = $(RPLYDIR)

DEFS     =
INCLUDES = -I/usr/include/opencv2 -I$(RPLYDIR)
LIBS     = -lopencv_highgui -lopencv_core -lm
LDFLAGS  = 
#CXXFLAGS = -Wall -ggdb -O0
#CFLAGS   = -Wall -ggdb -O0
CXXFLAGS = -Wall -O2 -DNDEBUG
CFLAGS   = -Wall -O2 -DNDEBUG

CXX	 = g++
LINKER	 = g++
CC       = gcc

COMPILE  = $(CXX) -c $(DEFS) $(INCLUDES) $(CXXFLAGS)
CCOMPILE = $(CC) -c $(DEFS) $(INCLUDES) $(CFLAGS)
LINK	 = $(LINKER) $(LDFLAGS) -o $@

PROGRAM  = oply_pinhole

SRCS     = oply_pinhole.cpp \
           OrderedPly.cpp \
           rply.c

OBJS     = oply_pinhole.o \
           OrderedPly.o \
           rply.o

all:: $(PROGRAM)

$(PROGRAM) : $(OBJS)
	$(LINK) $(OBJS) $(LIBS)

clean:
	rm -f $(OBJS) $(PROGRAM)

distclean: clean
	rm -f *.h~ *.cpp~ Makefile~

