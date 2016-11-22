CC=clang++-3.5 #Compiler

CFLAGS+=`pkg-config --cflags opencv` -c -std=c++11   -fPIE #-Ofast #Compiler Flags

DEFINES=-DENABLE_DELETE

INCPATH=

LDFLAGS+=`pkg-config --libs opencv` #Linker options

SOURCES= 	main.cpp

OBJECTS=$(SOURCES:.cpp=.o)  #Object files

EXECUTEABLE=tracker #Output name

all: $(HEADERS) $(SOURCES) $(EXECUTEABLE)

$(EXECUTEABLE): $(OBJECTS)
	$(CC)    $(OBJECTS) -o $(EXECUTEABLE) $(LDFLAGS)


.cpp.o:
	$(CC)  $(CFLAGS) $(INCPATH) $(DEFINES)   $< -o $@

clean:  ; rm *.o $(EXECUTEABLE) $(MOCFILES) $(HEADERS)
