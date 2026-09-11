all : split

CCC = g++
CCFLAGS = -O3
LIBS= -lm
TARGETDIR=.
CPPFLAGS += \
         -I.

        
OBJS2 = \
        $(TARGETDIR)/commandline.o \
	    $(TARGETDIR)/main.o \
        $(TARGETDIR)/Pb_Data.o \
        $(TARGETDIR)/Split_Bellman.o \
        $(TARGETDIR)/Split_Bellman_Bounded.o \
        $(TARGETDIR)/Split_Bellman_Soft.o \
        $(TARGETDIR)/Split_Linear.o \
        $(TARGETDIR)/Split_Linear_Bounded.o \
        $(TARGETDIR)/Split_Linear_Soft.o \
        $(TARGETDIR)/Split_Bellman_PTVRP.o \
        $(TARGETDIR)/Split_Linear_PTVRP.o \
        $(TARGETDIR)/Split_Layered_PTVRP.o

$(TARGETDIR)/split: $(OBJS2)
	$(CCC)  $(CCFLAGS) -o $(TARGETDIR)/split $(OBJS2)

$(TARGETDIR)/commandline.o: commandline.h commandline.cpp Pb_Data.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c commandline.cpp -o $(TARGETDIR)/commandline.o

$(TARGETDIR)/main.o: main.cpp commandline.h Pb_Data.h Split.h \
        Split_Bellman.h Split_Bellman_Soft.h Split_Bellman_Bounded.h \
        Split_Linear.h Split_Linear_Soft.h Split_Linear_Bounded.h \
        Split_Bellman_PTVRP.h Split_Linear_PTVRP.h Split_Layered_PTVRP.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c main.cpp -o $(TARGETDIR)/main.o

$(TARGETDIR)/Pb_Data.o: Pb_Data.h Pb_Data.cpp
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Pb_Data.cpp -o $(TARGETDIR)/Pb_Data.o

$(TARGETDIR)/Split_Bellman.o: Split_Bellman.h Split_Bellman.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Bellman.cpp -o $(TARGETDIR)/Split_Bellman.o

$(TARGETDIR)/Split_Bellman_Bounded.o: Split_Bellman_Bounded.h Split_Bellman_Bounded.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Bellman_Bounded.cpp -o $(TARGETDIR)/Split_Bellman_Bounded.o

$(TARGETDIR)/Split_Bellman_Soft.o: Split_Bellman_Soft.h Split_Bellman_Soft.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Bellman_Soft.cpp -o $(TARGETDIR)/Split_Bellman_Soft.o

$(TARGETDIR)/Split_Linear.o: Split_Linear.h Split_Linear.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Linear.cpp -o $(TARGETDIR)/Split_Linear.o

$(TARGETDIR)/Split_Linear_Bounded.o: Split_Linear_Bounded.h Split_Linear_Bounded.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Linear_Bounded.cpp -o $(TARGETDIR)/Split_Linear_Bounded.o

$(TARGETDIR)/Split_Linear_Soft.o: Split_Linear_Soft.h Split_Linear_Soft.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Linear_Soft.cpp -o $(TARGETDIR)/Split_Linear_Soft.o

$(TARGETDIR)/Split_Bellman_PTVRP.o: Split_Bellman_PTVRP.h Split_Bellman_PTVRP.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Bellman_PTVRP.cpp -o $(TARGETDIR)/Split_Bellman_PTVRP.o

$(TARGETDIR)/Split_Linear_PTVRP.o: Split_Linear_PTVRP.h Split_Linear_PTVRP.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Linear_PTVRP.cpp -o $(TARGETDIR)/Split_Linear_PTVRP.o

$(TARGETDIR)/Split_Layered_PTVRP.o: Split_Layered_PTVRP.h Split_Layered_PTVRP.cpp Split.h Pb_Data.h Trivial_Deque.h
	$(CCC) $(CCFLAGS) $(CPPFLAGS) -c Split_Layered_PTVRP.cpp -o $(TARGETDIR)/Split_Layered_PTVRP.o

clean:
	 $(RM) \
	 $(TARGETDIR)/split \
     $(TARGETDIR)/commandline.o \
	 $(TARGETDIR)/main.o \
     $(TARGETDIR)/Pb_Data.o \
     $(TARGETDIR)/Split_Bellman.o \
     $(TARGETDIR)/Split_Bellman_Bounded.o \
     $(TARGETDIR)/Split_Bellman_Soft.o \
     $(TARGETDIR)/Split_Linear.o \
     $(TARGETDIR)/Split_Linear_Bounded.o \
     $(TARGETDIR)/Split_Linear_Soft.o \
     $(TARGETDIR)/Split_Bellman_PTVRP.o \
     $(TARGETDIR)/Split_Linear_PTVRP.o \
     $(TARGETDIR)/Split_Layered_PTVRP.o

