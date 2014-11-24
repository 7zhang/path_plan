CXXFLAGS = -std=c++11
flags = -O2 -lboost_program_options -lpthread -lboost_system -lboost_thread -std=c++11
OBJS = robot_de_optimize.o function.o calc_criteria.o geometric.o \
	calc_state.o KunShanJacobi.o positioner.o robotkinematic.o robotdata.o \
	Transform.o load_seam.o
all: $(OBJS)
	g++ $(flags) -o path_plan $^
.c.o: 
	g++ -std=c++11 $<
clean:
	@rm -f *.o *.d *.d.* \#*\#

%.d: %.cpp
	@set -e; rm -f $@; g++ -std=c++11 -MM $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

-include $(OBJS:.o=.d)

# .PHONY:clean 
# clean:
#     rm -f $(TARGETS) *.o
