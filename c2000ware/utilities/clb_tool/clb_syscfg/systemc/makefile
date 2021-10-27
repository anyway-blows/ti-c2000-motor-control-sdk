INCLUDES  = -Iinclude

FILES  = src/Soprano_CLB_counter_SC_model.cpp
FILES += src/Soprano_CLB_LUT4_SC_model.cpp
FILES += src/Soprano_CLB_OutputLUT_SC_model.cpp
FILES += src/Soprano_CLB_FSM_SC_model.cpp
FILES += src/Soprano_CLB_HLC_SC_model.cpp

OBJS =  $(FILES:.cpp=.o)

clb.lib: $(OBJS)
	ar -r $@ $^

%.o:%.cpp
	g++ -c $(INCLUDES) $< -o $@

clean:
	rm -rf */*.o clb.lib
