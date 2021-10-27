#include "stdint.h"
#include "systemc.h"
SC_MODULE(squareWave)
{
    sc_in<bool> clk;
    sc_out<bool> out;
    uint32_t period;
    uint32_t duty_cycle;
	uint32_t repeat_count;
	uint32_t static_value;

    void generator(void)
    {
		uint32_t localcount=0;
        if(period == 0){
			out.write(static_value);
		}
		else {
			
			while (1) {
				if(localcount==repeat_count) {
					out.write(false);
					break;
				}
				out.write(false);
				wait(period - duty_cycle);
				out.write(true);
				wait(duty_cycle);
				localcount++;
			}
		}
    }

    typedef squareWave SC_CURRENT_USER_MODULE;
    squareWave(sc_module_name name_, uint32_t period_, uint32_t duty_cycle_, uint32_t repeat_count_, uint32_t static_value_) :
        period(period_), duty_cycle(duty_cycle_), repeat_count(repeat_count_), static_value(static_value_)
    {
        SC_THREAD(generator);
        sensitive << clk.pos();
    }
};
