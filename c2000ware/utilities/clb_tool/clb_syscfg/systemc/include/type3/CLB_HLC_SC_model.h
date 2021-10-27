#ifndef CLB_HLC_H
#define CLB_HLC_H

#include <iostream>
#include "systemc.h"

SC_MODULE(TENOR_CLB_HLC)
{
  // ports
 
  sc_in<bool>  P_clk;
  sc_in<bool>  P_rst_n;
  sc_in<bool>  P_glbl_load_en; 
  sc_in< sc_uint<32> > P_glbl_load_addr;
  sc_in< sc_uint<32> > P_glbl_load_value;
  sc_in< sc_uint<32> > P_counter_in_0;
  sc_in< sc_uint<32> > P_counter_in_1;
  sc_in< sc_uint<32> > P_counter_in_2;
  sc_in< sc_uint<32> > P_pull_data_in;
  sc_in< sc_uint<32> > e0;
  sc_in< sc_uint<32> > e1;
  sc_in< sc_uint<32> > e2;
  sc_in< sc_uint<32> > e3;
  sc_in<bool> ae0;
  sc_in<bool> ae1;
  sc_in<bool> ae2;
  sc_in<bool> ae3;
  sc_in<bool>          P_EVENT_BUS_IN[32];
  sc_in< sc_uint<32> > hlc_instructions[32];

  sc_in< bool > clb_result_in [8];
  sc_in< bool > clb_async_result_in[8];
  
//Tenor addition
  sc_in< bool > RECONFIG_PIPELINE_EN;
//These are not needed from a model standpoint. 
//  sc_in< sc_uint<5> > HLC_INSTR_READ_PTR;
//  sc_out< sc_uint<12> > HLC_INSTR_READ_VALUE;
    
// These will be outputs
  sc_out< sc_uint<32> > P_counter_load_bus;
  sc_out<bool> P_local_count_load_enable_0;
  sc_out<bool> P_local_count_load_enable_1;
  sc_out<bool> P_local_count_load_enable_2;
  sc_out<bool> P_local_count_match1_enable_0;
  sc_out<bool> P_local_count_match1_enable_1;
  sc_out<bool> P_local_count_match1_enable_2;
  sc_out<bool> P_local_count_match2_enable_0;
  sc_out<bool> P_local_count_match2_enable_1;
  sc_out<bool> P_local_count_match2_enable_2;
  sc_out<bool> P_push_data_en;
  sc_out< sc_uint<32> > P_push_data_out;
  sc_out<bool> P_pull_data_en;
  sc_out<bool> P_set_intr_flag;
  sc_out< sc_uint<32> > P_comm_R0_out;
  sc_out< sc_uint<32> > P_comm_R1_out;
  sc_out< sc_uint<32> > P_comm_R2_out;
  sc_out< sc_uint<32> > P_comm_R3_out;


  ///////// Interface signals ////////////////////////
         bool rst_n;
         bool global_load_en; // This is used to set global and initial values for loads; matches etc.
         sc_uint<32> global_load_addr;
         sc_uint<32> global_load_value;
         unsigned long counter_in_0, counter_in_1,counter_in_2;
         sc_uint<32> counter_load_bus;  // Counters are treated separate compared to other signals.
         bool local_count_load_enable_0, local_count_load_enable_1, local_count_load_enable_2;
         bool local_count_match1_enable_0, local_count_match1_enable_1, local_count_match1_enable_2;
         bool local_count_match2_enable_0,local_count_match2_enable_1, local_count_match2_enable_2;
         sc_uint<32> push_data_en;
         bool pull_data_en;
         bool set_int_flag;
         sc_uint<32> push_data_out;
         unsigned long pull_data_in;
         unsigned long EVENT_MUX_SEL_0, EVENT_MUX_SEL_1, EVENT_MUX_SEL_2,EVENT_MUX_SEL_3;
         bool EVENT_MUX_ALT_0, EVENT_MUX_ALT_1, EVENT_MUX_ALT_2,EVENT_MUX_ALT_3;
         sc_uint<32> output_event_wire;
         
	 unsigned long event_in_bus;
/////////////////////////////////////////////////////////////////////

////////////// Instance specific variables ///////////////////
////////////// Instance specific variables ///////////////////
         unsigned long instr_is_add, instr_is_sub, instr_is_mov, instr_is_T1_mov, instr_is_T2_mov, instr_active_next_state, instr_is_INTR;
         unsigned long instr_opcode;     /// This is the instruction opcode.
         unsigned long dest_port, src_port;  /// Source and destination port in case it is a MOV** instruction.
         unsigned long next_instr_PTR;
         unsigned long instr_array_load,  activity_trigger ;
         unsigned long comm_source_bus, comm_write_bus, comm_operand_bus;
         unsigned long R0_write_en, R1_write_en, R2_write_en,R3_write_en, stop_bit, event_en_REG_write_en;
         unsigned long event_index_wire;
         unsigned long set_pending_event[4], clr_pending_event[4], event_pending_wire[4] ;
         unsigned long pull_data_next_cycle;

         unsigned long instr_active_T ; //// This is a one bit state indicator which indicates if we are executing an instruction or waiting for
                                  //// for an event to kick off something.
    
         unsigned long REG_event_dly[4], event_pending_REG[4], event_en_REG[4]; // We will use this to detect active high going edge of the event.
		 unsigned long clb_result_in_REG[8];  // Tenor  : Pipeline this by 1 cycle.
         unsigned long current_instr_PTR;
         unsigned long INSTR_ARRAY[64];
         unsigned long comm_R0, comm_R1, comm_R2, comm_R3;
         unsigned long dest_port_DLY;
         unsigned long dummy_state_T;


         unsigned long active_event_in[4]; 
         unsigned long event_instr_ptr;
         unsigned long next_cycle_dummy_state;

		unsigned long lcl_R0_init;
		unsigned long lcl_R1_init ;
		unsigned long lcl_R2_init ;
		unsigned long lcl_R3_init ;
//        std::string HLC_file_path;

  // process definitions
  void hlc_logic();
  void hlc_mem_init(); // Call this function every time global reset (P_rst_n) is asserted (low).
//  void HLC_load_instr_array(std::string , unsigned long ,	unsigned long ,	unsigned long ,	unsigned long);
  void HLC_load_instr_array(unsigned long ,	unsigned long ,	unsigned long ,	unsigned long);

  SC_HAS_PROCESS(TENOR_CLB_HLC);

//  POTENZA_CLB_HLC(sc_core::sc_module_name nm, std::string csv_fileName, unsigned long R0_init, unsigned long R1_init, unsigned long R2_init, unsigned long R3_init);
  TENOR_CLB_HLC(sc_core::sc_module_name nm, unsigned long R0_init, unsigned long R1_init, unsigned long R2_init, unsigned long R3_init);
};

#endif