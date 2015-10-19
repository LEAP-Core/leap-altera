//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//


//
// Wrap the Bluespec-default Virtex-7 DDR3 driver with a LEAP standard DDR
// interface.
//
// The Bluespec-provided wrapper doesn't export the bare BVI wrapper.
// Instead, it is copied to this file.
//

import Clocks::*;
import DefaultValue::*;

`include "awb/provides/librl_bsv_base.bsh"
`include "awb/provides/ddr_sdram_definitions.bsh"
`include "awb/provides/fpga_components.bsh"


//        
// DDR_BANK_WIRES --
//     These are wires which are simply passed up to the toplevel,
//     where the UCF file ties them to pins.
//
typedef SDRAM_Pins_DE1 DDR_BANK_WIRES;


interface XILINX_DRAM_CONTROLLER;
    
    interface DDR_BANK_WIRES wires;
        
    interface Clock controller_clock;
    interface Reset controller_reset;
     
    // application interface
    method    Bool              init_done;
    method    Action            enqueue_address(DDR3Command command, Bit#(25) address);
    method    Action            enqueue_data(Bit#(16) data, Bit#(2) mask, Bool endBurst);
    method    Bit#(16)          dequeue_data;

    method    Action            device_temp_i(Bit#(12) i);
    method    Bit#(12)          device_temp_o;


    // Debug info
    method    Bit#(1)           cmd_rdy;
    method    Bit#(1)           enq_rdy;
    method    Bit#(1)           deq_rdy;
       
    method    Bool              dbg_wrlvl_start;
    method    Bool              dbg_wrlvl_done;
    method    Bool              dbg_wrlvl_err;
    method    Bit#(2)           dbg_rdlvl_start;
    method    Bit#(2)           dbg_rdlvl_done;
    method    Bit#(2)           dbg_rdlvl_err;

endinterface


//
// Function gives the controller a chance to approve the platform's
// configuration.
//
module checkDDRControllerConfig#(DDRControllerConfigure ddrConfig) ();
endmodule


module mkXilinxDRAMController#(Clock clk100,
                               Reset rst100,
                               Integer bankIdx)
    // Interface:
    (XILINX_DRAM_CONTROLLER);

    Clock user_clock    <- exposeCurrentClock;
    Reset user_reset_n  <- exposeCurrentReset;

    // Instantiate the Bluespec-standard DDR3 controller
    let sdramCtrl <- vMkDE1SDRAMController(clocked_by user_clock,
                                           reset_by user_reset_n);

    PulseWire pwAppWdfWren <- mkPulseWire(clocked_by user_clock, reset_by user_reset_n);
    PulseWire pwAppWdfEnd  <- mkPulseWire(clocked_by user_clock, reset_by user_reset_n);

    Wire#(Bit#(1))     userRD_n      <- mkDWire(1, clocked_by user_clock, reset_by user_reset_n);
    Wire#(Bit#(1))     userWR_n      <- mkDWire(1, clocked_by user_clock, reset_by user_reset_n);
    Wire#(Bit#(25))    userAddr      <- mkDWire(0, clocked_by user_clock, reset_by user_reset_n);
    Wire#(Bit#(2))     userBE_n      <- mkDWire(0, clocked_by user_clock, reset_by user_reset_n);
    Wire#(Bit#(16))    userWriteData <- mkDWire(0, clocked_by user_clock, reset_by user_reset_n);

    messageM("foo");

    (* fire_when_enabled, no_implicit_conditions *)
    rule drive_enables;
        sdramCtrl.user.user_rd_n(userRD_n);
        sdramCtrl.user.user_wr_n(userWR_n);
    endrule

    (* fire_when_enabled, no_implicit_conditions *)
    rule drive_data_signals;
        sdramCtrl.user.user_addr(userAddr);
        sdramCtrl.user.user_write_data(userWriteData);
        sdramCtrl.user.user_be_n(userBE_n);
    endrule
   

    // Exposed top-level wires
    interface DDR_BANK_WIRES wires = sdramCtrl.sdram;
        
    interface Clock controller_clock = user_clock;
    interface Reset controller_reset = user_reset_n;
     
    // maybe??
    method Bool init_done() = sdramCtrl.user.user_waitrequest;

    method Action enqueue_address(DDR3Command command, Bit#(25) address) if (!sdramCtrl.user.user_waitrequest);
        // Signal that command is being sent
        userWR_n <= pack(command == WRITE);
        userRD_n <= pack(command == READ);
        userAddr <= address;
    endmethod

    method Action enqueue_data(Bit#(16) data, Bit#(2) mask, Bool endBurst) if (!sdramCtrl.user.user_waitrequest);
        // Signal that write data is being sent
        userWriteData <= data;
        userBE_n <= mask;
    endmethod

    method Bit#(16) dequeue_data() if (sdramCtrl.user.user_read_data_valid);
        return sdramCtrl.user.user_read_data;
    endmethod

    //
    // Debug info is not currently exported by the standard Bluespec driver.
    //
    method Bit#(1) cmd_rdy = error("DE1 DDR debug not implemented");
    method Bit#(1) enq_rdy = error("DE1 DDR debug not implemented");
    method Bit#(1) deq_rdy = error("DE1 DDR debug not implemented");
       
    method Bool dbg_wrlvl_start = error("DE1 DDR debug not implemented");
    method Bool dbg_wrlvl_done = error("DE1 DDR debug not implemented");
    method Bool dbg_wrlvl_err = error("DE1 DDR debug not implemented");
    method Bit#(2) dbg_rdlvl_start = error("DE1 DDR debug not implemented");
    method Bit#(2) dbg_rdlvl_done = error("DE1 DDR debug not implemented");
    method Bit#(2) dbg_rdlvl_err = error("DE1 DDR debug not implemented");
endmodule



//
// This is just a local copy of the standard Bluespec Virtex-7 DDR3
// controller wrapper.  It is here only because Bluespec doesn't export
// the wrapper from the XilinxVirtex7DDR3 package.
//

////////////////////////////////////////////////////////////////////////////////
/// Types
////////////////////////////////////////////////////////////////////////////////
typedef struct {
   Bit#(2)    byteen;
   Bit#(25)   address;
   Bit#(16)   data;
} DDR3Request deriving (Bits, Eq);

typedef struct {
   Bit#(16)   data;
} DDR3Response deriving (Bits, Eq);	       

typedef enum {
   WRITE     = 0,
   READ      = 1
} DDR3Command deriving (Eq);

instance Bits#(DDR3Command, 3);
   function Bit#(3) pack(DDR3Command x);
      case(x) 
	 WRITE:   return 0;
	 READ:    return 1;
      endcase
   endfunction
   
   function DDR3Command unpack(Bit#(3) x);
      DDR3Command cmd;
      case(x)
	 0:       cmd = WRITE;
	 1:       cmd = READ;
	 default: cmd = READ;
      endcase
      return cmd;
   endfunction
endinstance

typedef struct {
   Bool               fast_train_sim_only;
   Integer            num_reads_in_flight;
} DDR3_Configure_V7;

instance DefaultValue#(DDR3_Configure_V7);
   defaultValue = DDR3_Configure_V7 {
      fast_train_sim_only:    False,
      num_reads_in_flight:    2
      };
endinstance

////////////////////////////////////////////////////////////////////////////////
/// Interfaces
////////////////////////////////////////////////////////////////////////////////
(* always_enabled, always_ready *)
interface SDRAM_Pins_DE1;
   (* prefix = "", result = "DRAM_ADDR" *)
   method    Bit#(13)          sdram_addr;
   (* prefix = "", result = "DRAM_BA" *)
   method    Bit#(2)           sdram_ba;
   (* prefix = "", result = "DRAM_RAS_N" *)
   method    Bit#(1)           sdram_ras_n;
   (* prefix = "", result = "DRAM_CAS_N" *)
   method    Bit#(1)           sdram_cas_n;
   (* prefix = "", result = "DRAM_WE_N" *)
   method    Bit#(1)           sdram_we_n;
   (* prefix = "", result = "DRAM_CS_N" *)
   method    Bit#(1)           sdram_cs_n;
   (* prefix = "", result = "DRAM_CKE" *)
   method    Bit#(1)           sdram_cke;
   (* prefix = "", result = "DRAM_CLK" *)
   method    Bit#(1)           sdram_clk;
   (* prefix = "", result = "DRAM_DQM" *)
   method    Bit#(2)           sdram_dqm;
   (* prefix = "DRAM_DQ" *)
   interface Inout#(Bit#(16))  dq;


endinterface   

interface SDRAM_User_DE1;
   interface Clock             	     clock;
   interface Reset             	     reset_n;
   method    Bool              	     init_done;
   method    Action                  request(Bit#(25) addr, Bit#(2) mask, Bit#(16) data);
   method    ActionValue#(Bit#(16))  read_data;

endinterface

interface SDRAM_Controller_DE1;
   (* prefix = "" *)
   interface SDRAM_Pins_DE1    sdram;
   (* prefix = "" *)
   interface SDRAM_User_DE1    user;
endinterface

(* always_ready, always_enabled *)
interface VSDRAM_User_DE1;
   method    Bool              user_waitrequest;
   method    Action            user_addr(Bit#(25) i);
   method    Action            user_rd_n(Bit#(1) i);
   method    Action            user_wr_n(Bit#(1) i);
   method    Action            user_write_data(Bit#(16) i);
   method    Action            user_be_n(Bit#(2) i);
   method    Bit#(16)          user_read_data;
   method    Bool              user_read_data_valid;
endinterface

interface VSDRAM_Controller_DE1;
   (* prefix = "" *)
   interface SDRAM_Pins_DE1     sdram;
   (* prefix = "" *)
   interface VSDRAM_User_DE1     user;
endinterface   

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
/// Implementation
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
import "BVI" de1_sdram =
module vMkDE1SDRAMController (VSDRAM_Controller_DE1);
   default_clock clk(clk);
   default_reset reset_n(reset_n);
      
   interface SDRAM_Pins_DE1 sdram;
      ifc_inout            dq(zs_dq)        clocked_by(no_clock)  reset_by(no_reset);
      method      zs_cke   sdram_cke     clocked_by(no_clock)  reset_by(no_reset);
      method      zs_clk   sdram_clk     clocked_by(no_clock)  reset_by(no_reset);
      method      zs_cs_n  sdram_cs_n    clocked_by(no_clock)  reset_by(no_reset);
      method      zs_ras_n sdram_ras_n   clocked_by(no_clock)  reset_by(no_reset);
      method      zs_cas_n sdram_cas_n   clocked_by(no_clock)  reset_by(no_reset);
      method      zs_we_n  sdram_we_n    clocked_by(no_clock)  reset_by(no_reset);
      method      zs_dqm   sdram_dqm     clocked_by(no_clock)  reset_by(no_reset);
      method      zs_ba    sdram_ba      clocked_by(no_clock)  reset_by(no_reset);
      method      zs_addr  sdram_addr    clocked_by(no_clock)  reset_by(no_reset);
   endinterface
   
   interface VSDRAM_User_DE1 user;
      method          		      user_addr(az_addr)        enable((*inhigh*)en0) clocked_by(clk) reset_by(no_reset);
      method                          user_rd_n(az_rd_n)        enable((*inhigh*)en01) clocked_by(clk) reset_by(no_reset);
      method                          user_wr_n(az_wr_n)        enable((*inhigh*)en00) clocked_by(clk) reset_by(no_reset);
      method          		      user_write_data(az_data)  enable((*inhigh*)en2) clocked_by(clk) reset_by(no_reset);
      method          		      user_be_n(az_be_n)        enable((*inhigh*)en4) clocked_by(clk) reset_by(no_reset);
      method za_data                  user_read_data            clocked_by(clk) reset_by(no_reset);
      method za_valid                 user_read_data_valid      clocked_by(clk) reset_by(no_reset);
      method za_waitrequest           user_waitrequest          clocked_by(clk) reset_by(no_reset);
   endinterface
   
   schedule
   (
    sdram_sdram_cke, sdram_sdram_clk, sdram_sdram_cs_n, sdram_sdram_ras_n, sdram_sdram_cas_n, sdram_sdram_we_n, 
    sdram_sdram_dqm, sdram_sdram_ba, sdram_sdram_addr, user_user_waitrequest
    )
   CF
   (
    sdram_sdram_cke, sdram_sdram_clk, sdram_sdram_cs_n, sdram_sdram_ras_n, sdram_sdram_cas_n, sdram_sdram_we_n, 
    sdram_sdram_dqm, sdram_sdram_ba, sdram_sdram_addr, user_user_waitrequest
    );
   
   // Some of these are not really conflict free. 
   schedule 
   (

   user_user_waitrequest, user_user_addr,user_user_rd_n, user_user_wr_n, user_user_read_data, user_user_be_n, user_user_write_data, user_user_read_data_valid 
   )
   CF (
 user_user_waitrequest, user_user_addr, user_user_rd_n, user_user_wr_n, user_user_read_data, user_user_be_n, user_user_write_data, user_user_read_data_valid 
    );

endmodule
