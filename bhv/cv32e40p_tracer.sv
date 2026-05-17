// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Engineer:       Andreas Traber - atraber@iis.ee.ethz.ch
// Additional contributions by: Davide Schiavone - pschiavo@iis.ee.ethz.ch
// Design Name:    RISC-V Tracer
// Project Name:   RI5CY
// Language:       SystemVerilog
// Description:    Traces the executed instructions
//

`ifdef CV32E40P_TRACE_EXECUTION

`include "uvm_macros.svh"

module cv32e40p_tracer
  import cv32e40p_pkg::*;
  import uvm_pkg::*;
#(
    parameter FPU   = 0,
    parameter ZFINX = 0
) (
    input logic clk_i,
    input logic rst_n,

    input logic [31:0] hart_id_i,

    input logic [31:0] pc,
    input logic [31:0] instr,
    input ctrl_state_e controller_state_i,

    input logic compressed,
    input logic id_valid,
    input logic is_decoding,
    input logic is_illegal,
    input logic trigger_match,

    input logic [31:0] rs1_value,
    input logic [31:0] rs2_value,
    input logic [31:0] rs3_value,

    input logic [31:0] rs2_value_vec,

    input logic rd_is_fp,
    input logic rs1_is_fp,
    input logic rs2_is_fp,
    input logic rs3_is_fp,

    input logic        ex_valid,
    input logic [ 5:0] ex_reg_addr,
    input logic        ex_reg_we,
    input logic [31:0] ex_reg_wdata,

    input logic        ex_data_req,
    input logic        ex_data_gnt,
    input logic        ex_data_we,
    input logic [31:0] ex_data_addr,
    input logic [31:0] ex_data_wdata,
    input logic        data_misaligned,

    input logic ebrk_insn,
    input logic debug_mode,
    input logic ebrk_force_debug_mode,

    input logic wb_bypass,

    input logic        wb_valid,
    input logic [ 5:0] wb_reg_addr,
    input logic        wb_reg_we,
    input logic [31:0] wb_reg_wdata,

    input logic [31:0] imm_u_type,
    input logic [31:0] imm_uj_type,
    input logic [31:0] imm_i_type,
    input logic [11:0] imm_iz_type,
    input logic [31:0] imm_z_type,
    input logic [31:0] imm_s_type,
    input logic [31:0] imm_sb_type,
    input logic [31:0] imm_s2_type,
    input logic [31:0] imm_s3_type,
    input logic [31:0] imm_vs_type,
    input logic [31:0] imm_vu_type,
    input logic [31:0] imm_shuffle_type,
    input logic [ 4:0] imm_clip_type,

    input logic apu_en_i,
    input logic apu_singlecycle_i,
    input logic apu_multicycle_i,
    input logic apu_rvalid_i
);

  import cv32e40p_tracer_pkg::*;

  integer f;
  string  fn;
  integer cycles;

  logic [5:0] rd, rs1, rs2, rs3, rs4;

  `include "cv32e40p_instr_trace.svh"

  string info_tag;

  // Pipeline stage handles
  instr_trace_t trace_ex;
  instr_trace_t trace_ex_delay;
  instr_trace_t trace_wb;
  instr_trace_t trace_wb_delay;

  // Shadow null-flags: kept in sync with handles so combinational
  // always @* blocks never need to compare handles against null directly.
  bit trace_ex_is_null       = 1;
  bit trace_ex_delay_is_null = 1;
  bit trace_wb_is_null       = 1;
  bit trace_wb_delay_is_null = 1;

  // Pipeline control strobes (driven by combinational always @*)
  bit trace_new              = 0;
  bit trace_new_ebreak       = 0;
  bit trace_ex_misaligned    = 0;
  bit trace_ex_retire        = 0;
  bit trace_ex_wb_bypass     = 0;
  bit trace_wb_retire        = 0;
  bit trace_wb_delay_retire  = 0;

  bit clear_trace_ex                  = 0;
  bit move_trace_ex_to_trace_wb       = 0;
  bit clear_trace_wb                  = 0;
  bit move_trace_wb_to_trace_wb_delay = 0;
  bit clear_trace_wb_delay            = 0;
  bit move_trace_ex_to_trace_ex_delay = 0;
  bit clear_trace_ex_delay            = 0;
  bit move_trace_ex_delay_to_trace_wb = 0;

  bit trace_wb_is_delay_instr = 0;

  // -------------------------------------------------------
  // Cycle counter
  // -------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) cycles <= 0;
    else        cycles <= cycles + 1;
  end

  // -------------------------------------------------------
  // File open
  // -------------------------------------------------------
  initial begin
    wait(rst_n == 1'b1);
    $sformat(fn, "trace_core_%h.log", hart_id_i);
    $sformat(info_tag, "CORE_TRACER %2d", hart_id_i);
    $display("[%s] Output filename is: %s", info_tag, fn);
    f = $fopen(fn, "w");
    $fwrite(f, "            Time           Cycle PC       Instr    Ctx Decoded instruction Register and memory contents\n");
  end

  // -------------------------------------------------------
  // Port assignments
  // -------------------------------------------------------
  assign rd  = {rd_is_fp,  instr[11:07]};
  assign rs1 = {rs1_is_fp, instr[19:15]};
  assign rs2 = {rs2_is_fp, instr[24:20]};
  assign rs3 = {rs3_is_fp, instr[29:25]};
  assign rs4 = {rs3_is_fp, instr[31:27]};

  // -------------------------------------------------------
  // Helper functions
  // -------------------------------------------------------

  function void do_print(instr_trace_t t);
    if (t == null) return;
    t.printInstrTrace();
  endfunction : do_print

  function void apply_reg_write(instr_trace_t trace, int unsigned reg_addr, int unsigned wdata);
    if (trace == null) return;
    foreach (trace.regs_write[i]) begin
      if (trace.regs_write[i].addr == reg_addr) begin
        trace.regs_write[i].value = wdata;
        `uvm_info(info_tag, $sformatf(
            "Write mapped %0d, %0d:0x%08x pc:0x%08x", i, reg_addr, wdata, trace.pc), UVM_DEBUG)
      end else begin
        `uvm_info(info_tag, $sformatf(
            "Unmapped write to %0d:0x%08x, expected write to %0d",
            reg_addr, wdata, trace.regs_write[i].addr), UVM_DEBUG)
      end
    end
  endfunction : apply_reg_write

  function void apply_mem_access(instr_trace_t trace, bit we, int unsigned addr,
                                 int unsigned wdata);
    mem_acc_t mem_acc;
    if (trace == null) return;
    mem_acc.addr  = addr;
    mem_acc.we    = we;
    mem_acc.wdata = we ? wdata : 'x;
    trace.mem_access.push_back(mem_acc);
  endfunction : apply_mem_access

  function instr_trace_t trace_new_instr();
    instr_trace_t trace;
    trace = new();
    trace.init(.cycles(cycles), .pc(pc), .compressed(compressed), .instr(instr));
    return trace;
  endfunction : trace_new_instr

  function bit is_wb_delay_instr(instr_trace_t t);
    if (t == null) return 0;
    return (t.str == "mret" || t.str == "uret" ||
            t.str == "ebreak" || t.str == "c.ebreak") ? 1 : 0;
  endfunction : is_wb_delay_instr

  // Update WB delay flag whenever trace_wb changes
  always @(trace_wb)
    trace_wb_is_delay_instr = (!trace_wb_is_null) ? is_wb_delay_instr(trace_wb) : 0;

  // -------------------------------------------------------
  // Combinational pipeline control
  // -------------------------------------------------------
  always @* begin
    trace_new              = 0;
    trace_new_ebreak       = 0;
    trace_ex_misaligned    = 0;
    trace_ex_retire        = 0;
    trace_ex_wb_bypass     = 0;
    trace_wb_retire        = 0;
    trace_wb_delay_retire  = 0;

    clear_trace_ex                  = 0;
    move_trace_ex_to_trace_wb       = 0;
    clear_trace_wb                  = 0;
    move_trace_wb_to_trace_wb_delay = 0;
    clear_trace_wb_delay            = 0;
    move_trace_ex_to_trace_ex_delay = 0;
    move_trace_ex_delay_to_trace_wb = 0;
    clear_trace_ex_delay            = 0;

    // WB Delay: always retire and clear
    if (!trace_wb_delay_is_null) begin
      trace_wb_delay_retire = 1;
      clear_trace_wb_delay  = 1;
    end

    // WB: retire or move to WB delay
    if (!trace_wb_is_null) begin
      if (wb_valid) begin
        if (trace_wb_is_delay_instr) begin
          move_trace_wb_to_trace_wb_delay = 1;
        end else begin
          trace_wb_retire = 1;
          clear_trace_wb  = 1;
        end
      end
    end

    // EX delay: advance to WB when result is ready
    if (!trace_ex_delay_is_null) begin
      if (apu_rvalid_i || !trace_ex_delay.is_apu) begin
        move_trace_ex_delay_to_trace_wb = 1;
        clear_trace_ex_delay            = 1;
      end
    end

    // EX: decode new instructions and advance pipeline
    if (id_valid && is_decoding && !is_illegal) begin
      trace_new = 1;
    end else if (is_decoding && !trigger_match && ebrk_insn &&
                 (ebrk_force_debug_mode || debug_mode)) begin
      trace_new_ebreak = 1;
    end

    if (!trace_ex_is_null && ex_valid && data_misaligned) begin
      trace_ex_misaligned = 1;
    end else if (wb_bypass) begin
      trace_ex_retire    = 1;
      trace_ex_wb_bypass = 1;
      clear_trace_ex     = 1;
    end else if (!trace_ex_is_null && apu_en_i && !apu_rvalid_i) begin
      move_trace_ex_to_trace_ex_delay = 1;
      clear_trace_ex                  = 1;
    end else if (!trace_ex_is_null && ex_valid && !data_misaligned) begin
      if (move_trace_ex_delay_to_trace_wb) begin
        move_trace_ex_to_trace_ex_delay = 1;
        clear_trace_ex                  = 1;
      end else begin
        move_trace_ex_to_trace_wb = 1;
        clear_trace_ex            = 1;
      end
    end
  end

  // -------------------------------------------------------
  // EX stage
  // -------------------------------------------------------
  always @(negedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      trace_ex         <= null;
      trace_ex_is_null <= 1;
    end else begin
      if (trace_ex_retire     && !trace_ex_is_null) trace_ex.retire     = 1;
      if (trace_ex_wb_bypass  && !trace_ex_is_null) trace_ex.wb_bypass  = 1;
      if (trace_ex_misaligned && !trace_ex_is_null) trace_ex.misaligned = 1;

      // WB bypass: print immediately, instruction skips WB
      if (trace_ex_retire && trace_ex_wb_bypass && !trace_ex_is_null)
        do_print(trace_ex);

      if (trace_new_ebreak) begin
        instr_trace_t new_instr;
        new_instr        = trace_new_instr();
        new_instr.ebreak = 1;
        new_instr.retire = 1;
        // ebreak waits for debug_mode; cannot print inline here.
        // Store it in trace_wb so the WB stage prints it when
        // debug_mode is observed (handled in WB block below).
        trace_wb         <= new_instr;
        trace_wb_is_null <= 0;
      end

      if (trace_new) begin
        instr_trace_t new_instr;
        new_instr = trace_new_instr();
        trace_ex         <= new_instr;
        trace_ex_is_null <= 0;
      end else if (clear_trace_ex) begin
        trace_ex         <= null;
        trace_ex_is_null <= 1;
      end
    end
  end

  // -------------------------------------------------------
  // EX delay stage
  // -------------------------------------------------------
  always @(negedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      trace_ex_delay         <= null;
      trace_ex_delay_is_null <= 1;
    end else begin
      if (move_trace_ex_to_trace_ex_delay) begin
        trace_ex_delay         <= trace_ex;
        trace_ex_delay_is_null <= trace_ex_is_null;
      end else if (clear_trace_ex_delay) begin
        trace_ex_delay         <= null;
        trace_ex_delay_is_null <= 1;
      end
    end
  end

  // -------------------------------------------------------
  // WB stage
  // Print happens here for normal instructions and ebreak.
  // -------------------------------------------------------
  always @(negedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      trace_wb         <= null;
      trace_wb_is_null <= 1;
    end else begin
      // Retire: print the instruction now
      if (trace_wb_retire && !trace_wb_is_null && trace_wb != null) begin
        do_print(trace_wb);
      end

      // ebreak: wait for debug_mode before printing
      if (!trace_wb_is_null && trace_wb != null &&
          trace_wb.ebreak && debug_mode) begin
        do_print(trace_wb);
        trace_wb         <= null;
        trace_wb_is_null <= 1;
      end else if (move_trace_ex_to_trace_wb) begin
        trace_wb         <= trace_ex;
        trace_wb_is_null <= trace_ex_is_null;
      end else if (move_trace_ex_delay_to_trace_wb) begin
        trace_wb         <= trace_ex_delay;
        trace_wb_is_null <= trace_ex_delay_is_null;
      end else if (clear_trace_wb) begin
        trace_wb         <= null;
        trace_wb_is_null <= 1;
      end
    end
  end

  // -------------------------------------------------------
  // WB delay stage
  // -------------------------------------------------------
  always @(negedge clk_i or negedge rst_n) begin
    if (!rst_n) begin
      trace_wb_delay         <= null;
      trace_wb_delay_is_null <= 1;
    end else begin
      // Retire: print on the extra delay cycle
      if (trace_wb_delay_retire && !trace_wb_delay_is_null && trace_wb_delay != null)
        do_print(trace_wb_delay);

      if (move_trace_wb_to_trace_wb_delay) begin
        trace_wb_delay         <= trace_wb;
        trace_wb_delay_is_null <= trace_wb_is_null;
      end else if (clear_trace_wb_delay) begin
        trace_wb_delay         <= null;
        trace_wb_delay_is_null <= 1;
      end
    end
  end

  // -------------------------------------------------------
  // Register writeback and memory access monitors
  // -------------------------------------------------------
  always @(negedge clk_i or negedge rst_n) begin
    if (rst_n) begin

      // EX register write
      if (ex_reg_we && (ex_valid || !wb_valid || apu_rvalid_i)) begin
        `uvm_info(info_tag, $sformatf("EX: Reg WR %02d = 0x%08x", ex_reg_addr, ex_reg_wdata), UVM_DEBUG);
        if (!trace_ex_delay_is_null &&
            !trace_ex_delay.got_regs_write &&
            !trace_ex_delay.is_load &&
            ((!trace_ex_delay.is_apu && (ex_valid || !wb_valid)) ||
             (trace_ex_delay.is_apu && apu_rvalid_i &&
              (apu_singlecycle_i || apu_multicycle_i)))) begin
          apply_reg_write(trace_ex_delay, ex_reg_addr, ex_reg_wdata);
          trace_ex_delay.got_regs_write = 1;
        end else if (!trace_ex_is_null) begin
          apply_reg_write(trace_ex, ex_reg_addr, ex_reg_wdata);
          if (trace_ex.got_regs_write) begin
            `uvm_info(info_tag, $sformatf(
                "EX: Multiple Reg WR %02d = 0x%08x", ex_reg_addr, ex_reg_wdata), UVM_DEBUG);
          end
          trace_ex.got_regs_write = 1;
        end else begin
          `uvm_info(info_tag, $sformatf(
              "EX: Reg WR %02d:0x%08x but no active EX instruction",
              ex_reg_addr, ex_reg_wdata), UVM_DEBUG);
        end
      end

      // WB register write
      if (wb_reg_we) begin
        `uvm_info(info_tag, $sformatf("WB: Reg WR %02d = 0x%08x", wb_reg_addr, wb_reg_wdata), UVM_DEBUG);
        if (!trace_ex_delay_is_null &&
            (trace_ex_delay.is_load ||
             (trace_ex_delay.is_apu && apu_rvalid_i &&
              !apu_singlecycle_i && !apu_multicycle_i))) begin
          apply_reg_write(trace_ex_delay, wb_reg_addr, wb_reg_wdata);
          trace_ex_delay.got_regs_write = 1;
        end else if (!trace_wb_is_null && !trace_wb.got_regs_write) begin
          if (!trace_wb.is_load || (trace_wb.is_load && wb_valid)) begin
            apply_reg_write(trace_wb, wb_reg_addr, wb_reg_wdata);
            trace_wb.got_regs_write = 1;
          end
        end else if (!trace_ex_is_null && !trace_ex.got_regs_write && trace_ex.misaligned) begin
          // Double load: managed by trace_wb, nothing to do here
        end else begin
          `uvm_info(info_tag, $sformatf(
              "WB: Reg WR %02d:0x%08x but no active WB instruction",
              wb_reg_addr, wb_reg_wdata), UVM_DEBUG);
        end
      end

      // Memory access in EX
      if (ex_data_req && ex_data_gnt) begin
        if (ex_data_we) begin
          `uvm_info(info_tag, $sformatf(
              "EX: Mem WR 0x%08x = 0x%08x", ex_data_addr, ex_data_wdata), UVM_DEBUG);
        end else begin
          `uvm_info(info_tag, $sformatf("EX: Mem RD 0x%08x", ex_data_addr), UVM_DEBUG);
        end
        if (trace_ex_is_null) begin
          `uvm_info(info_tag, $sformatf(
              "EX: Mem %s 0x%08x:0x%08x but no active EX instruction",
              ex_data_we ? "WR" : "RD", ex_data_addr, ex_reg_wdata), UVM_DEBUG);
        end else begin
          apply_mem_access(trace_ex, ex_data_we, ex_data_addr, ex_data_wdata);
        end
      end

    end
  end

endmodule : cv32e40p_tracer

`endif  // CV32E40P_TRACE_EXECUTION