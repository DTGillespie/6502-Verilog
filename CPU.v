`timescale 1ns / 1ps

module CU(
    input wire clk,
    input wire reset,
    input wire opcode,
    input wire [1:0] addr_mode,
	 input wire [15:0] current_pc,
	 output wire [15:0] next_pc,
	 output wire [7:0] pcl,
	 output wire [7:0] pch,
    output wire read,
    output wire write,
    output wire enable
);

// Define TCU States
parameter IDLE = 2'b00;
parameter FETCH = 2'b01;
parameter DECODE = 2'b10;
parameter EXECUTE = 2'b11;

reg [1:0] state; // TCU State Register

// Define TCU Control Signals (Memory Access Signals)
reg read_reg; 	// Read
reg write_reg;  // Write
reg enable_reg; // Enable

// Program Counter
reg [15:0] pc;

// Rising Edge Utility - Architecture Deviation
wire clk_rising_edge;
RisingEdgeDetector detector (
  .clk(clk),
  .clk_in(clk),
  .rising_edge(clk_rising_edge)
);

// TCU Finite State Machine
always @(posedge clk or posedge reset) begin
	if (reset) begin
		// Initialize TCU On Reset
		state = IDLE;
		read_reg = 1'b0;
		write_reg = 1'b0;
		enable_reg = 1'b0;
	end else begin 
		// State Transition Logic
		case(state)
			IDLE: begin
				// Wait For Rising Edge Of Clock
				if (clk_rising_edge) begin
					// Transition To State => FETCH
					state = FETCH;
					// Control Fetch Instruction Signals
					read_reg = 1'b1;
					write_reg = 1'b0;
					enable_reg = 1'b1;
					pc = 16'h0000;
				end
			end
			FETCH: begin
				// Transition To State => DECODE
				state = DECODE;
				// Control Decode Instruction Signals
				read_reg = 1'b0;
				write_reg = 1'b0;
				enable_reg = 1'b0;
			end
			DECODE: begin
				// Transition To State => EXECUTE (Based on opcode and addressing mode)
				if (opcode == 6'b110000 && addr_mode == 2'b01) begin
					// Example: Branch if equal instruction(opcode 110000, address mode: 01)
					state = EXECUTE;
					// IMPLEMENT BRANCH-INSTRUCTION LOGIC HERE
				end else begin
					// HANDLE OTHER OPCODES AND ADDRESSING MODES
					pc = current_pc + 1; // Assuming Simple Update
				end
			end
			EXECUTE: begin
				// Transition To State => IDLE
				state = IDLE;
				// IMPLEMENT 6502 LOGIC FOR RETURNING TO IDLE STATE
			end
		endcase
	end
end

// Assign the next_pc output based on the current_pc and state
assign next_pc = (state == FETCH) ? pc + 1 : pc;

// Assign PCL and PCH to the address bus
assign pcl = pc[7:0];
assign pch = pc[15:8];

endmodule

// Rising Edge Detection Utility
module RisingEdgeDetector (
	input wire clk,
	input wire clk_in, // Target signal
	output wire rising_edge
);

reg clk_prev;

always @(posedge clk) begin
	clk_prev = clk_in;
end

assign rising_edge = (clk_in && !clk_prev);

endmodule
