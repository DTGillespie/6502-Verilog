// ********** CMOS 6502 **********

module CpuCore(
	input debug_mode,
	input debug_step,
	input clock,
	input reset,
	output reg reset_indicator,
	output reg debug_mode_indicator,
	output reg debug_step_indicator,
	output seg_select,
	output seg
);


// Segmented LED Display 
SegmentedDisplayEncoder sde (
	.sys_clk_in(clock),
	.seg_select_out(seg_select),
	.seg_out(seg)
);

wire [15:0] address_bus;
wire [7:0] data_bus;

wire clock_1MHz;
ClockDivider clk_div(
	.clk_in(clock),
	.clk_out(clock_1MHz)
);

ControlUnit cu(
	.debug_mode(debug_mode),
	.debug_step(debug_step),
	.reset(reset),
	.clk_in(clock_1MHz),
	.data_bus_in(data_bus),
	.addr_bus_out(address_bus)
);

ROM rom (
	.addr_bus_in(address_bus),
	.data_bus_out(data_bus)
);

always @(*) begin

	if (reset) begin
		reset_indicator = 1'b1;
	end else begin
		reset_indicator = 1'b0;
	end
	
	if (debug_mode) begin
		debug_mode_indicator = 1'b1;
	end else begin
		debug_mode_indicator = 1'b0;
	end
	
	if (debug_step) begin
		debug_step_indicator = 1'b1;
	end else begin
		debug_step_indicator = 1'b0;
	end
	
end

endmodule

module ControlUnit(
	input wire debug_mode,
	input wire debug_step,
	input wire reset,
   input wire clk_in,
	input wire [7:0] data_bus_in,
	output wire [15:0] addr_bus_out
);

parameter IDLE = 2'b00;
parameter FETCH = 2'b01;
parameter DECODE = 2'b10;
parameter EXECUTE = 2'b11;

reg clk_internal;

reg [1:0] state;

reg read;
reg write;
reg enable;

reg [15:0] pc;
reg [7:0] pcl;
reg [7:0] pch;
//reg [15:0] current_pc;
//reg [15:0] next_pc;

reg [7:0] opcode;
reg [1:0] addr_mode;

task PrepFetch (
	input clk_in,
	output state_out,
	output read_out,
	output write_out,
	output enable_out,
	output pc_out
);
	begin
		// Wait For Rising Edge Of Clock
		if (clk_in) begin
			// Transition To State => FETCH
			state_out = FETCH;
			// Control Fetch Instruction Signals
			read_out = 1'b1;   // Enable reading from memory (ROM)
			write_out = 1'b0;  // Disable writing to memory
			enable_out = 1'b1; // Enable memory operation
			pc_out = 16'h0000; // Initialize program counter to 0x0000
		end
	end
endtask

task Fetch (
	output state_out,
	output read_out,
	output write_out,
	output enable_out
);
	begin
		// Transition To State => DECODE
		state_out = DECODE;
		// Control Decode Instruction Signals
		read_out = 1'b0;
		write_out = 1'b0;
		enable_out = 1'b0;
	end
endtask

task DecodeOp (
	input opcode_in, 
	input addr_mode_in, 
	input pc_in,
	output state_out, 
	output reg pc_out
);
	begin
		// Transition To State => EXECUTE (Based on opcode and addressing mode)
		if (opcode_in == 6'b110000 && addr_mode_in == 2'b01) begin
			// Example: Branch if equal instruction(opcode 110000, address mode: 01)
			state_out = EXECUTE;
			// IMPLEMENT BRANCH-INSTRUCTION LOGIC HERE
		end else begin
			// HANDLE OTHER OPCODES AND ADDRESSING MODES
			pc_out = pc_in + 1; // Assuming Simple Update
		end
	end
endtask

task Execute (
	output state_out
);
	begin
		// Transition To State => IDLE
		state = IDLE;
		// IMPLEMENT 6502 LOGIC FOR RETURNING TO IDLE STATE
	end
endtask

wire clk_rising_edge;
RisingEdgeDetector detector (
	.clk(clk_in),
	.clk_in(clk_in),
   .rising_edge(clk_rising_edge)
);
always @(posedge clk_in or posedge reset) begin
	if (reset) begin
		// Initialize CU On Reset
		state = IDLE;
		read = 1'b0;
		write  = 1'b0;
		enable = 1'b0;
	end else begin 
		// State Transition Logic
		case(state)
			IDLE: begin
				PrepFetch(clk_rising_edge, state, read, write, enable, pc);
			end
			FETCH: begin
				Fetch(state, read, write, enable);
			end
			DECODE: begin
				DecodeOp(opcode, addr_mode, pc, state, pc);
			end
			EXECUTE: begin
				Execute(state);
			end
		endcase
	end
end

// Manual Debugging
wire debug_clk_rising_edge;
RisingEdgeDetector debug_detector (
	.clk(debug_step),
	.clk_in(debug_step),
   .rising_edge(debug_clk_rising_edge)
);
always @(posedge debug_step) begin
	if (reset && debug_mode) begin
		state = IDLE;
		read = 1'b0;
		write  = 1'b0;
		enable = 1'b0;
	end else begin 
		case(state)
			IDLE: begin
				PrepFetch(debug_clk_rising_edge, state, read, write, enable, pc);
			end
			FETCH: begin
				Fetch(state, read, write, enable);
			end
			DECODE: begin
				DecodeOp(opcode, addr_mode, pc, state, pc);
			end
			EXECUTE: begin
				Execute(state);
			end
		endcase
	end
end

// Assign PCL and PCH to the address bus
assign addr_bus_out = {pc[7:0], pc[15:8]};

endmodule

// ********** ROM **********
module ROM (
	input wire [15:0] addr_bus_in,
	output wire [7:0] data_bus_out
);

reg [7:0] rom [0:16];

integer i;

initial begin

	rom[16'h0000] = 8'hA0;
	rom[16'h0001] = 8'hFF;

end

assign data_bus_out = rom[addr_bus_in];

endmodule

// ********** Utilities/Misc **********

// 7 Segment Display
module SegmentedDisplayEncoder (
	input wire sys_clk_in,
	output reg seg_select_out,
	output reg seg_out
);

integer i;

always @(posedge sys_clk_in) begin
	seg_out = ~seg_out;
	seg_select_out = 1'b1;
end

endmodule

// 1MHz Clock Divider
module ClockDivider (
	input wire clk_in,
	output reg clk_out
);

reg [24:0] counter;

initial begin
	clk_out = 1'b0;
end

always @(posedge clk_in) begin
	if (counter == 25'h1FFFFF) begin
		counter = 0;
		clk_out = ~clk_out;
	end else begin
		counter = counter + 1;
	end
end

endmodule

// Rising Edge Detector
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
