// ********** CMOS 6502 **********

module CpuCore(
	input clock,
	input reset,
	output reg reset_indicator
);

wire [15:0] address_bus;
wire [7:0] data_bus;

wire clock_1MHz;
ClockDivider clk_div(
	.clk_in(clock),
	.clk_out(clock_1MHz)
);

ControlUnit cu(
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
end

endmodule

module ControlUnit(
	input wire reset,
   input wire clk_in,
	input wire [7:0] data_bus_in,
	output wire [15:0] addr_bus_out
);

parameter IDLE = 2'b00;
parameter FETCH = 2'b01;
parameter DECODE = 2'b10;
parameter EXECUTE = 2'b11;

reg [1:0] state;

reg read;
reg write;
reg enable;

reg [15:0] pc;
reg [7:0] pcl;
reg [7:0] pch;
reg [15:0] current_pc;
//reg [15:0] next_pc;

reg [7:0] opcode;

reg [1:0] addr_mode;

wire clk_rising_edge;
RisingEdgeDetector detector (
	.clk(clk_in),
	.clk_in(clk_in),
   .rising_edge(clk_rising_edge)
);

// CU Finite State Machine
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
				// Wait For Rising Edge Of Clock
				if (clk_rising_edge) begin
					// Transition To State => FETCH
					state = FETCH;
					// Control Fetch Instruction Signals
					read = 1'b1;   // Enable reading from memory (ROM)
					write = 1'b0;  // Disable writing to memory
					enable = 1'b1; // Enable memory operation
					pc = 16'h0000; // Initialize program counter to 0x0000
				end
			end
			FETCH: begin
				// Transition To State => DECODE
				state = DECODE;
				// Control Decode Instruction Signals
				read = 1'b0;
				write = 1'b0;
				enable = 1'b0;
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
//assign next_pc = (state == FETCH) ? pc + 1 : pc;

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
