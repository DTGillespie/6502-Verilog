module 6502_System_Wrapper (
	input clk,
	input btn0,     // Reset
	input sw0, 		// Manual clock mode
	input btn3, 	// Step clock
	output reg ld0, // Reset indicator
	output reg ld1, // Step clock indicator 
	output reg ld2, // Manual clock mode indicator
	output ca, cb, cc, cd, ce, cf, cg, dp, // 7 segmented display cathodes + decimal point
	output aa, ab, ac, ad // Digit select anodes
);

wire [15:0] address_bus;
wire [7:0] data_bus;

wire clock_1MHz;
ClockDivider_1MHz clk_div(
	.clk_in(clock),
	.clk_out(clock_1MHz)
);

ControlUnit cu(
	.sw0(sw0),
	.btn3(btn3),
	.reset(btn0),
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
		ld0 <= 1'b1;
	end else begin
		ld0 <= 1'b0;
	end
	
	if (sw0) begin
		ld2 <= 1'b1;
	end else begin
		ld2 <= 1'b0;
	end
	
	if (btn3) begin
		ld1 <= 1'b1;
	end else begin
		ld1 <= 1'b0;
	end
	
end

assign {ca, cb, cc, cd, ce, cf, cg, dp} = seg_a;

endmodule

module ControlUnit(
	input wire sw0,
	input wire btn3,
	input wire reset,
	input wire clk_in,
	input wire [7:0] data_bus_in,
	output wire [15:0] addr_bus_out
);

parameter IDLE <= 2'b00;
parameter FETCH <= 2'b01;
parameter DECODE <= 2'b10;
parameter EXECUTE <= 2'b11;

reg clk_internal;

reg [1:0] state;

reg read;
reg write;
reg enable;

reg [15:0] pc;
reg [7:0] pcl;
reg [7:0] pch;

reg [7:0] opcode;
reg [1:0] addr_mode;

wire clk_rising_edge;
RisingEdgeDetector detector (
	.clk(clk_in),
	.clk_in(clk_in),
   .rising_edge(clk_rising_edge)
);
always @(posedge clk_in or posedge reset) begin
	if (reset) begin
		// Initialize CU On Reset
		state <= IDLE;
		read <= 1'b0;
		write <= 1'b0;
		enable <= 1'b0;
	end else begin 
		// State Transition Logic
		case(state)
			IDLE: begin
				// Transition To State => FETCH
				state <= FETCH;
				// Control Fetch Instruction Signals
				read <= 1'b1;   // Enable reading from memory (ROM)
				write <= 1'b0;  // Disable writing to memory
				enable <= 1'b1; // Enable memory operation
				pc <= 16'h0000; // Initialize program counter to 0x0000
			end
			FETCH: begin
				// Transition To State => DECODE
				state <= DECODE;
				// Control Decode Instruction Signals
				read <= 1'b0;
				write <= 1'b0;
				enable <= 1'b0;
			end
			DECODE: begin
				// Transition To State => EXECUTE (Based on opcode and addressing mode)
				if (opcode_in == 6'b110000 && addr_mode_in == 2'b01) begin
					// Example: Branch if equal instruction(opcode 110000, address mode: 01)
					state <= EXECUTE;
					// IMPLEMENT BRANCH-INSTRUCTION LOGIC HERE
				end else begin
					// HANDLE OTHER OPCODES AND ADDRESSING MODES
					pc <= pc_in + 1; // Assuming Simple Update
				end
			end
			EXECUTE: begin
				// Transition To State => IDLE
				state <= IDLE;
				// IMPLEMENT 6502 LOGIC FOR RETURNING TO IDLE STATE
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

	rom[16'h0000] <= 8'hA0;
	rom[16'h0001] <= 8'hFF;

end

assign data_bus_out = rom[addr_bus_in];

endmodule

// ********** Utilities/Misc **********

module SegmentedDisplayEncoder (
	input wire clk_in,
	input wire [3:0] dgt,
	output [7:0] reg c_out,
	output [3:0] reg a_out
);

wire clk;
ClockDivider_60Hz clk_div (
	.clk_in(clk_in),
	.clk_out(clk),
);

always @(posedge clk) begin
	case (dgt) 
		// High = OFF | Low = ON
		4'h0: c_out <= 7'b1000000;
		4'h1: c_out <= 7'b1111001;
		4'h2: c_out <= 7'b0100100;
		4'h3: c_out <= 7'b0110000;
		4'h4: c_out <= 7'b0011001;
		4'h5: c_out <= 7'b0010010;
		4'h6: c_out <= 7'b0000010;
		4'h7: c_out <= 7'b1111000;
		4'h8: c_out <= 7'b0000000;
		4'h9: c_out <= 7'b0011000;
		4'hA: c_out <= 7'b0000100;
		4'hB: c_out <= 7'b0000011;
		4'hC: c_out <= 7'b1000110;
		4'hD: c_out <= 7'b0100001;
		4'hE: c_out <= 7'b0000110;
		4'hF: c_out <= 7'b0001110;
		default : c_cout = 7'b1111111;
	endcase
end

endmodule

module ClockDivider_1MHz ( // From => 50MHz
	input wire clk_in,
	output reg clk_out
);

reg [24:0] cnt;

initial begin
	clk_out <= 1'b0;
end

always @(posedge clk_in) begin
	if (cnt == 25'h1FFFFF) begin
		cnt = 0;
		clk_out = ~clk_out;
	end else begin
		cnt = cnt + 1;
	end
end

endmodule

module ClockDivider_60Hz ( // From => 1MHz
	input wire clk_in,
	output reg clk_out
);

reg [23:0] cnt;

always @(posedge clk_in) begin
	if (cnt == 24'h41A1A - 1) begin
		cnt = 24'b0;
		clk_out = ~clk_out;
	end else begin
		cnt = cnt + 1;
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
	clk_prev <= clk_in;
end

assign rising_edge = (clk_in && !clk_prev);

endmodule