`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2020 07:48:35 PM
// Design Name: 
// Module Name: controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// 
// TODO:
// 1.  Put sclk on MRCC or SRCC pin 
module controller(
    sclk,
    cs,
    mosi,
    miso,
    sd_cmd,
    sd_clk,
    send_cmd
    );
    
    
    // Upstream interface to the Processor
    (* mark_debug = "true" *) input sclk;
    (* mark_debug = "true" *) input cs;
    (* mark_debug = "true" *) input mosi;
    (* mark_debug = "true" *) output miso;
    (* mark_debug = "true" *) input send_cmd;
    
    // Down stream SD Card Interface
    (* mark_debug = "true" *) output sd_clk;
    (* mark_debug = "true" *) inout sd_cmd;
    
    // Output signal definitions
    wire miso;
    wire sd_cmd;
    wire sd_clk;

    // Bi-directional gating control of access to the SD CMD line
    wire cmd_dir_out;
    wire cmd_bit_out;
    wire cmd_bit_in;    
    assign sd_cmd = cmd_dir_out ? cmd_bit_out : 1'bz;
    assign cmd_bit_in = ~cmd_dir_out ? sd_cmd : 1'b0;
    
    // Pass through the cmd line value to miso whenever we are not sending a cmd
    assign miso = cmd_bit_in;
    
    // SD clocking
    assign sd_clk = sclk;
   
    // Instantiation of a Command Bus Controller
    cmd_bus cmd_bus_0(
        sclk,
        cs,
        mosi,
        send_cmd,
        cmd_bit_in,
        cmd_bit_out,
        cmd_dir_out
    );    
endmodule
