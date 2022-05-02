`timescale 1ns / 1ps

module PIPO_shift_register(
    input clk_trig,         //Trigger switch on FPGA for PIPO
    input [27:0] in,        //Input to PIPO: On-time bus output
    output reg [27:0] out   //Output from PIPO: PWM generator input
    );
    
    always @(posedge clk_trig)
    out<=in;                //Input is transferred to output on positive clk edge

endmodule
