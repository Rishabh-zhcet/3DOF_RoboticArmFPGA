`timescale 1ns / 1ps

module Angle_to_on_time(
  input [3:0] angle,        //ON/OFF FPGA switch to select angle
  output reg [27:0] out     //Ontime_bus
    );
parameter x=100000;// factor for 1 ms 
 
    always@(angle)
        begin
        case(angle)
        
        /*Assigning set  of on-time/Angle using diffrent ON/OFF switches
        for a particluar action to be performed*/
        4'b0001: out= 1.0*x; // 45  degree
        4'b0010: out= 1.5*x; // 90  degree
        4'b0100: out= 2.0*x; // 135 degree
        4'b1000: out= 2.5*x; // 180 degree 
        default: out= 0; //default on-time is zero
        endcase
    end
endmodule