`timescale 1ns / 1ps

module Top_Module(
input clk,                   //Clock signal
input [3:0] enable,          //Enable signal
input [3:0] angle,           //ON/OFF FPGA switch to select angle
input [3:0] clk_trig,        //Trigger switch on FPGA for PIPO
output [3:0] pwm             //PWM output
    );
    
    wire [27:0] t[3:0];      //Output from PIPO
    wire [27:0] ontime_BUS;  //Ontime_bus
   
    //Module instantiations
    
    Angle_to_on_time DUT(angle,ontime_BUS);  
   
    PIPO_shift_register I_1( clk_trig[0],ontime_BUS,t[0]);
    PWM_generator  I_2(clk,enable[0],t[0],pwm[0]);
    
    PIPO_shift_register I_3( clk_trig[1],ontime_BUS,t[1]);
    PWM_generator  I_4(clk,enable[1],t[1],pwm[1]);
    
    PIPO_shift_register I_5( clk_trig[2],ontime_BUS,t[2]);
    PWM_generator  I_6(clk,enable[2],t[2],pwm[2]);
    
    PIPO_shift_register I_7( clk_trig[3],ontime_BUS,t[3]);
    PWM_generator  I_8(clk,enable[3],t[3],pwm[3]);
   
endmodule
