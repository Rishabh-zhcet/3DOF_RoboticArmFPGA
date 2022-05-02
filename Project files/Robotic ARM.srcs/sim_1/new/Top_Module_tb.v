`timescale 1ns / 1ps

module Top_Module_tb();
reg clk;
reg [3:0] enable;
reg [3:0] angle;
reg [3:0] clk_trig;
wire [3:0]pwm;

Top_Module DUT(clk,enable,angle,clk_trig, pwm);

initial 
    begin
    clk=0;enable=0;clk_trig=0;angle=0;
    end

initial

begin
//Enable signal of PWM Gen; Data on Angle Bus;       Giving ontime to PWM Gen by generating trigger pulse;            
#1000000   enable=4'b0001;  #1000000 angle= 4'b0100; #1000000 clk_trig[0]  =1;     #100000 clk_trig[0]  =0;
#100000000 enable=4'b0011;  #1000000 angle= 4'b0010; #1000000 clk_trig[1]  =1;     #100000 clk_trig[1]  =0;
#100000000 enable=4'b0111;  #1000000 angle= 4'b0100; #1000000 clk_trig[2:0]=3'b111;#100000 clk_trig[2:0]=3'b000;
#100000000 enable=4'b1100;  #1000000 angle= 4'b1000; #1000000 clk_trig[3:2]=2'b11; #100000 clk_trig[3:2]=2'b00;
end

always
#5 clk=~clk;             //200 MHz clock frequency same as internal clock of FPGA

initial
#500000000 $finish;

endmodule
