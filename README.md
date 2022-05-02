# 3DOF_RoboticArmFPGA
This repository contains the details of controlling a 3-DOF robotic arm with 4 servo motors using FPGA. The design is prototyped using the Nexys-4 DDR Artix-7  FPGA of Xilinx. This repo contails all the files necessary to complete the project such as Verilog code, test bench and constrain file.
# Table of Contents:
* [Introduction](#Introduction)
* [Tool Used](#Tool-Used)
    * [Vivado](#Vivado)
* [Algorithm](#Algorithm)
* [Verilog Code](#Verilog-Code)
    * [1. Top Module](#1-Top-Module)
    * [2. Angle to On-time Convertor](#2-Angle-to-On-time-Convertor)
    * [3. PIPO Shift Register](#3-PIPO-Shift-Register)
    * [4. PWM_Generator](#4-PWM_Generator)
* [Test Bench](#Test-Bench)
* [Constraints File](#Constraints-File)
* [Simulation Results](#Simulation-Results)
* [Acknowledgement](#Acknowledgement)
* [Author](#Author)



# Introduction

In this rapidly changing world, automation is becoming an essential part of almost every industry, be it manufacturing or the healthcare field. When we talk about automation, the first thing that comes to our mind is a robot. Robotics and automation are two distinct technologies but these terms are often used interchangeably. Together, they have tremendously transformed the manufacturing space. Pertaining to the manufacturing world, the most commonly used robot is the Robotic Arm. The Robotic Arm can be used in several manufacturing applications ranging from the assembly line to fault detection and packaging.

There are two ways of controlling the robotic arm to perform a particular task. A popular method is to use a microcontroller. A microcontroller based Robotic Arm offers programmability. However, this approach is a bit slower as it has to perform memory read/write operations to execute the instructions. Another approach can be to design a custom ASIC package to implement the Robotic Arm task algorithm. Custom ASIC package offers high speed. Nevertheless, it is costly for low volume production and it takes time to get the chip manufactured. A middle path is to use and FPGA to implement the Robotic Arm controller. FPGA offers programmability and is a quick way of getting the prototype ready. It is also the favoured technology for low volume applications.

We have used FPGA approach in our project. We have done prototyping using Nexys4DDR board equipped with Xilinx Artix-7 series FPGA. In addition, we have used SG90 servo motors for actuating the Robotic Arm. Verilog HDL has been used for programming the FPGA. The simulation and FPGA implementation of the design has been carried out using the Xilinx Vivado tool. We have designed a prototype of a Robotic Arm controller with 4-DOF (Degree of Freedom). The proposed arm is designed to operate manually and automatically to perform a particular action.

# Tool Used

### Vivado


<p align="center" width="100%">

   <img width="50%" src="https://user-images.githubusercontent.com/65393666/166301464-0940cd42-beb9-418e-937e-fe98b9aa0d58.png">
   
</p>

The Vivado Design Suite is a collection of Xilinx tools for designing, programming, synthesis, and analysis of hardware description language (HDL). It replaces Xilinx ISE and adds new functionality for system-on-a-chip development and synthesis. The Xilinx Vivado High-Level Synthesis (HLS) compiler provides a programming environment that is similar to that found on both regular and specialised processors for application development. For the interpretation, analysis, and optimization of C/C++ programs, Vivado HLS shares key technology with processor compilers. The main distinction is in the application's execution target.

Vivado is an integrated design environment (IDE) featuring system-to-IC level capabilities built on a shared scalable data model and a common debug environment that was released in April 2012. Vivado offers ESL design tools for synthesising and testing C-based algorithmic IP; standards-based packaging of both algorithmic and RTL IP for reuse; standards-based IP stitching and systems integration of all types of system building blocks; and block and system verification. The Vivado Webpack Edition is a free version of the design environment that provides designers with a limited version of the design environment.

Vivado HLS enables a software engineer to optimise code for throughput, power, and latency without having to deal with the performance bottleneck of a single memory area and restricted computing resources by targeting an FPGA as the execution fabric. This enables the incorporation of computationally costly software algorithms into genuine goods rather than merely functionality demos. The same categories apply to application code targeted at the Vivado HLS compiler as they do to any other processor compiler. All programs are analysed in terms of operations, conditional statements, loops, and functions using Vivado HLS.

# Algorithm

As previously stated, the time of the PWM signal sent at the input determines the angle of rotation of a servo motor. Our mission was to control all of the servos independently and from various angles. One servo should not have any effect on another servo.

As a result, based on the demand, the entire code was separated into the following modules:
  1. Top Module
  2. Angle to on-time convertor (RTL_ROM) 
  3. N-bit Parallel In Parallel Out (PIPO) shift register.
  4. PWM Generator Module
  
Fig. 4.2 shows the schematic view of the proposed solution. The concept was to use an Angle to the on-time converter to choose a desired input from the input switch and then transfer the matching on time to a single on-time bus. All of the PIPO shift registers shared the on-time bus. The PIPO shift register was used before each PWM generator to update the on-time of the associated PWM generator only when a CLK trigger pulse was present at the input. When we apply the trigger pulse to a specific PIPO shift, the on-time of the associated PWM generator is updated to the value on the on-time bus at the time. Using this method, we can give any angle to any servo without changing the angles of other servos. Also, an enable signal is used just for each servo for the safety purpose to avoid any conflict. In addition, for the sake of safety, an enable signal is used for each servo.

<p align="center" width="100%">

   <img width="100%" src="https://user-images.githubusercontent.com/65393666/166303548-4248fde1-4c0f-4968-8402-e6ea529d137f.png">
   
</p>

<p align="center">
<b>Fig:1 Schematic View of Proposed Model from Vivado HLS</b></br>
</p>

# Verilog Code

### 1. Top Module

```
`timescale 1ns / 1ps

module Top_Module(
input clk,		      //Clock signal
input [3:0] enable,          //Enable signal
input [3:0] angle,           //ON/OFF FPGA switch to select angle
input [3:0] clk_trig,        //Trigger switch on FPGA for PIPO
output [3:0] pwm             //PWM output signal
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
```

### 2. Angle to On-time Convertor

```
`timescale 1ns / 1ps

module Angle_to_on_time(
  input [3:0] angle,        //ON/OFF FPGA switch to select angle
  output reg [27:0] out     //Ontime_bus
    );
parameter x=100000;// factor for 1 ms 
 
    always@(angle)
        begin
        case(angle)
        
        /*Assigning set  of on-time/Angle using different ON/OFF 
                 Switches for a particular action to be performed*/
        4'b0001: out= 1.0*x; // 45  degree
        4'b0010: out= 1.5*x; // 90  degree
        4'b0100: out= 2.0*x; // 135 degree
        4'b1000: out= 2.5*x; // 180 degree 
        default: out= 0;     // default on-time is zero
        endcase 

    end
endmodule
```

### 3. PIPO Shift Register

```
`timescale 1ns / 1ps

module PIPO_shift_register(
    input clk_trig,         //Trigger switch on FPGA for PIPO
    input [27:0] in,        //Input to PIPO: On-time bus output
    output reg [27:0] out   //Output from PIPO: PWM generator input
    );
    
    always @(posedge clk_trig)
    out<=in;   //Input is transferred to output on positive clk edge

endmodule
```

### 4. PWM_Generator

```
`timescale 1ns / 1ps

module PWM_generator(
    input  clk,         //Clock signal
    input enable,       //Input enable-ON/OFF switch on FPGA
    input [27:0] ontime,//Input on-time: Output From PIPO
    output reg pwm      //PWM output signal
    );
    
    reg [27:0] n;       //On-time count variable
    reg [27:0] period;  //Period of pulse 
    parameter x=100000; // factor for 1 ms 
    
    //Setting initial conditions
    initial
    begin
    n<=0;          //Count=0
    pwm <=1'b1;    //Output signal is Logic High
    period <= 20*x;//20 ms
    end
    
    //Main PWM code
        always@(posedge clk)
        begin
            if(enable)
            begin
                if(ontime!=0)
                begin
                      n<=n+1;
                      if( n == ontime )
                                pwm <= 1'b0 ;
                      else if (  n == period)
                          begin
                                pwm <= 1'b1 ;      
                                n<=0;
                          end      
                end 
                else 
                pwm<=1'b0;
                              
            end
             
            else
            pwm<=1'b0;
                     
        end
  
endmodule
```
# Test Bench

```
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
//Enable signal of PWM Gen;    Data on Angle Bus;                Giving ontime to PWM Gen by generating trigger pulse;            
#1000000   enable=4'b0001;    #1000000 angle= 4'b0100;   #1000000 clk_trig[0]  =1;       	    #100000 clk_trig[0]  =0;
#100000000 enable=4'b0011;  #1000000 angle= 4'b0010;   #1000000 clk_trig[1]  =1;   	    #100000 clk_trig[1]  =0;
#100000000 enable=4'b0111;  #1000000 angle= 4'b0100;   #1000000 clk_trig[2:0]=3'b111;  #100000 clk_trig[2:0]=3'b000;
#100000000 enable=4'b1100;  #1000000 angle= 4'b1000;   #1000000 clk_trig[3:2]=2'b11;    #100000 clk_trig[3:2]=2'b00;
end

always
#5 clk=~clk;             //200 MHz clock frequency same as internal clock of FPGA

initial
#500000000 $finish;  

endmodule
```

# Constraints File

```
#For using internal clock of FPGA = 100 MHz as input clock signal
set_property -dict{PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports {clk}];

#For using trigger switches of FPGA for generating trigger pulses and disabling internal clock for the given input clk_trig
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_trig_IBUF[0]] 
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_trig_IBUF[1]]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_trig_IBUF[2]]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_trig_IBUF[3]]   

#Defining all the I/O ports aa Standard LVCMOS33

set_property IOSTANDARD LVCMOS33 [get_ports clk]

set_property IOSTANDARD LVCMOS33 [get_ports {angle[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {angle[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {angle[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {angle[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {clk_trig[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk_trig[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk_trig[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {clk_trig[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {enable[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enable[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enable[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {enable[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {pwm[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pwm[0]}]

#Configuring ON/OFF switches to assign Angle
set_property PACKAGE_PIN R15 [get_ports {angle[3]}]
set_property PACKAGE_PIN M13 [get_ports {angle[2]}]
set_property PACKAGE_PIN L16 [get_ports {angle[1]}]
set_property PACKAGE_PIN J15 [get_ports {angle[0]}]

#Configuring trigger switches for generating trigger pulse
set_property PACKAGE_PIN P17 [get_ports {clk_trig[3]}]
set_property PACKAGE_PIN P18 [get_ports {clk_trig[2]}]
set_property PACKAGE_PIN M17 [get_ports {clk_trig[1]}]
set_property PACKAGE_PIN M18 [get_ports {clk_trig[0]}]

#Configuring ON/OFF switches for enable
set_property PACKAGE_PIN V10 [get_ports {enable[3]}]
set_property PACKAGE_PIN U11 [get_ports {enable[2]}]
set_property PACKAGE_PIN U12 [get_ports {enable[1]}]
set_property PACKAGE_PIN H6  [get_ports {enable[0]}]

#Configuring Output ports to get PWM signal
set_property PACKAGE_PIN C17 [get_ports {pwm[3]}]
set_property PACKAGE_PIN D18 [get_ports {pwm[2]}]
set_property PACKAGE_PIN E18 [get_ports {pwm[1]}]
set_property PACKAGE_PIN G17 [get_ports {pwm[0]}]

#Configuring clk with internal clock pin E3
set_property PACKAGE_PIN E3  [get_ports clk]
```

# Simulation Results

<p align="center" width="100%">

   <img width="90%" src="https://user-images.githubusercontent.com/65393666/166306843-e438d8b4-4760-43ef-9696-58865ea22c47.png">
   
</p>

<p align="center">
<b>Fig. 2 Simulation results of test bench from Vivado HLS tool</b></br>
</p>

# Acknowledgement

Dr. Naushad Alam, Associate Professor, Department of Electronics Engineering, ZHCET, AMU.

# Author

[Rishabh Verma](https://github.com/Rishabh-zhcet), B.Tech Electronics Engineering, Zakir Husain College of Engineering and Technology (ZHCET), Aligarh Muslim University(AMU).
