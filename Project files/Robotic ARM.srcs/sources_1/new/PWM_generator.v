`timescale 1ns / 1ps

module PWM_generator(
    input  clk,         //Clock signal
    input enable,       //Input enable-ON/OFF switch on FPGA
    input [27:0] ontime,//Input on-time: Output From PIPO
    output reg pwm      //PWM output signal
    );
    
    reg [27:0] n;       //On-time count variable
    reg [27:0] period;  //Period of pulse 
    parameter x=100000;// factor for 1 ms 
    
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