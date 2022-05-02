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
set_property PACKAGE_PIN H6 [get_ports {enable[0]}]

#Configuring Output ports to get PWM signal
set_property PACKAGE_PIN C17 [get_ports {pwm[3]}]
set_property PACKAGE_PIN D18 [get_ports {pwm[2]}]
set_property PACKAGE_PIN E18 [get_ports {pwm[1]}]
set_property PACKAGE_PIN G17 [get_ports {pwm[0]}]

#Configuring clk with internal clock pin E3
set_property PACKAGE_PIN E3 [get_ports clk]
