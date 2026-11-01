# ============================================================================
# de2_115_shape_detect.sdc  -  timing constraints
# ============================================================================
# 50 MHz board oscillator
create_clock -name CLOCK_50   -period 20.000 [get_ports {CLOCK_50}]
# TRDB-D5M pixel clock (PIXCLK = XCLKIN = 25 MHz, PLL bypassed)
create_clock -name D5M_PIXCLK -period 40.000 [get_ports {GPIO[0]}]

derive_clock_uncertainty

# The two domains are asynchronous; every crossing goes through cdc_bus /
# sync_bits / dual-clock RAM.
set_clock_groups -asynchronous -group {CLOCK_50} -group {D5M_PIXCLK}

# Camera data, LVAL, FVAL: launched on the falling PIXCLK edge by the sensor
# (register 0x0A bit 15 set), captured on the rising edge -> ~20 ns margin.
set_input_delay -clock D5M_PIXCLK -max 8.0 [get_ports {GPIO[1] GPIO[3] GPIO[4] GPIO[5] GPIO[6] GPIO[7] GPIO[8] GPIO[9] GPIO[10] GPIO[11] GPIO[12] GPIO[13] GPIO[21] GPIO[22]}]
set_input_delay -clock D5M_PIXCLK -min 1.0 [get_ports {GPIO[1] GPIO[3] GPIO[4] GPIO[5] GPIO[6] GPIO[7] GPIO[8] GPIO[9] GPIO[10] GPIO[11] GPIO[12] GPIO[13] GPIO[21] GPIO[22]}]

# Slow / asynchronous board I/O
set_false_path -from [get_ports {KEY[*] SW[*] EX_IO[*] GPIO[23]}]
set_false_path -to   [get_ports {LEDR[*] LEDG[*] HEX*[*] LCD_* EX_IO[*] GPIO[16] GPIO[17] GPIO[19] GPIO[23] GPIO[24]}]

# VGA: data and VGA_CLK both leave from CLOCK_50 registers; VGA_CLK rises one
# CLOCK_50 period after the data changes (20 ns setup / 20 ns hold at the DAC).
set_false_path -to [get_ports {VGA_*}]
