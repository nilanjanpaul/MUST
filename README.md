###########################################
# Average Bandwiwth power
###########################################

# Transmit side:
# ./rf_hw_intf --conf "devices.xml,/devices/grid_tx" --tx-only --cmd-port 5112
# samps="octave -qf awgn256.m"
# ./sigtran --sig "`$samps`" --intv 500


# Receive side:
# ./rf_hw_intf --conf "devices.xml,/devices/grid_rx" --rx-only --cmd-port 5111
# ./sigproc_avg_bw_pwr --time 100





###########################################
# Find signal start idx
###########################################

# Transmit side:
# ./rf_hw_intf --conf "devices.xml,/devices/grid_tx" --tx-only --cmd-port 5112
# samps="octave -qf sync.m"
# ./sigtran --sig "`$samps`" --intv 500


# Receive side:
# ./rf_hw_intf --conf "devices.xml,/devices/grid_rx" --rx-only --cmd-port 5111
# sync="octave -qf sync.m"
# ./sigproc_start_idx --sync  "`$sync`"


# For octave display - rx_handler_find_idx()
# figure(1,"position",[1924 900 1150 200]); rx_udp_multi_receive;
# figure(2,"position",[1924 700 1150 200]); rx_udp_multi_receive;
# figure(3,"position",[1924 500 1150 200]); rx_udp_multi_receive;
