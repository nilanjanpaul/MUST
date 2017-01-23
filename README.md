# mrtx_config

# Example run:
# ./rf_hw_intf --conf "devices.xml,/devices/grid_tx" --tx-only


# sync="octave -qf sync.m"
# samps="octave -qf waveform1.m"
# ./rf_hw_intf --conf "devices.xml,/devices/grid_tx" --tx-only --sync "`$sync`" --sig "`$samps`"


# sync="octave -qf sync.m"
# ./rf_hw_intf --conf "devices.xml,/devices/grid_rx" --rx-only --sync "`$sync`" --cmd-port 5111


# For octave display - rx_handler_find_idx()
# figure(1,"position",[1924 900 1150 200])  ; rx_udp_multi_receive;
# figure(2,"position",[1924 700 1150 200]); rx_udp_multi_receive;
# figure(3,"position",[1924 500 1150 200]); rx_udp_multi_receive;
