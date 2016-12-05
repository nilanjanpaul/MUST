% Needs octave version 3.8.0+ and octave-sockets-1.2.0

if (exist("rcv_sck","var") == 0)
  rcv_port = 1337;
  rcv_sck = fUDP_open_rx(rcv_port);
  printf("Recv UDP pkts on port %i\n", rcv_port)
endif

if (exist("SELECT","var") == 0)
  SELECT = 0;
endif

if (SELECT == 0)
  clear y
  clear MultiDeviceBuffer

  [recv_data, recv_count]=recv(rcv_sck,4000,MSG_WAITALL);
  num_rx_channels = typecast(uint8(recv_data), 'uint32')

  [recv_data, recv_count]=recv(rcv_sck,4000,MSG_WAITALL);
  total_num_samples = typecast(uint8(recv_data), 'uint32')

  [recv_data, recv_count]=recv(rcv_sck,4000,MSG_WAITALL);
  num_udp_datagrams = typecast(uint8(recv_data), 'uint32')

  for channel = 1:num_rx_channels
    s = sprintf("channel = %i",channel);
    display(s)
    for idx = 1:num_udp_datagrams
      [recv_data, recv_count]=recv(rcv_sck,4000,MSG_WAITALL); y(:,idx) = typecast(uint8(recv_data), 'single complex');
    endfor
    MultiDeviceBuffer(channel,:) = y(:);
  endfor


  plot (real( MultiDeviceBuffer(1,[100:end])  ),'b' ); hold on;
  plot (real( MultiDeviceBuffer(2,[100:end])  ),'r' ); hold off;

  last_idx =  length(MultiDeviceBuffer(1,:));
  max ( real( MultiDeviceBuffer(1,[100: last_idx])  )  )
  max ( real( MultiDeviceBuffer(2,[100: last_idx])  )  )
  sleep(1);

endif


