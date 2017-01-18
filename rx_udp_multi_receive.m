% Needs octave version 3.8.0+ and octave-sockets-1.2.0

if (exist("rcv_sck","var") == 0)
  rcv_port = 1337;
  rcv_sck = fUDP_open_rx(rcv_port);
  printf("Recv UDP pkts on port %i\n", rcv_port)
endif

if (exist("SELECT","var") == 0)
  SELECT = 0;
endif

  _spb = 256;


  SYNC_TONE_1 = 4;
  SYNC_TONE_2 = 14;

  REFERENCE_ = zeros(1, _spb/2);
  REFERENCE_( SYNC_TONE_1/2 + 1) = .9;
  ref4_ = (ifft(REFERENCE_)) * _spb/2;


  REFERENCE_ = zeros(1, _spb/2);
  REFERENCE_( SYNC_TONE_2/2 + 1) = .9;
  ref16_ = (ifft(REFERENCE_)) * _spb/2;

  reference_ = [ref4_ ref16_ zeros(1,256*3)];  % this is TD reference signal
  REFERENCE_ = fft(reference_, 1024);   % this is FD reference signal


%while (true)
  printf("\nWaiting for signal...\n");
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

  plot (real( MultiDeviceBuffer(1,:)  ),'b' );
  sleep(.1)
  
    signal_ = MultiDeviceBuffer(1,:);
    SIGNAL_  = fft(signal_, 1024);
    signal_corr_ = ifft(SIGNAL_ .* conj(REFERENCE_ )) * 1024/2;
    [x,xi] = max(abs(signal_corr_));
    s = sprintf("idx,mag:: %i  %f ", xi, x);
    display(s)


%endwhile
