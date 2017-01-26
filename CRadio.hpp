#include <iostream>
#include <string.h>
#include <cassert>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/thread/thread.hpp>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <pugixml.hpp>

#include "CDeviceStorage.hpp"

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

#define RETURN_SUCCESS 0
#define RETURN_ERROR  -1

// Define number of buffers to store
#define RX_DEV_STORAGE_N_SPB_BUFFS  (64*4)
#define TX_DEV_STORAGE_N_SPB_BUFFS  (16)

struct __s_tuner_conf {
  double freq;
  double rate;
  double gain;
  double bw;
  std::string ant;
  unsigned int idxChannelAssignment;
};

struct __s_radio_conf {
  std::string type;
  std::string ip;
  std::vector<struct __s_tuner_conf> rx;
  std::vector<struct __s_tuner_conf> tx;
};

class CRadio
{
  private:
    uhd::usrp::multi_usrp::sptr _usrp;
    uhd::rx_streamer::sptr _rx_stream;
    uhd::tx_streamer::sptr _tx_stream;

    std::string _devargs;
    std::string _ref,_sync;
    std::string _subdev;
    std::string _rxant;
    std::string _txant;
    double _rxrate, _rxfreq, _rxgain;
    double _txrate, _txfreq, _txgain;
    double _bw;
    bool _isRxInit, _isTxInit, _is_rx_streaming_on, _is_tx_streaming_on;
    bool _is_rx_pause_on;

    std::vector<struct __s_radio_conf>  _radio_configs;
    unsigned int _nRadiosInConfig;        // number of USRP devices
    unsigned int _nRxChannels;            // total daughter boards with rx path in this configuration
    unsigned int _nTxChannels;            // total daughter boards with rx path in this configuration
    size_t _spb;

    CDeviceStorage _rx_mdst;
    CDeviceStorage _tx_mdst;
  //DeviceStorageType _rx_mdst; // this should go on heap or convert to SHM class
  //DeviceStorageType _tx_mdst; // this should go on heap or convert to SHM class

    void _run_rx();
    void _run_tx();


  public:
    CRadio();
    int make_device_handle(std::string XML_FILE_PATH, std::string xml_path_conf);
    void init_rx();
    void init_tx();

    CDeviceStorage& rx_mdst() { return _rx_mdst; } 
    CDeviceStorage& tx_mdst() { return _tx_mdst; } 

    void run_rx_async( );
    void kill_rx() { _is_rx_streaming_on = false; }

    void pause_rx() { _is_rx_pause_on = true; }
    void cont_rx()  { _is_rx_pause_on = false; }

    void plot_buffer(std::string udp_dst_addr, std::string port, void *ptr, unsigned int n_cf32_values);
    void plot_rx(std::string udp_dst_addr, std::string port, unsigned int start_idx, unsigned int nbuffs);
    void print_radio_configurations();

    void run_tx_async();
    void stop_tx() { _is_tx_streaming_on = false; }


    unsigned int nRxChannels() { return _nRxChannels; }
    unsigned int nTxChannels() { return _nTxChannels; }
    unsigned int spb()         { return _spb; }

    bool is_RxStreaming()      { return _is_rx_streaming_on; }

    // get handlers for freq gain rate values for a given channel
    double rx_freq(unsigned int ch)   { return _usrp->get_rx_freq( ch ); }
    double rx_rate(unsigned int ch)   { return _usrp->get_rx_rate( ch ); }
    double rx_gain(unsigned int ch)   { return _usrp->get_rx_gain( ch ); }

    double tx_freq(unsigned int ch)   { return _usrp->get_tx_freq( ch ); }
    double tx_rate(unsigned int ch)   { return _usrp->get_tx_rate( ch ); }
    double tx_gain(unsigned int ch)   { return _usrp->get_tx_gain( ch ); }


    // set handlers for rx freq gain rate values for a given channel
    double rx_freq(unsigned int ch, double val)
    {
      _usrp->set_rx_freq( val, ch );
      return _usrp->get_rx_freq( ch );
    }

    double rx_rate(unsigned int ch, double val) 
    {
      _usrp->set_rx_rate( val, ch );
      return _usrp->get_rx_rate( ch );
    }
    
    double rx_gain(unsigned int ch, double val)
    {
      _usrp->set_rx_gain( val, ch );
      return _usrp->get_rx_gain( ch );
    }

    double tx_freq(unsigned int ch, double val)
    {
      _usrp->set_tx_freq( val, ch );
      return _usrp->get_tx_freq( ch );
    }

    double tx_rate(unsigned int ch, double val) 
    {
      _usrp->set_tx_rate( val, ch );
      return _usrp->get_tx_rate( ch );
    }
    
    double tx_gain(unsigned int ch, double val)
    {
      _usrp->set_tx_gain( val, ch );
      return _usrp->get_tx_gain( ch );
    }

    std::string get_rx_ant() { return _rxant; }

};




CRadio::CRadio()
{
  _isRxInit = _isTxInit = _is_rx_streaming_on = false;
  _is_rx_pause_on = false;
  _spb = 256;
}

int CRadio::make_device_handle(std::string XML_FILE_PATH, std::string xml_path_conf)
{

  // =====================================================================================
  // read configurations from XML file into radio_conf structure
  // =====================================================================================

  // get config from XML file
  pugi::xml_document doc;
  doc.load_file(XML_FILE_PATH.c_str(), pugi::parse_default|pugi::parse_declaration);
  pugi::xml_node root = doc.document_element();

  std::string searchStr = xml_path_conf;
  pugi::xpath_node xpathNode = root.select_single_node(searchStr.c_str());
  if (!xpathNode) { // if xpath does not exist then bail
    std::cout << searchStr << " not exist\n";
    return RETURN_ERROR;
  }

  pugi::xml_node pugiNode = xpathNode.node();
  //pugiNode.print(std::cout,"  ");

  _sync = pugiNode.attribute("sync") ? pugiNode.attribute("sync").as_string() : "now";
  
  unsigned int rx_ch_assignment = 0;
  unsigned int tx_ch_assignment = 0;
  for (pugi::xml_node devNode = pugiNode.first_child(); devNode; devNode = devNode.next_sibling() ) {
    struct __s_radio_conf  radio_conf;
    radio_conf.type = devNode.attribute("type").as_string();
    radio_conf.ip   = devNode.attribute("ip").as_string();
    for (pugi::xml_node chNode = devNode.first_child(); chNode; chNode = chNode.next_sibling() ) {
      struct __s_tuner_conf tuner_conf;

      tuner_conf.freq = chNode.attribute("rx_freq") ? chNode.attribute("rx_freq").as_double()
	                                            : pugiNode.attribute("rx_freq").as_double();

      tuner_conf.gain = chNode.attribute("rx_gain") ? chNode.attribute("rx_gain").as_double()
	                                            : pugiNode.attribute("rx_gain").as_double();

      tuner_conf.rate = chNode.attribute("rx_rate") ? chNode.attribute("rx_rate").as_double()
	                                            : pugiNode.attribute("rx_rate").as_double();

      tuner_conf.bw = chNode.attribute("rx_bw") ? chNode.attribute("rx_bw").as_double()
                                                : pugiNode.attribute("rx_bw").as_double();

      tuner_conf.ant = chNode.attribute("rx_ant") ? chNode.attribute("rx_ant").as_string()
                                                  : pugiNode.attribute("rx_ant").as_string();

      tuner_conf.idxChannelAssignment = rx_ch_assignment++;
      radio_conf.rx.push_back( tuner_conf );

      struct __s_tuner_conf tx_tuner_conf;
      tx_tuner_conf.freq = chNode.attribute("tx_freq") ? chNode.attribute("tx_freq").as_double()
	                                            : pugiNode.attribute("tx_freq").as_double();

      tx_tuner_conf.gain = chNode.attribute("tx_gain") ? chNode.attribute("tx_gain").as_double()
	                                            : pugiNode.attribute("tx_gain").as_double();

      tx_tuner_conf.rate = chNode.attribute("tx_rate") ? chNode.attribute("tx_rate").as_double()
	                                            : pugiNode.attribute("tx_rate").as_double();

      tx_tuner_conf.bw = chNode.attribute("tx_bw") ? chNode.attribute("tx_bw").as_double()
                                                : pugiNode.attribute("tx_bw").as_double();

      tx_tuner_conf.ant = chNode.attribute("tx_ant") ? chNode.attribute("tx_ant").as_string()
                                                  : pugiNode.attribute("tx_ant").as_string();


      tx_tuner_conf.idxChannelAssignment = tx_ch_assignment++;
      radio_conf.tx.push_back( tx_tuner_conf );

    }
    _radio_configs.push_back(radio_conf);
  }

  _nRadiosInConfig = _radio_configs.size();


  // =====================================================================================
  // Create multi-usrp device handler
  // =====================================================================================

  // construct device args for multiple usrps.
  // B200 devices require 'type'.
  // n210, x310 devices require IP 'addr'.
  for (int i = 0 ; i < _radio_configs.size(); i++) {
    if (_radio_configs.at(i).type == "b200")
      _devargs += "type" + boost::lexical_cast<std::string>(i) + "=" + _radio_configs.at(i).type + ",";
    else
      _devargs += "addr" + boost::lexical_cast<std::string>(i) + "=" + _radio_configs.at(i).ip + ",";
  }
  std::cerr << boost::format("Creating the usrp device: %s") % _devargs << std::endl;

  _usrp = uhd::usrp::multi_usrp::make(_devargs);
  std::cerr << boost::format("Using Device: %s") % _usrp->get_pp_string() << std::endl;


  unsigned int num_mboards = _usrp->get_num_mboards();
  _nRxChannels = _usrp->get_rx_num_channels();
  _nTxChannels = _usrp->get_tx_num_channels();
  
  std::cerr << "Number mboards        = " << num_mboards << std::endl;
  std::cerr << "Number of rx channels = " << _nRxChannels << std::endl;
  std::cerr << "Number of tx channels = " << _nTxChannels << std::endl;
 


  // =====================================================================================
  // Device synchronization
  // =====================================================================================
  std::cerr << boost::format("Setting device timestamp to 0...") << std::endl;
  if (_sync == "now"){
    _ref = "internal";
    //This is not a true time lock, the devices will be off by a few RTT.
    //Rather, this is just to allow for demonstration of the code below.
    _usrp->set_clock_source(_ref);
    _usrp->set_time_now(uhd::time_spec_t(0.0) );
  }
  else if (_sync == "pps"){
    _ref = "external";
    _usrp->set_time_source(_ref);
    _usrp->set_clock_source(_ref);
    _usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    boost::this_thread::sleep(boost::posix_time::seconds(1)); //wait for pps sync pulse
  }
#if 0
  else if (_sync == "mimo"){
    UHD_ASSERT_THROW(_usrp->get_num_mboards() == 2);

    //make mboard 1 a slave over the MIMO Cable
    _usrp->set_clock_source("mimo", 1);
    _usrp->set_time_source("mimo", 1);

    //set time on the master (mboard 0)
    _usrp->set_time_now(uhd::time_spec_t(0.0), 0);

    //sleep a bit while the slave locks its time to the master
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
#endif


  return RETURN_SUCCESS;

}

void CRadio::init_rx()
{

  if (_isRxInit == true)
    return;
     
  // =====================================================================================
  // Set up uhd rx chain
  // =====================================================================================

  //always select the subdevice first, the channel mapping affects the other settings
  //if (vm.count("subdev")) usrp->set_rx_subdev_spec(_subdev);

  for (int i = 0; i < _nRadiosInConfig; i++) {
    struct __s_radio_conf radio_conf = _radio_configs.at(i);


    unsigned int totalRxChannelsForThisRadio = radio_conf.rx.size(); 
    for (int j = 0 ; j < totalRxChannelsForThisRadio ; j++) {
      struct __s_tuner_conf tuner_conf = radio_conf.rx.at(j);

      _usrp->set_rx_rate( tuner_conf.rate, tuner_conf.idxChannelAssignment );
      _usrp->set_rx_freq( tuner_conf.freq, tuner_conf.idxChannelAssignment );
      _usrp->set_rx_gain( tuner_conf.gain, tuner_conf.idxChannelAssignment );
    
      _usrp->set_rx_antenna( tuner_conf.ant, tuner_conf.idxChannelAssignment );
      _usrp->set_rx_bandwidth(tuner_conf.bw, tuner_conf.idxChannelAssignment );

      unsigned int ch_i = tuner_conf.idxChannelAssignment;
      std::cout << boost::format("Device: %s ") % radio_conf.ip << " --- " << "channel assignment: " << ch_i << std::endl;
      std::cout << boost::format("RX Rate: %f (%f) Msps") % (_usrp->get_rx_rate( ch_i )/1e6) %  (tuner_conf.rate/1e6) << std::endl;
      std::cout << boost::format("RX Freq: %f (%f) MHz") % (_usrp->get_rx_freq( ch_i )/1e6) % (tuner_conf.freq/1e6) << std::endl;
      std::cout << boost::format("RX Gain: %f (%f) dB") % _usrp->get_rx_gain( ch_i ) % tuner_conf.gain << std::endl;
      std::cout << boost::format("RX Ant : %s (%s) ") % _usrp->get_rx_antenna( ch_i ) % tuner_conf.ant << std::endl;
      std::cout << boost::format("RX BW: %f (%f) MHz") % (_usrp->get_rx_bandwidth( ch_i )/1e6) % (tuner_conf.bw/1e6) << std::endl;
      std::cout << std::endl;
    }

  }

  boost::this_thread::sleep(boost::posix_time::seconds(1.0)); //allow for some setup time

  for (int i = 0; i < _nRadiosInConfig; i++) {
    struct __s_radio_conf radio_conf = _radio_configs.at(i);

    unsigned int totalRxChannelsForThisRadio = radio_conf.rx.size(); 
    for (int j = 0 ; j < totalRxChannelsForThisRadio ; j++) {
      struct __s_tuner_conf tuner_conf = radio_conf.rx.at(j);

      //Check Ref and LO Lock detect
      std::vector<std::string> sensor_names;
      sensor_names = _usrp->get_rx_sensor_names( tuner_conf.idxChannelAssignment );
      if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
	uhd::sensor_value_t lo_locked = _usrp->get_rx_sensor("lo_locked", tuner_conf.idxChannelAssignment );
	std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string() << std::endl;
	UHD_ASSERT_THROW(lo_locked.to_bool());
      }
      sensor_names = _usrp->get_mboard_sensor_names(0);
      if ((_ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
	uhd::sensor_value_t mimo_locked = _usrp->get_mboard_sensor("mimo_locked",0);
	std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string() << std::endl;
	UHD_ASSERT_THROW(mimo_locked.to_bool());
      }
      if ((_ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()))  {
	uhd::sensor_value_t ref_locked = _usrp->get_mboard_sensor("ref_locked",0);
	std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string() << std::endl;
	UHD_ASSERT_THROW(ref_locked.to_bool());
      }

    }
  }

  // =====================================================================================
  // Allocate device streaming buffers
  // =====================================================================================
  _rx_mdst.shared_memory( "/ShmMultiDeviceBufferRx");

  _rx_mdst.init( _nRxChannels , _spb, RX_DEV_STORAGE_N_SPB_BUFFS);
  _rx_mdst.print_info();
  _isRxInit = true;

} // End of init_rx


void CRadio::run_rx_async()
{
  //boost::thread t(boost::bind( &CRadio::_run_rx, this, boost::ref(rx_mdst) ));
  boost::thread t(boost::bind( &CRadio::_run_rx, this ) );
  _is_rx_streaming_on = true;
}

void CRadio::_run_rx( )
{

  _usrp->set_time_now(uhd::time_spec_t(0.0) );

  //create a receive streamer - populate channel list
  //linearly map channels (index0 = channel0, index1 = channel1, ...)
  uhd::stream_args_t stream_args("fc32"); //complex floats
  for (unsigned int i = 0; i < _nRxChannels; i++)
    stream_args.channels.push_back(i);
  assert( stream_args.channels.size() == _nRxChannels);

  _rx_stream = _usrp->get_rx_stream(stream_args);

  //setup streaming
  double delay_sec = 0.5;
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(delay_sec);
  _rx_stream->issue_stream_cmd(stream_cmd); //tells all channels to stream samples to host. This is the GO button.


  //the first call to recv() will block this many seconds before receiving
  double timeout = delay_sec + 0.1; //timeout (delay before receive + padding)
  boost::system_time next_refresh = boost::get_system_time();
  uhd::rx_metadata_t _md;
  size_t num_rx_samps;
  //float pkt_time_intv_us = 1/_usrp->get_rx_rate( 0 ) * _spb * 1e6;
 
  while( _is_rx_streaming_on ) {

    //if (boost::get_system_time() < next_refresh) continue;
    //next_refresh = boost::get_system_time() + boost::posix_time::microseconds(long( pkt_time_intv_us  ));

    //receive multi channel buffers
    num_rx_samps = _rx_stream->recv( _rx_mdst.buffer_ptr_idx(_rx_mdst.head() ), _spb, _md, timeout);

    if (num_rx_samps != _spb) {
      std::cerr << "!!";
      continue;
    }

    //handle the error code
    if (_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
    else if (_md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) continue;
    else if (_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
      throw std::runtime_error(str(boost::format("Recv'd samples %i\nReceiver error %s") % num_rx_samps % _md.strerror()));
    }
    
    //use a small timeout for subsequent packets
    timeout = 0.01;

    if (_is_rx_pause_on) continue;

    _rx_mdst._BufferTime.at( 0 ).full_sec = _md.time_spec.get_full_secs();
    _rx_mdst._BufferTime.at( 0 ).frac_sec = _md.time_spec.get_frac_secs();

    // if storage buffers are full then drop from tail end 
    if (_rx_mdst.head( +1 ) == _rx_mdst.tail(  ) )
      _rx_mdst.tail_inc( );

    _rx_mdst.head_inc( );

  } // while()

  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  stream_cmd.stream_now = true;
  _rx_stream->issue_stream_cmd(stream_cmd);

} // end of run_rx



void CRadio::plot_buffer(std::string udp_dst_addr, std::string port, void *ptr_cf32, unsigned int n_cf32_values)
{
  std::cout << "Sending samples to " << udp_dst_addr << ":" << port << std::endl;
  // send out samples via udp_xport

  uhd::transport::udp_simple::sptr udp_xport = uhd::transport::udp_simple::make_connected(udp_dst_addr, port);

  // send number of channels (or streams)
  unsigned int Channels = 1;
  udp_xport->send(boost::asio::buffer( &Channels , sizeof(Channels) ));

  // send number of cf32 samples to receive
  udp_xport->send(boost::asio::buffer( &n_cf32_values , sizeof(n_cf32_values) ));

  unsigned int num_samps_per_datagram = _spb; //num_samps_per_datagram = (uhd::transport::udp_simple::mtu) / sizeof(std::complex<float>);

  // send number of datagrams
  unsigned int num_udp_datagrams = floor((float)n_cf32_values / (float)(num_samps_per_datagram));
  udp_xport->send(boost::asio::buffer( &num_udp_datagrams , 4 ));


  std::complex<float> *ptr_to_buffer = (std::complex<float> *)ptr_cf32;
  for (unsigned int i = 0; i < num_udp_datagrams; ++i) {
    udp_xport->send(boost::asio::buffer((void*)ptr_to_buffer,
					num_samps_per_datagram * sizeof(std::complex<float>) )
		    );
    ptr_to_buffer += num_samps_per_datagram;
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }

}

void CRadio::plot_rx(std::string udp_dst_addr, std::string port, unsigned int start_idx, unsigned int nbuffs)
{
  std::cout << "Sending samples to " << udp_dst_addr << ":" << port << std::endl;
  // send out samples via udp_xport

  uhd::transport::udp_simple::sptr udp_xport = uhd::transport::udp_simple::make_connected(udp_dst_addr, port);

  udp_xport->send(boost::asio::buffer( &_nRxChannels , 4 ));

  size_t total_num_samps = _spb * nbuffs;
  udp_xport->send(boost::asio::buffer( &total_num_samps , 4 ));

  unsigned int num_samps_per_datagram = _spb; //num_samps_per_datagram = (uhd::transport::udp_simple::mtu) / sizeof(std::complex<float>);

  // DEBUG replace floor with ceil
  unsigned int num_udp_datagrams = floor((float)total_num_samps / (float)(num_samps_per_datagram));
  udp_xport->send(boost::asio::buffer( &num_udp_datagrams , 4 ));

  for (unsigned int channel = 0; channel < _nRxChannels; ++channel) {
    //std::complex<float> *ptr_to_device_buffer = (std::complex<float> *)rx_mdst.buffer_ptr_( channel, 0);
    std::complex<float> *ptr_to_device_buffer = (std::complex<float> *)_rx_mdst.buffer_ptr_ch_idx_( channel, start_idx);

    for (unsigned int i = 0; i < num_udp_datagrams; ++i) {
      udp_xport->send(boost::asio::buffer((void*)ptr_to_device_buffer,
					  num_samps_per_datagram * sizeof(std::complex<float>) )
		      );
      ptr_to_device_buffer += num_samps_per_datagram;
      //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
  }

}

void CRadio::print_radio_configurations()
{
  std::cout << "Device list:: " << std::endl;
  for (int i = 0 ; i < _radio_configs.size(); i++) {
    struct __s_radio_conf radio_conf = _radio_configs.at(i);
    std::cout << "  type : " << radio_conf.type;
    std::cout << "  ip : " << radio_conf.ip;
    std::cout << "  rx channels : " << radio_conf.rx.size();
    //std::cout << "  tx chan : " << radio_configs.tx.size();
    std::cout << std::endl;
    for (int j = 0 ; j < _radio_configs.at(i).rx.size(); j++) {
      struct __s_tuner_conf tuner_conf = _radio_configs.at(i).rx.at(j);
      std::cout << "  rx : " << tuner_conf.freq << " " << tuner_conf.rate << " " << tuner_conf.gain << " " << tuner_conf.bw << " " << tuner_conf.ant << std::endl;
      std::cout << "  rx channel assignment : " << tuner_conf.idxChannelAssignment << std::endl;
    }
  }
}


void CRadio::init_tx()
{

  if (_isTxInit == true)
    return;

  // =====================================================================================
  // Set up uhd tx chain
  // =====================================================================================

  //always select the subdevice first, the channel mapping affects the other settings
  //if (vm.count("subdev")) usrp->set_rx_subdev_spec(_subdev);

  
  for (int i = 0; i < _nRadiosInConfig; i++) {
    struct __s_radio_conf radio_conf = _radio_configs.at(i);


    unsigned int totalTxChannelsForThisRadio = radio_conf.tx.size();  DBG_OUT(totalTxChannelsForThisRadio);
    for (int j = 0 ; j < totalTxChannelsForThisRadio ; j++) {
      struct __s_tuner_conf tuner_conf = radio_conf.tx.at(j);         DBG_OUT(tuner_conf.idxChannelAssignment);

      _usrp->set_tx_rate( tuner_conf.rate, tuner_conf.idxChannelAssignment );
      _usrp->set_tx_freq( tuner_conf.freq, tuner_conf.idxChannelAssignment );
      _usrp->set_tx_gain( tuner_conf.gain, tuner_conf.idxChannelAssignment );
    
      _usrp->set_tx_antenna( tuner_conf.ant, tuner_conf.idxChannelAssignment );
      _usrp->set_tx_bandwidth(tuner_conf.bw, tuner_conf.idxChannelAssignment );

      unsigned int ch_i = tuner_conf.idxChannelAssignment;
      std::cout << boost::format("Device: %s ") % radio_conf.ip << " --- " << "channel assignment: " << ch_i << std::endl;
      std::cout << boost::format("TX Rate: %f (%f) Msps") % (_usrp->get_tx_rate( ch_i )/1e6) %  (tuner_conf.rate/1e6) << std::endl;
      std::cout << boost::format("TX Freq: %f (%f) MHz") % (_usrp->get_tx_freq( ch_i )/1e6) % (tuner_conf.freq/1e6) << std::endl;
      std::cout << boost::format("TX Gain: %f (%f) dB") % _usrp->get_tx_gain( ch_i ) % tuner_conf.gain << std::endl;
      std::cout << boost::format("TX Ant : %s (%s) ") % _usrp->get_tx_antenna( ch_i ) % tuner_conf.ant << std::endl;
      std::cout << boost::format("TX BW: %f (%f) MHz") % (_usrp->get_tx_bandwidth( ch_i )/1e6) % (tuner_conf.bw/1e6) << std::endl;
      std::cout << std::endl;
    }
    
  }

  boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

  for (int i = 0; i < _nRadiosInConfig; i++) {
    struct __s_radio_conf radio_conf = _radio_configs.at(i);

    unsigned int totalTxChannelsForThisRadio = radio_conf.tx.size(); 
    for (int j = 0 ; j < totalTxChannelsForThisRadio ; j++) {
      struct __s_tuner_conf tuner_conf = radio_conf.tx.at(j);

      //Check Ref and LO Lock detect
      std::vector<std::string> sensor_names;
      sensor_names = _usrp->get_tx_sensor_names( tuner_conf.idxChannelAssignment );
      if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
	uhd::sensor_value_t lo_locked = _usrp->get_tx_sensor("lo_locked", tuner_conf.idxChannelAssignment );
	std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
	UHD_ASSERT_THROW(lo_locked.to_bool());
      }
      sensor_names = _usrp->get_mboard_sensor_names(0);
      if ((_ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
	uhd::sensor_value_t mimo_locked = _usrp->get_mboard_sensor("mimo_locked",0);
	std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string() << std::endl;
	UHD_ASSERT_THROW(mimo_locked.to_bool());
      }
      if ((_ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()))  {
	uhd::sensor_value_t ref_locked = _usrp->get_mboard_sensor("ref_locked",0);
	std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
	UHD_ASSERT_THROW(ref_locked.to_bool());
      }

    }
  }

  // =====================================================================================
  // device streaming buffers
  // =====================================================================================
  _tx_mdst.shared_memory( "/ShmMultiDeviceBufferTx");;
  _tx_mdst.init( _nTxChannels , _spb, TX_DEV_STORAGE_N_SPB_BUFFS);

  _tx_mdst.print_info();
  _isTxInit = true;

} // End of init_tx



void CRadio::run_tx_async()
{
  boost::thread t(boost::bind( &CRadio::_run_tx, this ));
  _is_tx_streaming_on = true;
}


void CRadio::_run_tx()
{

  std::vector<std::complex<float> > zero_buf(_spb);

  //create a receive streamer - populate channel list
  //linearly map channels (index0 = channel0, index1 = channel1, ...)
  uhd::stream_args_t stream_args("fc32"); //complex floats
  for (unsigned int i = 0; i < _nTxChannels; i++)
    stream_args.channels.push_back(i);
  assert( stream_args.channels.size() == _nTxChannels);

  _tx_stream = _usrp->get_tx_stream(stream_args);

  //setup streaming
  double delay_sec = 1.5;

  //the first call to recv() will block this many seconds before receiving
  double timeout = delay_sec + 0.1; //timeout (delay before receive + padding)
  //boost::system_time next_console_refresh = boost::get_system_time();
  boost::system_time next_refresh = boost::get_system_time();

  uhd::tx_metadata_t md;   //setup the metadata flags
  md.start_of_burst = true;
  md.end_of_burst   = false;
  md.has_time_spec  = true;
  md.time_spec = uhd::time_spec_t(0.1);

  
  float pkt_time_intv_us = 10; //1/_usrp->get_tx_rate( 0 ) * _spb * 1e6;

  while( _is_tx_streaming_on ) {
    
    if (boost::get_system_time() < next_refresh) continue;
    next_refresh = boost::get_system_time() + boost::posix_time::microseconds(long( pkt_time_intv_us  ));

    if (_tx_mdst.head() == 0) {
      _tx_stream->send((void*)zero_buf.data(), _spb, md);
      md.start_of_burst = false;
      md.has_time_spec = false;
      continue;
    }

    // send multi channel buffers to device
    for (unsigned int idx = 0; idx < _tx_mdst.head(); idx++)
    {
      _tx_stream->send(_tx_mdst.buffer_ptr_idx( idx ), _spb, md);
      md.start_of_burst = false;
      md.has_time_spec = false;
    }
    
    // Comment line below to keep sending sequence
    _tx_mdst.head_set(0);

  } // while()

  // ???? send a mini EOB packet
  md.end_of_burst = true;
  _tx_stream->send("", 0, md);


}
