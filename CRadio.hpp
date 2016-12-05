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

#include "DeviceStorage.h"

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

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
  bool _isRxInit, _isTxInit, _is_RxStreaming, _is_TxStreaming;
    std::vector<struct __s_radio_conf>  _radio_configs;
    unsigned int _nRadiosInConfig;
    unsigned int _nRxChannels;
    unsigned int _nTxChannels;
    size_t _spb;
    void _run_rx(DeviceStorageType& rx_mdst);
    void _run_tx(DeviceStorageType& tx_mdst);


  public:
    CRadio();
    void make_device_handle(std::string XML_FILE_PATH, std::string xml_path_conf);
    void init_rx();
    void init_tx();

    void run_rx(DeviceStorageType& rx_mdst);
    void stop_rx() { _is_RxStreaming = false; }
    void run_rx(DeviceStorageType& rx_mdst, double seconds_in_future, size_t total_num_samps );

    void plot1(DeviceStorageType& rx_mdst, std::string udp_dst_addr, std::string port);
    void print_radio_configurations();

    void run_tx(DeviceStorageType& tx_mdst);
    void stop_tx() { _is_TxStreaming = false; }


    unsigned int nRxChannels() { return _nRxChannels; }
    unsigned int nTxChannels() { return _nTxChannels; }
    unsigned int spb() { return _spb; }
    bool is_RxStreaming()  {return _is_RxStreaming; }

    double get_rx_freq()   { return _rxfreq; }
    double get_rx_rate()   { return _rxrate; }
    double get_rx_gain()   { return _rxgain; }
    std::string get_rx_ant() { return _rxant; }

    double get_tx_freq()   { return _txfreq; }
    double get_tx_rate()   { return _txrate; }
    double get_tx_gain()   { return _txgain; }
};




CRadio::CRadio()
{
  _isRxInit = _isTxInit = _is_RxStreaming = false;
  _spb = 256;
}

void CRadio::make_device_handle(std::string XML_FILE_PATH, std::string xml_path_conf)
{
  // get config from XML file
  pugi::xml_document doc;
  doc.load_file(XML_FILE_PATH.c_str(), pugi::parse_default|pugi::parse_declaration);
  pugi::xml_node root = doc.document_element();

  std::string searchStr = xml_path_conf;
  pugi::xpath_node xpathNode = root.select_single_node(searchStr.c_str());
  if (!xpathNode) { // if xpath does not exist then bail
    std::cout << searchStr << " not exist\n";
    exit;
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
      tuner_conf.freq = chNode.attribute("rx_freq").as_double();
      tuner_conf.rate = chNode.attribute("rx_rate").as_double();
      tuner_conf.gain = chNode.attribute("rx_gain").as_double();
      tuner_conf.bw   = chNode.attribute("rx_bw") ? chNode.attribute("rx_bw").as_double() : (double)(25e6); 
      tuner_conf.ant  = chNode.attribute("rx_ant") ? chNode.attribute("rx_ant").as_string() : "RX2"; 
      tuner_conf.idxChannelAssignment = rx_ch_assignment++;
      radio_conf.rx.push_back( tuner_conf );

      struct __s_tuner_conf tx_tuner_conf;
      tx_tuner_conf.freq = chNode.attribute("tx_freq").as_double();
      tx_tuner_conf.rate = chNode.attribute("tx_rate").as_double();
      tx_tuner_conf.gain = chNode.attribute("tx_gain").as_double();
      tx_tuner_conf.bw   = chNode.attribute("tx_bw") ? chNode.attribute("tx_bw").as_double() : (double)(25e6);
      tx_tuner_conf.ant  = chNode.attribute("tx_ant") ? chNode.attribute("tx_ant").as_string() : "TX/RX"; 
      tx_tuner_conf.idxChannelAssignment = tx_ch_assignment++;
      radio_conf.tx.push_back( tx_tuner_conf );

    }
    _radio_configs.push_back(radio_conf);
  }

  _nRadiosInConfig = _radio_configs.size();

  for (int i = 0 ; i < _radio_configs.size(); i++)
    _devargs += "addr" + boost::lexical_cast<std::string>(i) + "=" + _radio_configs.at(i).ip + ",";
  
  std::cout << boost::format("Creating the usrp device: %s") % _devargs << std::endl;
  _usrp = uhd::usrp::multi_usrp::make(_devargs);

  std::cout << boost::format("Using Device: %s") % _usrp->get_pp_string() << std::endl;


  unsigned int num_mboards = _usrp->get_num_mboards();
  _nRxChannels = _usrp->get_rx_num_channels();
  _nTxChannels = _usrp->get_tx_num_channels();
  
  std::cout << "Number mboards        = " << num_mboards << std::endl;
  std::cout << "Number of rx channels = " << _nRxChannels << std::endl;
  std::cout << "Number of tx channels = " << _nTxChannels << std::endl;
 


  // Initialize synchronization
  std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
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

      std::cout << boost::format("Device: %s ") % radio_conf.ip <<  std::endl;
      std::cout << boost::format("RX Rate: %f (%f) Msps") % (_usrp->get_rx_rate( i )/1e6) %  (tuner_conf.rate/1e6) << std::endl;
      std::cout << boost::format("RX Freq: %f (%f) MHz") % (_usrp->get_rx_freq( i )/1e6) % (tuner_conf.freq/1e6) << std::endl;
      std::cout << boost::format("RX Gain: %f (%f) dB") % _usrp->get_rx_gain( i ) % tuner_conf.gain << std::endl;
      std::cout << boost::format("RX Ant : %s (%s) ") % _usrp->get_rx_antenna( i ) % tuner_conf.ant << std::endl;
      std::cout << boost::format("RX BW: %f (%f) MHz") % (_usrp->get_rx_bandwidth( i )/1e6) % (tuner_conf.bw/1e6) << std::endl;
      std::cout << std::endl;
    }

  }

  boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

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

  _isRxInit = true;

} // End of init_rx


void CRadio::run_rx( DeviceStorageType& rx_mdst )
{
  boost::thread t(boost::bind( &CRadio::_run_rx, this, boost::ref(rx_mdst) ));
  _is_RxStreaming = true;
}

void CRadio::_run_rx(DeviceStorageType& rx_mdst)
{

  //create a receive streamer - populate channel list
  //linearly map channels (index0 = channel0, index1 = channel1, ...)
  uhd::stream_args_t stream_args("fc32"); //complex floats
  for (unsigned int i = 0; i < _nRxChannels; i++)
    stream_args.channels.push_back(i);
  assert( stream_args.channels.size() == _nRxChannels);

  _rx_stream = _usrp->get_rx_stream(stream_args);

  //setup streaming
  double delay_sec = 1.5;
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(delay_sec);
  _rx_stream->issue_stream_cmd(stream_cmd); //tells all channels to stream samples to host. This is the GO button.


  //the first call to recv() will block this many seconds before receiving
  double timeout = delay_sec + 0.1; //timeout (delay before receive + padding)
  boost::system_time next_console_refresh = boost::get_system_time();
  uhd::rx_metadata_t _md;

  while( _is_RxStreaming ) {
    //receive multi channel buffers
    size_t num_rx_samps = _rx_stream->recv( rx_mdst._MultiDeviceBufferPtrs.at(rx_mdst.head), _spb, _md, timeout);

    if (num_rx_samps != _spb) {
      std::cerr << "!!";
      continue;
    }
#if 0
    for(int i = 0; i < rx_mdst._MultiDeviceBuffer.at(0).size(); ++i) {
      float v = (float)i;
      rx_mdst._MultiDeviceBuffer.at(0).at(i) = std::complex<float>(v,v);
      rx_mdst._MultiDeviceBuffer.at(1).at(i) = std::complex<float>(1000+v,1000+v);
    }
#endif
    //use a small timeout for subsequent packets
    timeout = 0.01;

    rx_mdst._BufferTime.at( 0 ).full_sec = _md.time_spec.get_full_secs();
    rx_mdst._BufferTime.at( 0 ).frac_sec = _md.time_spec.get_frac_secs();

    // if storage buffers are full then dropp from tail end 
    if ( ((rx_mdst.head + 1) % rx_mdst.nbuffptrs ) == rx_mdst.tail)
      rx_mdst.tail = (rx_mdst.tail + 1) % rx_mdst.nbuffptrs;

    rx_mdst.head++;
    rx_mdst.head = rx_mdst.head % rx_mdst.nbuffptrs;


    //handle the error code
    if (_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
    else if (_md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) continue;
    else if (_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
      throw std::runtime_error(str(boost::format("Recv'd samples %i\nReceiver error %s") % num_rx_samps % _md.strerror()));
    }

#if 0
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
    std::cerr << "[" << _md.time_spec.get_full_secs() + _md.time_spec.get_frac_secs() << "] ";
    std::cerr << rx_mdst.head << " " << rx_mdst.tail << std::endl;
#endif

  } // while()


  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  stream_cmd.stream_now = true;
  _rx_stream->issue_stream_cmd(stream_cmd);


} // end of run_rx





void CRadio::run_rx(DeviceStorageType& rx_mdst, double seconds_in_future, size_t total_num_samps)
{
  //allocate buffers to receive with samples (one buffer per channel)
  rx_mdst.init( (unsigned int)_nRxChannels,
		(unsigned int)_spb,
		(unsigned int)ceil((float)total_num_samps / (float)_spb)) ;

  //_spb = _rx_stream->get_max_num_samps();


  //create a receive streamer - populate channel list
  //linearly map channels (index0 = channel0, index1 = channel1, ...)
  uhd::stream_args_t stream_args("fc32"); //complex floats
  for (unsigned int i = 0; i < _nRxChannels; i++)
    stream_args.channels.push_back(i);
  assert( stream_args.channels.size() == _nRxChannels);

  _rx_stream = _usrp->get_rx_stream(stream_args);

  //setup streaming
  std::cout << std::endl << boost::format("Begin streaming %u samples, %f seconds in the future...") % total_num_samps % seconds_in_future << std::endl;
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
  stream_cmd.num_samps = total_num_samps;
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
  _rx_stream->issue_stream_cmd(stream_cmd); //tells all channels to stream samples to host. This is the GO button.


  //the first call to recv() will block this many seconds before receiving
  double timeout = seconds_in_future + 0.1; //timeout (delay before receive + padding)

  size_t num_acc_samps = 0; //number of accumulated samples
    
  uhd::rx_metadata_t _md;
  while(num_acc_samps < total_num_samps) {
    //receive multi channel buffers
    size_t num_rx_samps = _rx_stream->recv( rx_mdst._MultiDeviceBufferPtrs.at(rx_mdst.head++), _spb, _md, timeout);
      
    //use a small timeout for subsequent packets
    timeout = 0.1;

    //handle the error code
    if (_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
    else if (_md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) continue;
    else if (_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
      throw std::runtime_error(str(boost::format("Recv'd samples %i\nReceiver error %s") % num_acc_samps % _md.strerror()));
    }
    int verbose = 1;
    if(verbose) std::cout << boost::format(
					   "Received packet: %u samples, %u full secs, %f frac secs"
					   ) % num_rx_samps % _md.time_spec.get_full_secs() % _md.time_spec.get_frac_secs() << std::endl;

    num_acc_samps += num_rx_samps;
  } // while()


  if (num_acc_samps < total_num_samps)  {
    std::cerr << "Receive timeout before all samples received..." << std::endl;
    return;
  }

} // end of run_rx( ... ) 


void CRadio::plot1(DeviceStorageType& rx_mdst, std::string udp_dst_addr, std::string port)
{
  std::cout << "Sending samples to " << udp_dst_addr << ":" << port << std::endl;
  // send out samples via udp_xport

  uhd::transport::udp_simple::sptr udp_xport = uhd::transport::udp_simple::make_connected(udp_dst_addr, port);

  udp_xport->send(boost::asio::buffer( &_nRxChannels , 4 ));
  size_t total_num_samps = rx_mdst.nsamps_per_ch;

  udp_xport->send(boost::asio::buffer( &total_num_samps , 4 ));

  unsigned int num_samps_per_datagram;
  //num_samps_per_datagram = (uhd::transport::udp_simple::mtu) / sizeof(std::complex<float>);
  num_samps_per_datagram = 256;

  // DEBUG replace floor with ceil
  unsigned int num_udp_datagrams = floor((float)total_num_samps / (float)(num_samps_per_datagram));
  udp_xport->send(boost::asio::buffer( &num_udp_datagrams , 4 ));


  for (unsigned int channel = 0; channel < _nRxChannels; ++channel) {
    std::complex<float> *ptr_to_device_buffer = (std::complex<float> *)rx_mdst._MultiDeviceBuffer.at(channel).data();
    for (unsigned int i = 0; i < num_udp_datagrams; ++i) {
      udp_xport->send(boost::asio::buffer((void*)ptr_to_device_buffer,
					  num_samps_per_datagram * sizeof(std::complex<float>) )
		      );
      ptr_to_device_buffer += num_samps_per_datagram;
      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
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

      std::cout << boost::format("Device: %s ") % radio_conf.ip <<  std::endl;
      std::cout << boost::format("TX Rate: %f (%f) Msps") % (_usrp->get_tx_rate( i )/1e6) %  (tuner_conf.rate/1e6) << std::endl;
      std::cout << boost::format("TX Freq: %f (%f) MHz") % (_usrp->get_tx_freq( i )/1e6) % (tuner_conf.freq/1e6) << std::endl;
      std::cout << boost::format("TX Gain: %f (%f) dB") % _usrp->get_tx_gain( i ) % tuner_conf.gain << std::endl;
      std::cout << boost::format("TX Ant : %s (%s) ") % _usrp->get_tx_antenna( i ) % tuner_conf.ant << std::endl;
      std::cout << boost::format("TX BW: %f (%f) MHz") % (_usrp->get_tx_bandwidth( i )/1e6) % (tuner_conf.bw/1e6) << std::endl;
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

  _isTxInit = true;

} // End of init_tx



void CRadio::run_tx( DeviceStorageType& tx_mdst )
{
  boost::thread t(boost::bind( &CRadio::_run_tx, this, boost::ref(tx_mdst) ));
  _is_TxStreaming = true;
}


void CRadio::_run_tx(DeviceStorageType& tx_mdst)
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
  boost::system_time next_console_refresh = boost::get_system_time();
  boost::system_time next_refresh = boost::get_system_time();

  uhd::tx_metadata_t md;   //setup the metadata flags
  md.start_of_burst = true;
  md.end_of_burst   = false;
  md.has_time_spec  = true;
  md.time_spec = uhd::time_spec_t(0.1);

  while( _is_TxStreaming ) {
    
    if (boost::get_system_time() < next_refresh) continue;
    next_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(50));

    if (tx_mdst.head == 0) {
      _tx_stream->send((void*)zero_buf.data(), _spb, md);
      md.start_of_burst = false;
      md.has_time_spec = false;
      continue;
    }

    // send multi channel buffers to device
    for (unsigned int idx = 0; idx < tx_mdst.head; idx++)
    {
      _tx_stream->send(tx_mdst._MultiDeviceBufferPtrs.at(idx), _spb, md);
      md.start_of_burst = false;
      md.has_time_spec = false;
    }
    
    // Comment line below to keep sending sequence
    //tx_mdst.head = 0;

  } // while()

  // ???? send a mini EOB packet
  md.end_of_burst = true;
  _tx_stream->send("", 0, md);


}
