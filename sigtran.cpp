#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>


#include <boost/thread/thread_time.hpp>
#include "CDeviceStorage.hpp"
//#include <fftw3.h>
#include <signal.h>

#include "UDPSimple.hpp"
#include "CWriteOml.h"
#include "CTimer.h"
#include "TCPSimple.hpp"
#include <fstream>


namespace po = boost::program_options;

#define READ_VECTOR(readFileName, buffer)				\
  {									\
    std::ifstream is (readFileName, std::ifstream::binary);		\
    if (!is)								\
    {									\
      std::cout << "Error openning file: " << readFileName <<std::endl;	\
    }									\
    else								\
    {						                        \
      unsigned int numberOfBytes;					\
      unsigned int typeOfData;						\
      is.read((char*) &numberOfBytes, sizeof(numberOfBytes));		\
      is.read((char*) &typeOfData,    sizeof(typeOfData));		\
      buffer.resize(1);							\
      unsigned int nElements = numberOfBytes / sizeof(buffer.at(0));	\
      buffer.resize(nElements);						\
      is.read ((char*) &buffer.front(), numberOfBytes);			\
    }									\
  }


void tx_handler_sync_plus_ofdm(CDeviceStorage& tx_mdst, unsigned int run_time, unsigned int intv_us);


bool local_kill_switch = false;

void signal_interrupt_control_c(int s){
  printf("Caught signal %d\n",s);
  local_kill_switch = true;
}



void plot_cf32_buffer(std::string udp_dst_addr, std::string port, void *ptr_cf32, unsigned int n_cf32_values)
{
  static UDPSimple udp_xport(udp_dst_addr, port);

  // send number of channels (or streams)
  unsigned int Channels = 1;
  udp_xport.send( (void*)&Channels, sizeof(Channels) );

  // send number of cf32 samples to receive
  udp_xport.send( (void*)&n_cf32_values, sizeof(n_cf32_values) );

  unsigned int _spb = 256;
  unsigned int num_samps_per_datagram = _spb; //num_samps_per_datagram = (uhd::transport::udp_simple::mtu) / sizeof(std::complex<float>);

  // send number of datagrams
  unsigned int num_udp_datagrams = floor((float)n_cf32_values / (float)(num_samps_per_datagram));
  udp_xport.send((void*)&num_udp_datagrams, sizeof(num_udp_datagrams));

  std::complex<float> *ptr_to_buffer = (std::complex<float> *)ptr_cf32;
  for (unsigned int i = 0; i < num_udp_datagrams; ++i) {
    udp_xport.send( (void*)ptr_to_buffer, num_samps_per_datagram * sizeof(std::complex<float>) );
    ptr_to_buffer += num_samps_per_datagram;
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}


std::vector<std::complex<float> > tx_signal;

//
// Transmit function.
// Example
//
void tx_handler_my_signal(CDeviceStorage& tx_mdst, unsigned int run_time, unsigned int intv_us)
{
  unsigned int spb = tx_mdst.spb();
  unsigned int nTxChannels = tx_mdst.nChannels();
  unsigned int total_num_samps = tx_mdst.nsamps_per_ch();
  unsigned int n_spb_buffs = tx_mdst.nbuffptrs();
  //DBG_OUT(spb); DBG_OUT(nTxChannels); DBG_OUT(total_num_samps); DBG_OUT (n_spb_buffs);


  // copy signal to tx_mdst buffers for sending 
  for (unsigned int ch = 0; ch < nTxChannels; ch++) {
    // replicate signal to all (N_SPB_BUFFS) buffers per chan
    for (unsigned int idx = 0; idx < 1; idx++) {
      memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
	     (void*)tx_signal.data(), spb * sizeof(std::complex<float>) );
    }
  }

  for (unsigned int ch = 0; ch < nTxChannels; ch++) {
    // replicate signal to all (N_SPB_BUFFS) buffers per chan
    for (unsigned int idx = 1; idx < n_spb_buffs; idx++) {
      memset((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx), 0, spb * sizeof(std::complex<float>));
    }
  }

  while( !local_kill_switch ) { 
    if (tx_mdst.head() != 0) continue;

    // if signal is constant then generate once above the while conditional.
    // if sign is NOT constant then update signal here and copy to tx_mdst buffers.
    // To be tried out :P


    // send signal in intervals
    boost::this_thread::sleep(boost::posix_time::microseconds(intv_us));

    tx_mdst.head_set( n_spb_buffs ); // kick off sending
    //std::cerr << ".";
  }

}


int main(int argc, char *argv[])
{
  unsigned int intv_us, run_time;
  CDeviceStorage tx_mdst;
  std::string sig_samples_str;

  //setup the program options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "brief description of get/set handlers")
    ("sig", po::value<std::string>(&sig_samples_str), "cf32 samples")
    ("time", po::value<unsigned int>(&run_time)->default_value(10), "run time in seconds")
    ("intv", po::value<unsigned int>(&intv_us),"dwell 'intv' usec between transmits")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  //print the help message
  if (vm.count("help")){
    std::cerr << desc << std::endl;
    return ~0;
  }

  signal (SIGINT, signal_interrupt_control_c);
  
  tx_mdst.attach_shm("/ShmMultiDeviceBufferTx");
  tx_mdst.print_info();

  if (vm.count("sig")) {
    std::vector<std::string> tokens;
    boost::split(tokens, sig_samples_str, boost::is_any_of("| "), boost::token_compress_on);
    for (unsigned int i = 0; i < tokens.size() ; i++) {
      std::complex<float> cf = boost::lexical_cast<std::complex<float> >( tokens.at(i) );
      tx_signal.push_back(cf);
      //std::cout << tokens.at(i) << " " << cf << std::endl;
    }
    assert(tokens.size() == tx_signal.size() );
    assert(tx_signal.size() == tx_mdst.spb() );
  }


  // plot the transmit signal
  // plot_cf32_buffer("10.10.0.10", "1337",  (void*)tx_signal.data(), tx_mdst.spb() );

  //tx_handler_my_signal(tx_mdst, run_time, intv_us);
  tx_handler_sync_plus_ofdm(tx_mdst, run_time, intv_us);

  std::cerr << "Done!" << std::endl;
  return ~0;
}




//
// Transmit function.
// Create your own signal in OFDMPacketRandomSymbols256.dat and send
//
void tx_handler_sync_plus_ofdm(CDeviceStorage& tx_mdst, unsigned int run_time, unsigned int intv_us)
{
  unsigned int spb = tx_mdst.spb();
  unsigned int nTxChannels = tx_mdst.nChannels();
  unsigned int total_num_samps = tx_mdst.nsamps_per_ch();
  unsigned int n_spb_buffs = tx_mdst.nbuffptrs();
  //DBG_OUT(spb); DBG_OUT(nTxChannels); DBG_OUT(total_num_samps); DBG_OUT (n_spb_buffs);


  // read in ofdm signal from file
  std::vector<std::complex<float> > ofdm_signal;

  //std::ifstream is ("OFDMPacketRandomSymbols256.dat", std::ifstream::binary);
  std::ifstream is ("my_signal_256.dat", std::ifstream::binary);
  is.seekg (0, is.end);
  unsigned int length = is.tellg();
  is.seekg (0, is.beg);
  unsigned int nSamples = length / sizeof(std::complex<float>);
  ofdm_signal.resize( nSamples );
  is.read ( (char*)ofdm_signal.data(), length);

  // plot the signal
  // plot_cf32_buffer("10.10.0.10", "1337",  (void*)ofdm_signal.data(), ofdm_signal.size() );


  //nTxChannels = 1;
  // idx 0: sync signal 256 samples
  for (unsigned int ch = 0; ch < nTxChannels; ch++) {
    // replicate signal to all (N_SPB_BUFFS) buffers per chan
    for (unsigned int idx = 0; idx < 1; idx++) {
      memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
	     (void*)tx_signal.data(), spb * sizeof(std::complex<float>) );
    }
  }

  // idx 1..2: ofdm signal 256 samples
  for (unsigned int ch = 0; ch < nTxChannels; ch++) {
    // just insert 1 copy of the ofdm signal
    for (unsigned int idx = 1; idx < n_spb_buffs; idx++) {
      memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
	     (void*)ofdm_signal.data(), spb * sizeof(std::complex<float>) );
    }
  }
#if 0
  // idx 3..n_spb_buffs: zeros
  for (unsigned int ch = 0; ch < nTxChannels; ch++) {
    // replicate signal to all (N_SPB_BUFFS) buffers per chan
    for (unsigned int idx = 8; idx < n_spb_buffs; idx++) {
      memset((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx), 0, spb * sizeof(std::complex<float>));
    }
  }
#endif


#if 0
  for (unsigned int ch = 0; ch < nTxChannels; ch++) {
    memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, 1),
	   (void*)ofdm_signal.data(), ofdm_signal.size() * sizeof(std::complex<float>) );
  }
#endif

  while( !local_kill_switch ) { 
    if (tx_mdst.head() != 0) continue;

    // if signal is constant then generate once above the while conditional.
    // if sign is NOT constant then update signal here and copy to tx_mdst buffers.
    // To be tried out :P


    // send signal in intervals
    boost::this_thread::sleep(boost::posix_time::microseconds(intv_us));
    tx_mdst.head_set( n_spb_buffs ); // kick off sending
    std::cerr << ".";
  }

}
