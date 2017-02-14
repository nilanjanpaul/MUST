#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>


#include <boost/thread/thread_time.hpp>
#include "CDeviceStorage.hpp"
#include <fftw3.h>
#include <signal.h>

#include "log4cxx/logger.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/helpers/exception.h"

//#include <fstream>
#include "UDPSimple.hpp"
#include "CWriteOml.h"
#include "CTimer.h"
#include "TCPSimple.hpp"

using namespace log4cxx;
using namespace log4cxx::helpers;


// octave commands:							\
// > rcv_port = 1337; rcv_sck = fUDP_open_rx(rcv_port);			\
// > [recv_data, recv_count]=recv(rcv_sck,4000,MSG_DONTWAIT); y = typecast(uint8(recv_data), 'single complex'); \

static bool sort_f32_asc (float i,float j) { return (i<j); }

static bool mag_compare(std::complex<float> a, std::complex<float> b)
{
  return (std::abs(a) < std::abs(b));
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

LoggerPtr logger(Logger::getLogger("sigproc")); // replace with name of this file
bool local_kill_switch = false;

void signal_interrupt_control_c(int s){
  printf("Caught signal %d\n",s);
  local_kill_switch = true;
}
namespace po = boost::program_options;

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
    for (unsigned int idx = 0; idx < n_spb_buffs; idx++) {
      memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
	     (void*)tx_signal.data(), spb * sizeof(std::complex<float>) );
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
  std::string addr, port;
  unsigned int spb, bin;
  unsigned int intv_us, run_time;
  unsigned int profile;
  CDeviceStorage tx_mdst;
  std::string sig_samples_str;

  //setup the program options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "brief description of get/set handlers")
    ("sig", po::value<std::string>(&sig_samples_str), "cf32 samples")
    ("time", po::value<unsigned int>(&run_time)->default_value(10), "run time in seconds")
    ("intv", po::value<unsigned int>(&intv_us),"repeat every 'intv' usec")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  //print the help message
  if (vm.count("help")){
    std::cerr << desc << std::endl;
    return ~0;
  }

  // Configure the logger
  PropertyConfigurator::configure("./logconf.prop");

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

  tx_handler_my_signal(tx_mdst, run_time, intv_us);


  std::cerr << "Done!" << std::endl;
  return ~0;
}

