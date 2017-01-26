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

//#define UDP2OCTAVE(ptr, nbytes)    UDPSimple_send((void*)ptr, nbytes);

#if 1
//void UDPSimple_send(void *ptr, unsigned int nbytes)
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
#endif

LoggerPtr logger(Logger::getLogger("sigproc")); // replace with name of this file
bool local_kill_switch = false;

void signal_interrupt_control_c(int s){
  printf("Caught signal %d\n",s);
  local_kill_switch = true;
}

namespace po = boost::program_options;


std::vector<std::complex<float> > t_sync_ref;
std::vector<std::complex<float> > f_sync_ref;

void rx_handler_find_idx(CDeviceStorage& rx_mdst, unsigned int run_time)
{

  boost::this_thread::sleep(boost::posix_time::milliseconds(100));

  unsigned int spb = rx_mdst.spb();
  unsigned int nRxChannels = rx_mdst.nChannels();

  std::vector<float> mag_buff( spb );
  std::vector<std::complex<float> > fft_buff( spb );
  assert( fft_buff.size() == spb);

  unsigned int total_num_samps = rx_mdst.nsamps_per_ch();

  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );

  // noise floor measurement
  bool nf_ready = false;
  std::vector<float> mag4hist(500);
  float mag4_threshold = 0.0;
  unsigned int nf_idx = 0;

  // sync stuff
  unsigned int state = 0;
  float mag_4m1, mag_14m1;
  std::vector<std::complex<float> > fft_m4_buff(spb*4);
  std::vector<std::complex<float> > f_corr_m4_buff(spb*4);
  std::vector<std::complex<float> > t_corr_m4_buff(spb*4);


  t_sync_ref.resize( spb*4 );  // there is alread 256 samples in here. Verify resizing retains existing values.
  f_sync_ref.resize( spb*4 );
  fftwf_plan fft_sync_ref_p = fftwf_plan_dft_1d(spb*4,
						(fftwf_complex*)&t_sync_ref.front(),
						(fftwf_complex*)&f_sync_ref.front(),
						FFTW_FORWARD,
						FFTW_ESTIMATE);
  fftwf_execute(fft_sync_ref_p);  // should be in f_sync_ref
  fftwf_plan ifft_corr_p = fftwf_plan_dft_1d(spb*4,
					     (fftwf_complex*)&f_corr_m4_buff.front(),
					     (fftwf_complex*)&t_corr_m4_buff.front(),
					     FFTW_BACKWARD,
					     FFTW_ESTIMATE);




  while ((!local_kill_switch) && (boost::get_system_time() < done_time)) {

    if (rx_mdst.tail() == rx_mdst.head() ) continue;

    unsigned int ch = 0; // this is the channel to trigger on

    fftwf_plan fft_p = fftwf_plan_dft_1d( spb,
					  (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst.tail()),
					  (fftwf_complex*)&fft_buff.front(),
					  FFTW_FORWARD,
					  FFTW_ESTIMATE);
    fftwf_execute(fft_p);

    fftwf_destroy_plan(fft_p);

    // magnitude
    for(int i = 0; i < spb; i++)
      mag_buff.at(i) = abs(fft_buff.at(i));

    // before doing anything else calc  noise level statistics for bin 4
    // and set bin 4 detection threshold
    if (nf_ready != true) { 
      
      mag4hist.at( nf_idx++ ) = mag_buff.at(4);
      if ( nf_idx == mag4hist.size() ) { 
      
	nf_ready = true;
	std::cerr << "nf ready!" << std::endl;

	std::sort (mag4hist.begin(), mag4hist.end(), sort_f32_asc);
	//for(int i = 0; i < mag4hist.size(); i++)  std::cerr << mag4hist.at(i) << std::endl;

	float sum = 0.0;
	float sum_sq = 0.0;
	float avg, variance, stddev;
	for(int i = 0; i < mag4hist.size()-1; i++) {
	  sum += mag4hist.at(i);
	  sum_sq += mag4hist.at(i) * mag4hist.at(i);
	}
	avg = sum / ( mag4hist.size()-1) ;
	variance = sum_sq / ( mag4hist.size()-1)  - (avg * avg);
	stddev = sqrt(variance);
	std::cerr << "nf mag4 avg: " << avg << std::endl;
	std::cerr << "nf mag4 var: " << variance << std::endl;
	std::cerr << "nf mag4 std: " << stddev << std::endl;
	
	float n_stddevs = 10.0;
	mag4_threshold = avg + n_stddevs * stddev;
	std::cerr << "mag4 thres = " << mag4_threshold << " --- this is " << n_stddevs << " stddev away from mean" << std::endl;
      }
    }



    // find max value index                               - asm inline this
    unsigned int pki = std::distance(mag_buff.begin(),
				     std::max_element(mag_buff.begin(), mag_buff.end()) );

    if ((nf_ready == true) && 
	(state == 0) && 
	//(pki == 4) &&
	(mag_buff.at(4) > mag4_threshold) &&
	(rx_mdst.tail() < (rx_mdst.nbuffptrs() - 4) ))  // check roll over cond at tail - simple method
      {
	mag_4m1   = mag_buff.at(4);
	mag_14m1  = mag_buff.at(14);
	LOG4CXX_INFO(logger, "ch | tail | head | pki | mag " << ch << " " << rx_mdst.tail() << " " << rx_mdst.head() << " " << pki << " " << mag_buff.at(pki));

	state = 1;
      }
    else if ((nf_ready == true) && 
	     (state == 1)  ) {
      // check tail buffer idx @ -1 0
      std::cerr << mag_4m1   << " " << mag_14m1  << std::endl;
      std::cerr << mag_buff.at(4)  << " " << mag_buff.at(14) << std::endl;


      fftwf_plan fft_1024 = fftwf_plan_dft_1d( spb*4,
					       (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst.tail(-2)),
					       (fftwf_complex*)&fft_m4_buff.front(),
					       FFTW_FORWARD,
					       FFTW_ESTIMATE);

      // take 1024 FFT on signal from idx
      fftwf_execute( fft_1024 );  // should be in fft_m4_buff
      fftwf_destroy_plan( fft_1024 );

      for (int i = 0; i < spb*4; i++)
	f_corr_m4_buff.at(i) = fft_m4_buff.at(i) * std::conj(f_sync_ref.at(i));
      
      fftwf_execute( ifft_corr_p );  // should be in t_corr_m4_buff

      //find max value index
      unsigned int sample_corr_idx = std::distance(t_corr_m4_buff.begin(),
						   std::max_element(t_corr_m4_buff.begin(), t_corr_m4_buff.end(), mag_compare) );

      DBG_OUT(sample_corr_idx);

      plot_cf32_buffer("10.10.0.10", "1337",  rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 0, rx_mdst.tail(-2)), spb * 4);
      plot_cf32_buffer("10.10.0.10", "1337",  rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 1, rx_mdst.tail(-2)), spb * 4);
      plot_cf32_buffer("10.10.0.10", "1337",  rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 2, rx_mdst.tail(-2)), spb * 4);

      //usrp1.plot_buffer("10.17.0.10", "1337", rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 0, rx_mdst.tail(-2)), spb * 4);
      //usrp1.plot_buffer("10.10.0.10", "1337", rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 1, rx_mdst.tail(-2)), spb * 4);
      //usrp1.plot_buffer("10.10.0.10", "1337", rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 2, rx_mdst.tail(-2)), spb * 4);
      LOG4CXX_INFO(logger, "tail | head : " << rx_mdst.tail() << " " << rx_mdst.head() );;


      //usrp1.plot_rx("10.10.0.10", "1337", rx_mdst._tail-2, 4);
      local_kill_switch = true;
      state = 0;
    }
   
    // increment tail for next round
    rx_mdst.tail_inc();
    //rx_mdst._tail = rx_mdst._tail % rx_mdst.nbuffptrs();

    //boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1000e3));

    DBG_OUT(mag_buff.at(4));
    //DBG_OUT(rx_mdst._head);
    //DBG_OUT(rx_mdst._tail);

  } // while

} // rx_handler_find_idx




int main(int argc, char *argv[])
{
  std::string addr, port;
  unsigned int spb, bin;
  unsigned int intv;
  unsigned int profile;
  std::vector<std::string> str_opts;
  CDeviceStorage rx_mdst;
  CDeviceStorage tx_mdst;
  std::string sync_samps_str;

  //setup the program options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "brief description of get/set handlers")
    ("opts",  po::value<std::vector<std::string> >(&str_opts), "optional handlers")
    ("sync", po::value<std::string>(&sync_samps_str), "cf32 sync signal")
    //("spec", "[use with --func rx] find peaks from magnitude fft")
    ("intv", po::value<unsigned int>(&intv),"repeat every 'intv' usec")
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
  
#if 0
  if (rx_mdst.check_shm("/ShmMultiDeviceBufferRx") == false )
  {
    std::cerr << "shm unavail.\n";
    return ~0;
  }
#endif
  rx_mdst.attach_shm("/ShmMultiDeviceBufferRx");
  rx_mdst.print_info();

  if (vm.count("sync")) {
    std::vector<std::string> tokens;
    boost::split(tokens, sync_samps_str, boost::is_any_of("| "), boost::token_compress_on);
    for (unsigned int i = 0; i < tokens.size() ; i++) {
      std::complex<float> cf = boost::lexical_cast<std::complex<float> >( tokens.at(i) );
      t_sync_ref.push_back(cf);
      //std::cout << tokens.at(i) << " " << cf << std::endl;
    }
    assert(tokens.size() == t_sync_ref.size() );
    assert(t_sync_ref.size() == rx_mdst.spb() );
  }

  // test here
  //plot_cf32_buffer("10.10.0.10", "1337", t_sync_ref.data(), t_sync_ref.size() );
  //plot_cf32_buffer("10.10.0.10", "1337", t_sync_ref.data(), t_sync_ref.size() );
  //return ~0;
  //

  unsigned int run_time = 10;
  rx_handler_find_idx(rx_mdst, run_time);

  boost::system_time next_console_refresh = boost::get_system_time();

  unsigned int count_processed_buffers = 0;
  while(!local_kill_switch) {

    if (boost::get_system_time() > next_console_refresh) {
      count_processed_buffers = 0;
      next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
      std::cerr << ".";
    }

  } // while (true)

  std::cerr << "Done!" << std::endl;
  return ~0;
}

