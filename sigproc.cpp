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

#include <fstream>
#include "UDPSimple.hpp"
#include "CWriteOml.h"
#include "CTimer.h"
#include "TCPSimple.hpp"

using namespace log4cxx;
using namespace log4cxx::helpers;



using boost::asio::ip::udp;
typedef boost::shared_ptr<tcp::socket> socket_ptr;
void command_server(short port);
void command_session(socket_ptr sock);
int nCmdThreadCnt = 0;

CDeviceStorage rx_mdst;
std::vector<std::vector<std::complex<float> > > snapshot256;


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



//
// Receiving end function.
// Example: find approx. bandwidth power
//
void rx_handler_find_avg_bw_pwr(CDeviceStorage& rx_mdst, unsigned int run_time)
{
  unsigned int spb = rx_mdst.spb();
  unsigned int nRxChannels = rx_mdst.nChannels();
  unsigned int total_num_samps = rx_mdst.nsamps_per_ch();

  std::vector<float> mag_buff( spb );
  std::vector<std::complex<float> > fft_buff( spb );
  assert( fft_buff.size() == spb );

  double freq, rate, gain;

  // get params freq rate gain
  {
    TCPSimple tcp_xport("localhost","5111");
    std::string reply;

    tcp_xport.send_n_recv("rxchan all", reply);
    //std::cout << reply;
    // reply constructed in rf_hw_intf command thread
    // rxchan 0 900000000 2000000 10
    // rxchan 1 900000000 2000000 10
    // rxchan 2 900000000 2000000 10
    //

    std::vector<std::string> lines, tokens;
    boost::split(lines, reply, boost::is_any_of("\n"), boost::token_compress_on);
    assert(lines.size() > 1 );

    boost::split(tokens, lines.at(0), boost::is_any_of(" "), boost::token_compress_on);

    assert(tokens.size() == 5 );
    freq = boost::lexical_cast<double>( tokens.at(2) );
    rate = boost::lexical_cast<double>( tokens.at(3) );
    gain = boost::lexical_cast<double>( tokens.at(4) );

  }


#if 1
  //  OML writer - data base initialization
  CWriteOml OML;
  std::string omlDbFilename("measured_avg_bw_pwr");
  std::string omlServerName("oml:3003");
  OML.init(omlDbFilename, omlServerName );

  //  OML writer - create keys with associated type
  std::vector< std::pair<std::string, OmlValueT> > _omlKeys;
  _omlKeys.push_back( std::make_pair("sampling",    OML_DOUBLE_VALUE) );
  _omlKeys.push_back( std::make_pair("cfreq_MHz",   OML_DOUBLE_VALUE) );
  _omlKeys.push_back( std::make_pair("gain_dB",     OML_DOUBLE_VALUE));
  _omlKeys.push_back( std::make_pair("FFTLength",   OML_INT32_VALUE));
  //_omlKeys.push_back( std::make_pair("nRxChannels", OML_INT32_VALUE));
  _omlKeys.push_back( std::make_pair("Channel",     OML_INT32_VALUE));
  _omlKeys.push_back( std::make_pair("AvgPwr",     OML_DOUBLE_VALUE));
  _omlKeys.push_back( std::make_pair("ExecTime",    OML_DOUBLE_VALUE));
  OML.start( _omlKeys );


  //  OML writer - assign key value
  OML.set_key("sampling", (void*)&rate );
  OML.set_key("cfreq_MHz", (void*)&freq );
  OML.set_key("gain_dB", (void*)&gain );
  OML.set_key("FFTLength", (void*)&total_num_samps);
  //OML.set_key("nRxChannels", (void*)&nRxChannels);

  //double estimate = -99.0;
  //OML.set_key("ExecTime", (void*)&estimate);

#endif
  
  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );

  while ((!local_kill_switch) && (boost::get_system_time() < done_time)) {
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));

    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
    
    while (  !(rx_mdst.head(+1) == rx_mdst.tail()) );

    rx_mdst.print_time();
    //DBG_OUT(rx_mdst.head()); DBG_OUT(rx_mdst.tail());


    // find peaks for different channels
    for (unsigned int ch = 0; ch < nRxChannels; ch++) {
      std::vector<float> mag_buff( total_num_samps );
      std::vector<std::complex<float> > fft_buff( total_num_samps );
      fftwf_plan fft_p = fftwf_plan_dft_1d( total_num_samps,
					    (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( ch, 0 ),
                                            (fftwf_complex*)&fft_buff.front(),
                                            FFTW_FORWARD,
                                            FFTW_ESTIMATE);


      float sum_mag = 0.0;
      {
      CTimer tmr("64k fft, mag, norm, sum-----");
      fftwf_execute(fft_p); // FWD FFT is NOT normalized so divide by N

      // magnitude - ignore frequencies around DC component
      for(int i = 10; i < total_num_samps-10; i++)
        mag_buff.at(i) = abs(fft_buff.at(i));


      // sum of magnitudes
      for(int i = 10; i < total_num_samps-10; i++)
	sum_mag += mag_buff.at(i);

      sum_mag /= ((float)total_num_samps * (float)total_num_samps) ;  // for FWD FFT normalization. mag values has N^2 to factor out.

      sum_mag /= (total_num_samps - 20); // for pwr calculation
      }
      double log_pwr = 10*log10( sum_mag);

      std::cerr << "ch: " << ch << " --- avg pwr = " << log_pwr << std::endl;

#if 0
      // find first 15 peaks
      double avg_pw = 0.0;
      for (unsigned int i = 0; i < 15 ; ++i)
	{
	  // find max value index                               - asm inline this
	  unsigned int pki = std::distance(mag_buff.begin(),
					   std::max_element(mag_buff.begin(), mag_buff.end()) );
	  std::cerr << pki << ": " << mag_buff.at(pki) / total_num_samps << ": " << pki* usrp1.rx_rate(ch) / (total_num_samps) <<  std::endl;

	  avg_pw += mag_buff.at(pki) / total_num_samps;
	  // now zero out this peak
	  mag_buff.at(pki) = 0.0;
	}
      std::cerr << "ch: " << ch << " --- avg pwr = " << 10*log10( avg_pw) << std::endl;
#endif

#if 1
      //  OML writer - assign key value
      OML.set_key("Channel", (void*)&ch);
      OML.set_key("AvgPwr", (void*)&log_pwr);
      
      //  OML writer - insert into database
      OML.insert();
#endif
    } // for loop channels

  } // while ((!local_kill_switch) && (boost::get_system_time() < done_time))

}





//
// Receiving end function.
// Example to find start of signal
//

std::vector<std::complex<float> > t_sync_ref;
std::vector<std::complex<float> > f_sync_ref;

void rx_handler_find_signal_start_idx(CDeviceStorage& rx_mdst, unsigned int run_time)
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
  boost::system_time next_sync_time = boost::get_system_time() + boost::posix_time::microseconds(long(2.0e6));


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
  //plot_cf32_buffer("10.10.0.10", "1337",  (void*)ofdm_signal.data(), ofdm_signal.size() );


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

    unsigned int trigger_ch = 3; // this is the channel to trigger on

    fftwf_plan fft_p = fftwf_plan_dft_1d( spb,
					  (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( trigger_ch, rx_mdst.tail()),
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
        !(boost::get_system_time() < next_sync_time) &&
	(mag_buff.at(4) > mag4_threshold) &&
	(rx_mdst.tail() < (rx_mdst.nbuffptrs() - 4) ))  // check roll over cond at tail - simple method
    {
      mag_4m1   = mag_buff.at(4);
      mag_14m1  = mag_buff.at(14);
      LOG4CXX_INFO(logger, "ch | tail | head | pki | mag " << trigger_ch << " " << rx_mdst.tail() << " " << rx_mdst.head() << " " << pki << " " << mag_buff.at(pki));

      state = 1;
    }
    else if ((nf_ready == true) && 
	     (state == 1)  ) {

      // wait to accumualte enough samples in queue
      if ( rx_mdst.depth() > 5 ) state = 2;
      continue; // ==> do NOT increment tail pointer
    }
    else if ((nf_ready == true) && 
	     (state == 2)  ) {

      // once a sync seq has been detected do not expect another sync sequence for specificed duration
      next_sync_time = boost::get_system_time() + boost::posix_time::microseconds(long(3.0e6));
      //rx_mdst.pause( true );

      // check tail buffer idx @ -1 0
      //std::cerr << mag_4m1   << " " << mag_14m1  << std::endl;
      //std::cerr << mag_buff.at(4)  << " " << mag_buff.at(14) << std::endl;

      std::vector<unsigned int> corr_idx(nRxChannels);
      // find coorelation idx for all channels
      for (unsigned int ch = 0; ch < nRxChannels; ch++) {
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
	corr_idx.at(ch) = std::distance(t_corr_m4_buff.begin(),
					std::max_element(t_corr_m4_buff.begin(), t_corr_m4_buff.end(), mag_compare) );

	if ( (ch != trigger_ch) && (corr_idx.at(ch) >= (spb*4 - 128)))
	     corr_idx.at(ch) = 0;

	DBG_OUT(corr_idx.at(ch));

      } // for (unsigned int ch = 0; ch < nRxChannels; ch++)


#if 0
      for (unsigned int ch = 0; ch < nRxChannels; ch++) {
	// channel (ch) signal starts at address rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst.tail(-2)) + sample_corr_idx + spb
	std::complex<float> *cf32_ptr;
	cf32_ptr = (std::complex<float> *) rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst.tail(-2)) + corr_idx.at( ch ) + spb;
	std::vector<std::complex<float> > theta_ch0(spb);
	for (unsigned int i = 0 ; i < spb; i++)
	  theta_ch0.at(i) = *(cf32_ptr+i) / ofdm_signal.at(i);
	//plot_cf32_buffer("10.10.0.10", "1337",  theta_ch0.data(), spb);

	cf32_ptr = (std::complex<float> *) rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst.tail(-2)) + corr_idx.at( ch ) + spb*2;
	for (unsigned int i = 0 ; i < spb; i++)
	  theta_ch0.at(i) = *(cf32_ptr+i) / ofdm_signal.at(i);
	//plot_cf32_buffer("10.10.0.10", "1337",  theta_ch0.data(), spb);
      }
#endif

      // record shapshot of 256 samples / per ch  for  viewing pleasure
      for (unsigned int ch = 0; ch < nRxChannels; ch++) {
	std::complex<float> *cf32_ptr = (std::complex<float> *) rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst.tail(-2)) + corr_idx.at( ch ) + spb;
	memcpy ( (void*)snapshot256.at(ch).data(), (void*)cf32_ptr, spb * sizeof(std::complex<float> ) );
      }

      //plot_cf32_buffer("10.10.0.10", "1337",  cf32_ptr, spb * 4);
      //plot_cf32_buffer("10.10.0.10", "1337",  (std::complex<float> *)rx_mdst.buffer_ptr_ch_idx_( 0, rx_mdst.tail(-2)) , spb * 4);
      //plot_cf32_buffer("10.10.0.10", "1337",  (std::complex<float> *)rx_mdst.buffer_ptr_ch_idx_( 3, rx_mdst.tail(-2)) , spb * 4);
      //plot_cf32_buffer("10.10.0.10", "1337",  (std::complex<float> *)rx_mdst.buffer_ptr_ch_idx_( 4, rx_mdst.tail(-2)) , spb * 4);
      LOG4CXX_INFO(logger, "tail | head : " << rx_mdst.tail() << " " << rx_mdst.head() );

      //local_kill_switch = true;
      state = 0;
      //rx_mdst.pause(false);

    }
   
    // increment tail for next round
    rx_mdst.tail_inc();
    //rx_mdst._tail = rx_mdst._tail % rx_mdst.nbuffptrs();

    //boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1000e3));

  } // while

  std::cerr << "time up - exitting..." << std::endl;


} // rx_handler_find_signal_start_idx



int main(int argc, char *argv[])
{
  std::string addr, port;
  unsigned int spb, bin;
  unsigned int intv, run_time;
  unsigned int profile;
  std::string sync_samps_str;
  short command_server_port;

  //setup the program options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "brief description of get/set handlers")
    ("sync", po::value<std::string>(&sync_samps_str), "cf32 sync signal")
    ("time", po::value<unsigned int>(&run_time)->default_value(10), "run time in seconds")
    ("cmd-port", po::value<short>(&command_server_port)->default_value(5180),"command server port")

    //("intv", po::value<unsigned int>(&intv),"repeat every 'intv' usec")
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
  
  boost::thread cmds(command_server, command_server_port);


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

  snapshot256.resize( rx_mdst.nChannels() );
  for (unsigned int ch = 0; ch < snapshot256.size(); ch++)
    snapshot256.at(ch).resize (rx_mdst.spb());


  // test here
  //plot_cf32_buffer("10.10.0.10", "1337", t_sync_ref.data(), t_sync_ref.size() );
  //plot_cf32_buffer("10.10.0.10", "1337", t_sync_ref.data(), t_sync_ref.size() );
  //return ~0;
  //
#ifdef COMPILE_SIG_START_IDX
  rx_handler_find_signal_start_idx(rx_mdst, run_time);
#endif

#ifdef COMPILE_AVG_BW_PWR
  rx_handler_find_avg_bw_pwr(rx_mdst, run_time);
#endif

  //cmds.join();

  std::cerr << "Done!" << std::endl;
  return ~0;
}


void command_session(socket_ptr sock)
{
  std::vector<unsigned char> command(256);
  boost::system::error_code error;
  unsigned int N_SAMPS_TO_SEND = 32;

  try
  { 
    while(1)
    {
      sock->read_some(boost::asio::buffer( command ), error); // wait for the client to query
      if(error == boost::asio::error::eof)
      {
        LOG4CXX_INFO(logger, "Connection closed");
        break;
      }
      else if(error)
        throw boost::system::system_error(error);
        

      LOG4CXX_INFO(logger, (char*)&command.front());

      std::string str_command = boost::lexical_cast<std::string>( &command.front() );
      std::string resp;

      std::vector<std::string> tokens;
      boost::split(tokens, str_command, boost::is_any_of(", "), boost::token_compress_on);

      if (tokens.at(0) == "hello") {
	resp = "Received: ";
	resp += boost::lexical_cast<std::string>( &command.front() );
      }
      else if (tokens.at(0) == "ping") {
	resp.resize(256);
	unsigned char *ucptr = (unsigned char*)resp.data();
	for(unsigned int i = 0; i < resp.size(); i++) {
	  unsigned char d = (unsigned char)(( rand()/(double)RAND_MAX ) * 80.0) + 20.0;
	  *ucptr++ = (unsigned char)d;
	}
      }
      else if (tokens.at(0) == "GetConfig") {
	unsigned int nChannels = rx_mdst.nChannels();;

	            // 8 byte for header +  sizeof(nchannels)     + sizeof(nsamples)
	resp.resize( (sizeof(float) * 2) + (sizeof(unsigned int)) + (sizeof(unsigned int)));
	unsigned int *ui32ptr = (unsigned int*)resp.data();
	unsigned int *ui32ptr_numbytes = ui32ptr + 1;

	*ui32ptr++ = 0x000000ff;            //Type
	*ui32ptr++ = 0x0; //sizeof( nChannels) ;   //num bytes following this field

	*ui32ptr++ = nChannels; 
	*ui32ptr++ = 32;

	*ui32ptr_numbytes = sizeof(unsigned int) * ((unsigned int)(ui32ptr - ui32ptr_numbytes - 1) );
	//DBG_OUT(*ui32ptr_numbytes );
      }
      else if (tokens.at(0) == "GetIQ") {

	unsigned int nChannels = rx_mdst.nChannels();;

	            // 8 byte for header +  sizeof( total samples  in all channels)
	resp.resize( (sizeof(float) * 2) + (sizeof(std::complex<float>) * (N_SAMPS_TO_SEND * nChannels)) );
	//resp.resize( (sizeof(std::complex<float>) * (32 + 32 + 32))  + (sizeof(float) * 2) ); // + 8 byte for header
	unsigned int *ui32ptr = (unsigned int*)resp.data();
	unsigned int *ui32ptr_numbytes = ui32ptr + 1;
;
	*ui32ptr++ = 0x000000fe; //Type
	*ui32ptr++ = 0x0; // (sizeof(std::complex<float>) * (32 + 32 + 32));   //num bytes following this field

	// send 32 IQ points per channel
	for (unsigned int ch = 0; ch < nChannels; ch++) {
	  memcpy((void*)ui32ptr, snapshot256.at( ch ).data(), (sizeof(std::complex<float>) * N_SAMPS_TO_SEND)) ;
	  ui32ptr += N_SAMPS_TO_SEND * (sizeof(std::complex<float>) / sizeof(unsigned int)) ;
	}

	*ui32ptr_numbytes = sizeof(unsigned int) * ((unsigned int)(ui32ptr - ui32ptr_numbytes - 1) );

      }
      else {
	LOG4CXX_INFO(logger, str_command );
      }
      boost::asio::write(*sock, boost::asio::buffer(resp.data(),resp.size() ));
	
    }
    LOG4CXX_INFO(logger, "command thread count: " << --nCmdThreadCnt << " [host: " << sock->remote_endpoint().address().to_string() << "]" );
  }  
  catch (std::exception& e)
  {
    std::cerr << "Exception in command thread: " << e.what() << "\n";
    LOG4CXX_ERROR(logger, "command thread count: " << --nCmdThreadCnt << " [host: " << sock->remote_endpoint().address().to_string() << "]" );
    return;
  }
}






void command_server(short port)
{
  boost::asio::io_service io_service;

  LOG4CXX_INFO(logger, "Starting command server at " << port);
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
  for (;;)
  //while( !local_kill_switch  )
  {
    socket_ptr sock(new tcp::socket(io_service));
    if (nCmdThreadCnt >= 1) continue;
    a.accept(*sock);
    boost::thread t(boost::bind(command_session, sock));
    LOG4CXX_INFO(logger, "command thread count: " << ++nCmdThreadCnt << " [host: " << sock->remote_endpoint().address().to_string() << "]" );
  }
}

