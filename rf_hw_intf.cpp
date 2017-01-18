#include <iostream>
#include <complex>
#include <fstream>
#include <sstream>

#include <fftw3.h>

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>

#include "log4cxx/logger.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/helpers/exception.h"


#include "CRadio.hpp"
#include "CTimer.h"
#include "CDeviceStorage.hpp"

#include <signal.h>

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

#define DEFAULT_COMMAND_SERVER_PORT 5180


#define TEST_FREQ_OFFSET_W_PPS     0
#define TEST_BAND_AMPLITUDE_W_PPS  1
#define TEST_COMMAND_LINE_SAMPS  2


// SELECT ME
#define TEST_CASE TEST_COMMAND_LINE_SAMPS

using boost::asio::ip::tcp;

using namespace log4cxx;
using namespace log4cxx::helpers;

namespace po = boost::program_options;
typedef boost::shared_ptr<tcp::socket> socket_ptr;


LoggerPtr logger(Logger::getLogger("rf_hw_intf")); // replace with name of this file
bool local_kill_switch = false;

std::string command_parse (std::vector<char> command);
void command_session(socket_ptr sock);
void command_server(short port);

CRadio usrp1;
int nCmdThreadCnt = 0;


void signal_interrupt_control_c(int s){
  printf("Caught signal %d\n",s);
  local_kill_switch = true;
}

static bool mag_compare(std::complex<float> a, std::complex<float> b)
{
  return (std::abs(a) < std::abs(b));
}

static bool real_compare(std::complex<float> a, std::complex<float> b)
{
  return (a.real() < b.real() );
}

void generate_freq_domain_signal(unsigned int spb, std::vector<unsigned int> bin, std::vector<float> mag, std::vector<std::complex<float> > &time_buff)
{
  std::vector<std::complex<float> > freq_buff(spb);

  // size of bin and mag vector must match
  if ( bin.size() != mag.size() )
    {
      std::cerr << "Error: bin.size() != mag.size()" << std::endl;
      return;
    }

  // Transform declarations
  fftwf_plan ifft_p = fftwf_plan_dft_1d(spb,
					(fftwf_complex*)&freq_buff.front(),
					(fftwf_complex*)&time_buff.front(),
					FFTW_BACKWARD,
					FFTW_ESTIMATE);

  memset((void*)freq_buff.data(), 0, spb*sizeof(std::complex<float>) );

  // populate bins with magnitude
  for (unsigned int i = 0; i < bin.size(); i++)
    {
      if (bin.at(i) >= spb)
	{
	  std::cerr << "Error: bin.at(i) >= bin.size()" << std::endl;
	  continue;
	}
      freq_buff.at( bin.at(i) ) = std::complex<float>(mag.at(i), 0.0);
    }

  fftwf_execute(ifft_p); // Octave equivalent: ifft(Y)*spb

}

// Example:
//   int my_i[]   = {10, 20, 30, 40, 50, 60, 70, 80}; 
//   float my_m[] = {.1, .1, .1, .1, .1, .1, .1, .1}; 
//   std::vector<unsigned int> bin_vec(my_i, my_i + sizeof(my_i) / sizeof(int) );
//   std::vector<float>        mag_vec(my_m, my_m + sizeof(my_m) / sizeof(float) );
//   multi_tone_1_256(bin_vec, mag_vec, signal);
void multi_tone_1_256(std::vector<unsigned int> bin_vec, std::vector<float> mag_vec, std::vector<std::complex<float> > &wave_buff)
{
  unsigned int spb = wave_buff.size();
  generate_freq_domain_signal(spb, bin_vec, mag_vec, wave_buff);
}


void multi_tone_1_256_b1_b2(unsigned int b1, unsigned int b2, std::vector<std::complex<float> > &wave_buff)
{
  unsigned int spb = wave_buff.size();

  float mag = .0125;
  std::vector<float> mag_vec(b2-b1, mag);

  std::vector<unsigned int> bin_vec;
  for (unsigned int b = b1; b < b2; b++)
    bin_vec.push_back(b % spb);

  assert(bin_vec.size() == mag_vec.size());

  generate_freq_domain_signal(spb, bin_vec, mag_vec, wave_buff);
}



// Example:
//  std::vector<std::complex<float> > signal( usrp1.spb() );
//  int bin = 4;
//  single_tone_1_256( bin, 0.5, signal );
void single_tone_1_256(int bin, float magnitude, std::vector<std::complex<float> > &wave_buff)
{
  unsigned int spb = wave_buff.size();

  int my_i[]   = {bin};          std::vector<unsigned int> bin_vec(my_i, my_i + sizeof(my_i) / sizeof(int) );
  float my_m[] = {magnitude};  std::vector<float>        mag_vec(my_m, my_m + sizeof(my_m) / sizeof(float) );
  generate_freq_domain_signal(spb, bin_vec, mag_vec, wave_buff);

}


std::vector<std::complex<float> > t_sync_ref;
std::vector<std::complex<float> > f_sync_ref;

void rx_handler_find_idx(CRadio& usrp1, CDeviceStorage& rx_mdst, unsigned int run_time)
{

  boost::this_thread::sleep(boost::posix_time::milliseconds(100));

  unsigned int spb = usrp1.spb();
  unsigned int nRxChannels = usrp1.nRxChannels();

  std::vector<float> mag_buff( spb );
  std::vector<std::complex<float> > fft_buff( spb );
  assert( fft_buff.size() == spb);

  unsigned int total_num_samps = rx_mdst.nsamps_per_ch();

  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );

  // calc noise floor ?

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

    if (rx_mdst._tail == rx_mdst._head) continue;

    unsigned int ch = 0; // this is the channel to trigger on

    fftwf_plan fft_p = fftwf_plan_dft_1d( spb,
					  (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst._tail),
					  (fftwf_complex*)&fft_buff.front(),
					  FFTW_FORWARD,
					  FFTW_ESTIMATE);
    fftwf_execute(fft_p);

    fftwf_destroy_plan(fft_p);

    // magnitude
    for(int i = 0; i < spb; i++)
      mag_buff.at(i) = abs(fft_buff.at(i));

    // find max value index                               - asm inline this
    unsigned int pki = std::distance(mag_buff.begin(),
				     std::max_element(mag_buff.begin(), mag_buff.end()) );

    if ((state == 0) && 
	(pki == 4) &&
	(rx_mdst._tail < (rx_mdst.nbuffptrs() - 4) ))  // check roll over cond at tail - simple method
      {
	mag_4m1   = mag_buff.at(4);
	mag_14m1  = mag_buff.at(14);
	LOG4CXX_INFO(logger, "ch | tail | head | pki | mag " << ch << " " << rx_mdst._tail << " " << rx_mdst._head << " " << pki << " " << mag_buff.at(pki));

	state = 1;
      }
    else if ((state == 1)  ) {
      // check tail buffer idx @ -1 0
      std::cerr << mag_4m1   << " " << mag_14m1  << std::endl;
      std::cerr << mag_buff.at(4)  << " " << mag_buff.at(14) << std::endl;


      fftwf_plan fft_1024 = fftwf_plan_dft_1d( spb*4,
					       (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( ch, rx_mdst._tail-2),
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
   
      usrp1.plot_buffer("10.10.0.10", "1337", rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 0, rx_mdst._tail-2), spb * 4);
      usrp1.plot_buffer("10.10.0.10", "1337", rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 1, rx_mdst._tail-2), spb * 4);
      usrp1.plot_buffer("10.10.0.10", "1337", rx_mdst.buffer_ptr_ch_idx_( /*ch*/ 2, rx_mdst._tail-2), spb * 4);
      LOG4CXX_INFO(logger, "tail | head : " << rx_mdst._tail << " " << rx_mdst._head );;


      //usrp1.plot_rx("10.10.0.10", "1337", rx_mdst._tail-2, 4);
      local_kill_switch = true;
      state = 0;
    }
   
    // increment tail for next round
    rx_mdst._tail++;
    rx_mdst._tail = rx_mdst._tail % rx_mdst.nbuffptrs();

    //boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1000e3));

    //DBG_OUT(rx_mdst._head);
    //DBG_OUT(rx_mdst._tail);

  } // while

} // rx_handler_find_idx

void rx_handler_find_peaks(CRadio& usrp1, CDeviceStorage& rx_mdst, unsigned int run_time)
{

  std::vector<float> mag_buff( usrp1.spb() );
  std::vector<std::complex<float> > fft_buff( usrp1.spb());
  assert( fft_buff.size() == usrp1.spb());

  unsigned int total_num_samps = rx_mdst.nsamps_per_ch();

  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );

  while ((!local_kill_switch) && (boost::get_system_time() < done_time)) {
 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(500e3));

    while (rx_mdst._head != 0);
    
    rx_mdst.print_time();
    //DBG_OUT(rx_mdst.head);
    //DBG_OUT(rx_mdst.tail);

    // find peaks for different channels
    for (unsigned int ch = 0; ch < usrp1.nRxChannels(); ch++) {
      std::vector<float> mag_buff( total_num_samps );
      std::vector<std::complex<float> > fft_buff( total_num_samps );

      fftwf_plan fft_p = fftwf_plan_dft_1d( total_num_samps,
					    (fftwf_complex*)rx_mdst.buffer_ptr_ch_idx_( ch, 0),
					    /*(fftwf_complex*)rx_mdst.buffer_ptr_( ch, 0), */
					    (fftwf_complex*)&fft_buff.front(),
					    FFTW_FORWARD,
					    FFTW_ESTIMATE);
      fftwf_execute(fft_p);

      // magnitude - ignore frequencies around DC component
      for(int i = 10; i < total_num_samps-10; i++)
	mag_buff.at(i) = abs(fft_buff.at(i));

      // find first 15 peaks
      double avg_pw = 0.0;
      for (unsigned int i = 0; i < 15 ; ++i)
      {
	// find max value index                               - asm inline this
	unsigned int pki = std::distance(mag_buff.begin(),
					 std::max_element(mag_buff.begin(), mag_buff.end()) );
	//std::cerr << pki << ": " << mag_buff.at(pki) / total_num_samps << ": " << pki* usrp1.rx_rate(ch) / (total_num_samps) <<  std::endl;

	avg_pw += mag_buff.at(pki) / total_num_samps;
	// now zero out this peak
	mag_buff.at(pki) = 0.0;
      }
      //std::cerr << "ch: " << ch << " --- avg pwr = " << 10*log10( avg_pw) << std::endl;

      LOG4CXX_INFO(logger, "ch: " << ch << " --- avg pwr = " << 10*log10( avg_pw) );



    } // for loop channels

  } // while

} // rx_handler_find_peaks


int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string xml_config_str, xml_file, xml_path, sig_samples_str, sync_samps_str;
    double seconds_in_future;
    size_t total_num_samps;
    double rate, freq, gain;
    unsigned int run_time;
    short command_server_port;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("conf", po::value<std::string>(&xml_config_str), "specify xml device configuration file and path \"devices.xml,/devices/active\"")
        ("print-conf", "print device conf") 
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("time", po::value<unsigned int>(&run_time)->default_value(10), "run time in seconds")
        ("sync", po::value<std::string>(&sync_samps_str), "cf32 sync signal")
        ("sig", po::value<std::string>(&sig_samples_str), "cf32 samples")
        ("rx-only", "enable receive side only")
        ("tx-only", "enable transmit side only")
        ("cmd-port", po::value<short>(&command_server_port)->default_value(5180),"command server port")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Multi Receive %s") % desc << std::endl;
	return 0;
    }

    // Configure the logger
    PropertyConfigurator::configure("./logconf.prop");

    // Ctrl-C for breaking
    signal (SIGINT, signal_interrupt_control_c);

    // launch thread for remote control
    boost::thread cmds(command_server, command_server_port);

    // parse xml args
    std::vector<std::string> tokens;
    boost::split(tokens, xml_config_str, boost::is_any_of(","), boost::token_compress_on);
    assert(tokens.size() == 2 );

    xml_file = tokens.at(0);
    xml_path = tokens.at(1);

    if (usrp1.make_device_handle(xml_file, xml_path) == RETURN_ERROR) {
      std::cerr << "unable to make device" << std::endl;
      return 0;
    }

    if (vm.count("print-conf")){
      usrp1.print_radio_configurations();
      return 0;
    }

    if ( !vm.count("rx-only") && !vm.count("tx-only"))
    {
      std::cerr << "must specify flags --rx-only or --tx-only" << std::endl;
      return 0;
    }

    if ( vm.count("rx-only") ) {

      DBG_OUT( usrp1.nRxChannels() );
      DBG_OUT( usrp1.spb() );

      if (vm.count("sync")) {
	std::vector<std::string> tokens;
	boost::split(tokens, sync_samps_str, boost::is_any_of("| "), boost::token_compress_on);
	for (unsigned int i = 0; i < tokens.size() ; i++) {
	  std::complex<float> cf = boost::lexical_cast<std::complex<float> >( tokens.at(i) );
	  t_sync_ref.push_back(cf);
	  //std::cout << tokens.at(i) << " " << cf << std::endl;
	}
	assert(tokens.size() == t_sync_ref.size() );
	assert(t_sync_ref.size() == usrp1.spb() );
      }


      usrp1.init_rx();

      // get reference to device streaming buffer
      CDeviceStorage &rx_mdst = usrp1.rx_mdst();
      //unsigned int N_SPB_BUFFS = rx_mdst.nbuffptrs();

      usrp1.run_rx_async();  // non-blocking call spawns a receive thread

      // loca this from link
      //rx_handler_find_peaks(usrp1, rx_mdst, run_time); // move this into callback / timer object
      rx_handler_find_idx(usrp1, rx_mdst, run_time);

      usrp1.kill_rx();
      boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    }

    else if ( vm.count("tx-only") ) {
      DBG_OUT( usrp1.nTxChannels() );
      DBG_OUT( usrp1.spb() );

      // parse samples from command line
      std::vector<std::complex<float> > tx_sync;
      if (vm.count("sync")) {
	std::vector<std::string> tokens;
	boost::split(tokens, sync_samps_str, boost::is_any_of("| "), boost::token_compress_on);
	for (unsigned int i = 0; i < tokens.size() ; i++) {
	  std::complex<float> cf = boost::lexical_cast<std::complex<float> >( tokens.at(i) );
	  tx_sync.push_back(cf);
	  std::cout << tokens.at(i) << " " << cf << std::endl;
	}
	assert(tokens.size() == tx_sync.size() );
	assert(tx_sync.size() == usrp1.spb() );
      }

      std::vector<std::complex<float> > tx_signal;
      if (vm.count("sig")) {
	std::vector<std::string> tokens;
	boost::split(tokens, sig_samples_str, boost::is_any_of("| "), boost::token_compress_on);
	for (unsigned int i = 0; i < tokens.size() ; i++) {
	  std::complex<float> cf = boost::lexical_cast<std::complex<float> >( tokens.at(i) );
	  tx_signal.push_back(cf);
	  //std::cout << tokens.at(i) << " " << cf << std::endl;
	}
	assert(tokens.size() == tx_signal.size() );
	assert(tx_signal.size() == usrp1.spb() );
      }

      usrp1.init_tx();
      
      CDeviceStorage &tx_mdst = usrp1.tx_mdst();
      unsigned int N_SPB_BUFFS = tx_mdst.nbuffptrs();

      usrp1.run_tx_async();  // non-blocking call spawns a transmit thread thread


      //boost::system_time next_update = boost::get_system_time();

      if (vm.count("sync") && vm.count("sig")) {

	// copy sync to tx_mdst buffers at idx = 0
	for (unsigned int ch = 0; ch < usrp1.nTxChannels(); ch++) {
	  // replicate signal to all (N_SPB_BUFFS) buffers per chan
	  for (unsigned int idx = 0; idx < 1; idx++) {
	    memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
		   (void*)tx_sync.data(), usrp1.spb() * sizeof(std::complex<float>) );
	  }
	}

	// copy signal to tx_mdst buffers at idx = 1.. N_SPB_BUFFS
	for (unsigned int ch = 0; ch < usrp1.nTxChannels(); ch++) {
	  // replicate signal to all (N_SPB_BUFFS) buffers per chan
	  for (unsigned int idx = 1; idx < N_SPB_BUFFS; idx++) {
	    memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
		   (void*)tx_signal.data(), usrp1.spb() * sizeof(std::complex<float>) );
	  }
	}

      }
      else if (vm.count("sig")) {
	// copy signal to tx_mdst buffers for sending 
	for (unsigned int ch = 0; ch < usrp1.nTxChannels(); ch++) {
	  // replicate signal to all (N_SPB_BUFFS) buffers per chan
	  for (unsigned int idx = 0; idx < N_SPB_BUFFS; idx++) {
	    memcpy((void*)tx_mdst.buffer_ptr_ch_idx_(ch, idx),
		   (void*)tx_signal.data(), usrp1.spb() * sizeof(std::complex<float>) );
	  }
	}
      }

      while( !local_kill_switch ) { 
	if (tx_mdst._head != 0) continue;


	// if signal is constant then generate once above the while conditional.
	// if sign is NOT constant then update signal here and copy to tx_mdst buffers.

	// send signal in intervals
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	tx_mdst._head = N_SPB_BUFFS; // kick off sending
	std::cerr << ".";
      }
      usrp1.stop_tx();
      tx_mdst._head = 0;
      tx_mdst._tail = 0;

    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    
    //finished
    std::cerr << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}





std::string command_parse (std::vector<char> command)
{
  std::string temp( &command.front() );
  LOG4CXX_INFO(logger, temp);
  std::vector<std::string> sToken;
  boost::split(sToken, temp, boost::is_any_of(" ") );
  
#if 0
  for (unsigned int i = 0; i < sToken.size(); i++)
    std::cerr << sToken.at(i) << " ";
  std::cerr << std::endl;
#endif

  temp = "";
  if ( (sToken.size() == 3) && (sToken.at(0) == "rxfreq") && (sToken.at(1) == "all")) {
    double set_val = boost::lexical_cast<double>( sToken.at(2) );
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_freq(i, set_val);
      std::string s = "rxfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + " ";
      temp += s + " ";
      LOG4CXX_INFO(logger, s);
    } 
  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxfreq") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_freq(i);
      std::string s = "rxrate " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + " ";
      temp += s + " ";
      LOG4CXX_INFO(logger, s);
    } 
  }
  else if ( (sToken.size() == 3) && (sToken.at(0) == "rxgain") && (sToken.at(1) == "all")) {
    double set_val = boost::lexical_cast<double>( sToken.at(2) );
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_gain(i, set_val);
      std::string s = "rxfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + " ";
      temp += s + " ";
      LOG4CXX_INFO(logger, s);
    } 
  }
  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxgain") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_gain(i);
      std::string s = "rxgain " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + " ";
      temp += s + " ";
      LOG4CXX_INFO(logger, s);
    } 

  }

  else if ( (sToken.size() == 3) && (sToken.at(0) == "txfreq") && (sToken.at(1) == "all")) {
    double set_val = boost::lexical_cast<double>( sToken.at(2) );
    for (unsigned int i = 0; i < usrp1.nTxChannels(); i++)
    {
      double act_val = usrp1.tx_freq(i, set_val);
      std::string s = "txfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + " ";
      temp += s + " ";
      LOG4CXX_INFO(logger, s);
    } 
  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "txfreq") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nTxChannels(); i++)
    {
      double act_val = usrp1.tx_freq(i);
      std::string s = "txrate " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + " ";
      temp += s + " ";
      LOG4CXX_INFO(logger, s);
    } 
  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxhold") && (sToken.at(1) == "on")) {
    usrp1.pause_rx();
    temp += "rx hold: " + sToken.at(1);
    LOG4CXX_INFO(logger, temp);
  }
  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxhold") && (sToken.at(1) == "off")) {
    usrp1.cont_rx();
    temp += "rx hold: " + sToken.at(1);
    LOG4CXX_INFO(logger, temp);
  }


  else {
    temp = "---Unknown command---";
  }

  return (temp);
}

void command_session(socket_ptr sock)
{
  boost::system::error_code error;
 
  try
  { 
    while(1)
    {
      std::vector<char> command(256);

      sock->read_some(boost::asio::buffer( command ), error); // wait for the client to query
      if(error == boost::asio::error::eof)
      {
        //LOG4CXX_INFO(logger, "Connection closed");
        break;
      }
      else if(error)
        throw boost::system::system_error(error);

      //LOG4CXX_INFO(logger, (char*)&command.front());

      std::string resp;
      resp = "Rsp: " ;
      resp += command_parse(command);
      resp += "\r\n";

      boost::asio::write(*sock, boost::asio::buffer(resp.data(),resp.size() ));
	
    }
    --nCmdThreadCnt;
    //LOG4CXX_INFO(logger, "command thread count: " << nCmdThreadCnt << " [host: " << sock->remote_endpoint().address().to_string() << "]" );
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
  {
    socket_ptr sock(new tcp::socket(io_service));
    if (nCmdThreadCnt >= 1) continue;
    a.accept(*sock);
    boost::thread t(boost::bind(command_session, sock));
    ++nCmdThreadCnt;
    //LOG4CXX_INFO(logger, "command thread count: " << nCmdThreadCnt << " [host: " << sock->remote_endpoint().address().to_string() << "]" );
  }
}













#if 0
// forward fft is unnormalized. so divide by N. In this case N = 256.

    //---
    {
    std::vector<std::complex<float> > wb(256);
    int my_i[]   = {16, 32 };
    float my_m[] = {.1, .2 }; 
    std::vector<unsigned int> bin_vec(my_i, my_i + sizeof(my_i) / sizeof(int) );
    std::vector<float>        mag_vec(my_m, my_m + sizeof(my_m) / sizeof(float) );
    multi_tone_1_256(bin_vec, mag_vec, wb );
    // find max value index                               - asm inline this
    unsigned int pki = std::distance(wb.begin(),
				     std::max_element(wb.begin(), wb.end(), real_compare ) );
    std::cerr << pki << ": " << wb.at(pki) <<  std::endl;
    
    std::vector<std::complex<float> > fft_buff( 256 );
    fftwf_plan fft_p = fftwf_plan_dft_1d( 256,
					  (fftwf_complex*)&wb.front(),
					  (fftwf_complex*)&fft_buff.front(),
					  FFTW_FORWARD,
					  FFTW_ESTIMATE);

    fftwf_execute(fft_p);
    std::vector<float> mag_buff( 256 );
    // magnitude - ignore frequencies around DC component
    for(int i = 0; i < 256; i++)
      mag_buff.at(i) = abs(fft_buff.at(i));

    // find max value index                               - asm inline this
                 pki = std::distance(mag_buff.begin(),
				     std::max_element(mag_buff.begin(), mag_buff.end()) );
    std::cerr << pki << ": " << mag_buff.at(pki)/256 << std::endl;

    exit(0);
    }	
    //---
#endif
