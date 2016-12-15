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

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>

#include "CRadio.hpp"
#include "CTimer.h"
#include "DeviceStorage.h"

#include <signal.h>
#include "CWriteOml.h"


#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

#define TEST_FREQ_OFFSET_W_PPS     0
#define TEST_BAND_AMPLITUDE_W_PPS  1


// SELECT ME
#define TEST_CASE TEST_BAND_AMPLITUDE_W_PPS


namespace po = boost::program_options;

bool local_kill_switch = false;
void signal_interrupt_control_c(int s){
  printf("Caught signal %d\n",s);
  local_kill_switch = true;
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

void rx_handler_find_peaks(CRadio& usrp1, DeviceStorageType& rx_mdst, unsigned int run_time)
{
  std::vector<float> mag_buff( usrp1.spb() );
  std::vector<std::complex<float> > fft_buff( usrp1.spb());
  assert( fft_buff.size() == usrp1.spb());

  unsigned int total_num_samps = rx_mdst.nsamps_per_ch;


  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );
  unsigned int ch = 0;


  while ((!local_kill_switch) && (boost::get_system_time() < done_time)) {
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(0.5e6));
    while (rx_mdst.head != 0);
    
    rx_mdst.print_time();
    //DBG_OUT(rx_mdst.head);
    //DBG_OUT(rx_mdst.tail);

    //std::cerr << rx_mdst.head << " " << rx_mdst.tail << std::endl;
    //usrp1.plot1(rx_mdst, "10.13.0.10", "1337"); continue;

    // find peaks for different channels
    for (unsigned int ch = 0; ch < usrp1.nRxChannels(); ch++) {
      std::vector<float> mag_buff( total_num_samps );
      std::vector<std::complex<float> > fft_buff( total_num_samps );
      fftwf_plan fft_p = fftwf_plan_dft_1d( total_num_samps,
					    (fftwf_complex*)&rx_mdst._MultiDeviceBuffer.at( ch ).front(),
					    (fftwf_complex*)&fft_buff.front(),
					    FFTW_FORWARD,
					    FFTW_ESTIMATE);

      fftwf_execute(fft_p);

      // magnitude - ignore frequencies around DC component
      for(int i = 10; i < total_num_samps-10; i++)
	mag_buff.at(i) = abs(fft_buff.at(i));


      // find first 15 peaks
      std::cerr << "channel: " << ch << ": " << std::endl;
      for (unsigned int i = 0; i < 15 ; ++i)
      {
	// find max value index                               - asm inline this
	unsigned int pki = std::distance(mag_buff.begin(),
					 std::max_element(mag_buff.begin(), mag_buff.end()) );
	std::cerr << pki << ": " << mag_buff.at(pki) << ": " << pki* usrp1.rx_rate(ch) / (total_num_samps) <<  std::endl;

	// now zero out this peak
	mag_buff.at(pki) = 0.0;
      }

    } // for loop channels

  } // while

}

// =========================
void rx_handler_find_peak_write_to_oml(CRadio& usrp1, DeviceStorageType& rx_mdst, unsigned int run_time)
{
  std::vector<float> mag_buff( usrp1.spb() );
  std::vector<std::complex<float> > fft_buff( usrp1.spb());
  assert( fft_buff.size() == usrp1.spb());

  unsigned int total_num_samps = rx_mdst.nsamps_per_ch;


  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );
  unsigned int ch = 0;

  //  OML writer set up
  CWriteOml OML;
  std::string omlDbFilename("measured_carrier_offsets");
  std::string omlServerName("idb2:3003");
  //std::string omlServerName("file");
  OML.init(omlDbFilename, omlServerName );
  OML.start();


  while ((!local_kill_switch) && (boost::get_system_time() < done_time)) {
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(0.5e6));
    while (rx_mdst.head != 0);
    
    rx_mdst.print_time();
    //DBG_OUT(rx_mdst.head);
    //DBG_OUT(rx_mdst.tail);

    //std::cerr << rx_mdst.head << " " << rx_mdst.tail << std::endl;
    //usrp1.plot1(rx_mdst, "10.13.0.10", "1337"); continue;

    // find peaks for different channels
    for (unsigned int ch = 0; ch < usrp1.nRxChannels(); ch++) {
      std::vector<float> mag_buff( total_num_samps );
      std::vector<std::complex<float> > fft_buff( total_num_samps );
      fftwf_plan fft_p = fftwf_plan_dft_1d( total_num_samps,
					    (fftwf_complex*)&rx_mdst._MultiDeviceBuffer.at( ch ).front(),
					    (fftwf_complex*)&fft_buff.front(),
					    FFTW_FORWARD,
					    FFTW_ESTIMATE);

      fftwf_execute(fft_p);

      // magnitude - ignore frequencies around DC component
      for(int i = 10; i < total_num_samps-10; i++)
	mag_buff.at(i) = abs(fft_buff.at(i));


      // find max value index                               - asm inline this
      unsigned int pki = std::distance(mag_buff.begin(),
				       std::max_element(mag_buff.begin(), mag_buff.end()) );
      std::cerr << "channel: " << ch << ": " << pki << ": " << mag_buff.at(pki) << ": " << pki* usrp1.rx_rate(ch) / (total_num_samps) <<  std::endl;

      OML.insert((uint32_t)usrp1.rx_rate(ch),
		 (float)usrp1.rx_freq(ch),  
		 (uint32_t)usrp1.rx_gain(ch),
		 (uint32_t) total_num_samps,
		 (uint32_t) ch,
		 (uint32_t) pki,
		 (float) mag_buff.at(pki),
		 (float) pki* usrp1.rx_rate(ch) / (total_num_samps)
		 );

    } // for loop
  } // while


}




int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string xml_config_str, xml_file, xml_path;
    double seconds_in_future;
    size_t total_num_samps;
    double rate, freq, gain;
    unsigned int run_time;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("conf", po::value<std::string>(&xml_config_str), "specify xml device configuration file and path \"devices.xml,/devices/active\"")
        ("print-conf", "print device conf") 
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
      //("nsamps", po::value<size_t>(&total_num_samps)->default_value(16384), "total number of samples to receive")
        ("time", po::value<unsigned int>(&run_time)->default_value(10), "run time in seconds")
        ("rx-only", "enable receive side only")
        ("tx-only", "enable transmit side only")

    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Multi Receive %s") % desc << std::endl;
	return 0;
    }

    // Ctrl-C for breaking
    signal (SIGINT, signal_interrupt_control_c);

    std::vector<std::string> tokens;
    boost::split(tokens, xml_config_str, boost::is_any_of(","), boost::token_compress_on);
    assert(tokens.size() == 2 );

    xml_file = tokens.at(0);
    xml_path = tokens.at(1);

    CRadio usrp1;
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

      usrp1.init_rx();
      unsigned int N_SPB_BUFFS = 64*4;

      DeviceStorageType rx_mdst(usrp1.nRxChannels(), usrp1.spb(), N_SPB_BUFFS); // this should go on the heap
      //rx_mdst.print_info();
      assert(rx_mdst._MultiDeviceBuffer.size() == usrp1.nRxChannels() );

#if 1 // threaded examples
      usrp1.run_rx( rx_mdst );  // non-blocking call spawns a receive thread
#if (TEST_CASE == TEST_FREQ_OFFSET_W_PPS)
      rx_handler_find_peak_write_to_oml(usrp1, rx_mdst, run_time); 
#endif
#if (TEST_CASE == TEST_BAND_AMPLITUDE_W_PPS)
      rx_handler_find_peaks(usrp1, rx_mdst, run_time);
#endif
      usrp1.stop_rx();
      boost::this_thread::sleep(boost::posix_time::milliseconds(250));


#else // non threaded example
      total_num_samps = usrp1.spb() * N_SPB_BUFFS;
      usrp1.run_rx_once( rx_mdst, seconds_in_future, total_num_samps);

      // example to find peak in all channels
      usrp1.plot1(rx_mdst, "10.13.0.10", "1337");
      for (unsigned int ch = 0; ch < usrp1.nRxChannels(); ch++) {

	std::vector<float> mag_buff( total_num_samps );
	std::vector<std::complex<float> > fft_buff( total_num_samps );
	fftwf_plan fft_p = fftwf_plan_dft_1d( total_num_samps,
					      (fftwf_complex*)&rx_mdst._MultiDeviceBuffer.at( ch ).front(),
					      (fftwf_complex*)&fft_buff.front(),
					      FFTW_FORWARD,
					      FFTW_ESTIMATE);

	fftwf_execute(fft_p);

	// magnitude - ignore frequencies around DC component
	for(int i = 10; i < total_num_samps-10; i++)
	  mag_buff.at(i) = abs(fft_buff.at(i));


	// find max value index                               - asm inline this
	unsigned int pki = std::distance(mag_buff.begin(),
					 std::max_element(mag_buff.begin(), mag_buff.end()) );
	std::cerr << "channel: " << ch << ": " << pki << ": " << mag_buff.at(pki) << ": " << pki* usrp1.get_rx_rate(ch) / (total_num_samps) <<  std::endl;
      }
#endif

    }

    else if ( vm.count("tx-only") ) {
      DBG_OUT( usrp1.nTxChannels() );
      DBG_OUT( usrp1.spb() );

      std::vector<std::vector<std::complex<float> > > signal( usrp1.nTxChannels(), std::vector<std::complex<float> >(usrp1.spb() ));
      unsigned int N_SPB_BUFFS = 32;

      usrp1.init_tx();
      DeviceStorageType tx_mdst(usrp1.nTxChannels(), usrp1.spb(), N_SPB_BUFFS); // this should go on the heap


      usrp1.run_tx( tx_mdst );  // non-blocking call spawns a transmit thread thread
      boost::system_time next_update = boost::get_system_time();

#if (TEST_CASE == TEST_FREQ_OFFSET_W_PPS)
	// do stuff here...                            time this part!!!
	single_tone_1_256( 32, 0.5, signal.at(0) );
#endif

#if (TEST_CASE == TEST_BAND_AMPLITUDE_W_PPS)
	int my_i[]   = {16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240};
	float my_m[] = {.1, .1, .1, .1, .1, .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1,  .1}; 
	std::vector<unsigned int> bin_vec(my_i, my_i + sizeof(my_i) / sizeof(int) );
	std::vector<float>        mag_vec(my_m, my_m + sizeof(my_m) / sizeof(float) );
	multi_tone_1_256(bin_vec, mag_vec, signal.at(0));
#endif
	// copy signal to tx_mdst buffers for sending 
	for (unsigned int ch = 0; ch < usrp1.nTxChannels(); ch++) {

	  // replicate signal to all (N_SPB_BUFFS) buffers per chan
	  for (unsigned int idx = 0; idx < N_SPB_BUFFS; idx++) {
	    memcpy((void*)tx_mdst._MultiDeviceBufferPtrs.at(idx).at(ch),
		   (void*)signal.at(ch).data(), usrp1.spb() * sizeof(std::complex<float>) );
	  }
	}



      while( !local_kill_switch ) { 
	if (tx_mdst.head != 0) continue;


	// if signal is constant then generate once above the while conditional.
	// if sign is NOT constant then update signal here and copy to tx_mdst buffers.

	tx_mdst.head = N_SPB_BUFFS; // kick off sending

      }
      usrp1.stop_tx();
      tx_mdst.head = tx_mdst.tail = 0;
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));

    
    //finished
    std::cerr << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
