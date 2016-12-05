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

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

namespace po = boost::program_options;


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

void single_tone_1_256(int bin, float magnitude, std::vector<std::complex<float> > &wave_buff)
{
  unsigned int spb = wave_buff.size();

  int my_i[]   = {bin};          std::vector<unsigned int> bin_vec(my_i, my_i + sizeof(my_i) / sizeof(int) );
  float my_m[] = {magnitude};  std::vector<float>        mag_vec(my_m, my_m + sizeof(my_m) / sizeof(float) );
  generate_freq_domain_signal(spb, bin_vec, mag_vec, wave_buff);

}


// =========================
void rx_handler_soemthing_generic(CRadio& usrp1, DeviceStorageType& rx_mdst, unsigned int run_time)
{
  std::vector<float> mag_buff( usrp1.spb() );
  std::vector<std::complex<float> > fft_buff( usrp1.spb());
  assert( fft_buff.size() == usrp1.spb());

  boost::system_time next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
  boost::system_time done_time = boost::get_system_time() + boost::posix_time::microseconds(long( run_time*1.0e6) );
  unsigned int ch = 0;
  while(boost::get_system_time() < done_time) {
    //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    if (boost::get_system_time() < next_console_refresh) continue;
    next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1.0e6));
    rx_mdst.print_time();
    std::cerr << rx_mdst.head << " " << rx_mdst.tail << std::endl;
    //usrp1.plot1(rx_mdst, "10.13.0.10", "1337");


    // find peaks for different channels

    fftwf_plan fft_p = fftwf_plan_dft_1d(usrp1.spb(),
					 (fftwf_complex*)&rx_mdst._MultiDeviceBuffer.at( ch ).front(),
					 (fftwf_complex*)&fft_buff.front(),
					 FFTW_FORWARD,
					 FFTW_ESTIMATE);

    fftwf_execute(fft_p);

    // magnitude
    for(int i = 0; i < usrp1.spb(); i++)
      mag_buff.at(i) = abs(fft_buff.at(i));

    // find max value index                               - asm inline this
    unsigned int pki = std::distance(mag_buff.begin(),
				     std::max_element(mag_buff.begin(), mag_buff.end()) );
    std::cerr << "channel: " << ch << ": " << pki << std::endl;
    ch++;
    ch = ch % usrp1.nRxChannels();

  }
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
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(16384), "total number of samples to receive")
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

    std::vector<std::string> tokens;
    boost::split(tokens, xml_config_str, boost::is_any_of(","), boost::token_compress_on);
    assert(tokens.size() == 2 );

    xml_file = tokens.at(0);
    xml_path = tokens.at(1);


    CRadio usrp1;
    usrp1.make_device_handle(xml_file, xml_path);

    DBG_OUT( usrp1.nRxChannels() );
    DBG_OUT( usrp1.spb() );

    if (vm.count("print-conf")){
      usrp1.print_radio_configurations();
      return 0;
    }

    if ( vm.count("rx-only") ) {

      usrp1.init_rx();

      DeviceStorageType rx_mdst(usrp1.nRxChannels(), usrp1.spb(), 8); // this should go on the heap

      usrp1.run_rx( rx_mdst );  // non-blocking call spawns a receive thread
      //usrp1.run_rx( rx_mdst, seconds_in_future, total_num_samps);

      rx_handler_soemthing_generic(usrp1, rx_mdst, run_time); 

      usrp1.stop_rx();
      boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    }

    else if ( vm.count("tx-only") ) {



      usrp1.init_tx();
      DeviceStorageType tx_mdst(usrp1.nTxChannels(), usrp1.spb(), 8); // this should go on the heap

      std::vector<std::complex<float> > signal( usrp1.spb() );
      int bin = 4;
      single_tone_1_256( bin, 0.5, signal );

      // populate tx_mdst buffer with signal
      unsigned int ch = 0;
      memcpy((void*)tx_mdst._MultiDeviceBuffer.at(ch).data(), (void*)signal.data(), usrp1.spb() * sizeof(std::complex<float>) );
      tx_mdst.head++;


      usrp1.run_tx( tx_mdst );  // non-blocking call spawns a transmit thread thread
      while(1) { 
      boost::this_thread::sleep(boost::posix_time::milliseconds(250));
      }      
    }
    else
    {
      std::cerr << "must specify --rx-only or --tx-only" << std::endl;
    }

    
    //finished
    std::cerr << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
