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



int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string xml_config_str, xml_file, xml_path;
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
      //("time", po::value<unsigned int>(&run_time)->default_value(10), "run time in seconds")
        ("rx-only", "enable receive direction only")
        ("tx-only", "enable transmit direction only")
        ("cmd-port", po::value<short>(&command_server_port)->default_value(5180),"command server port")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("Multi-channel Uhd STreamer v000.001.000\n %s") % desc << std::endl;
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

      
      usrp1.init_rx(256,       // this is 256 samples per BUFF
		    64*4);     // 64*4 BUFFs

      // attach to streaming buffer memory structure
      //CDeviceStorage &rx_mdst = usrp1.rx_mdst();
      CDeviceStorage rx_mdst;
      rx_mdst.attach_shm("/ShmMultiDeviceBufferRx");

      //unsigned int N_SPB_BUFFS = rx_mdst.nbuffptrs();

      usrp1.run_rx_async();  // non-blocking call spawns a receive thread

#if 0
      //rx_handler_find_idx(usrp1, rx_mdst, run_time);
      rx_handler_find_avg_bw_pwr(rx_mdst, run_time)

#else  // use split system

      // Back ground loop
      boost::system_time next_console_refresh = boost::get_system_time();
      while(!local_kill_switch) {

	if (boost::get_system_time() > next_console_refresh) {
	  //count_processed_buffers = 0;
	  next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1000e3));
	  std::cerr << ".";
	}

      } //  while(!local_kill_switch)
#endif
      usrp1.kill_rx();
      boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    }

    else if ( vm.count("tx-only") ) {
      DBG_OUT( usrp1.nTxChannels() );
      DBG_OUT( usrp1.spb() );

      usrp1.init_tx(256,       // this is 256 samples per BUFF
		    16*4);     // 16*4 BUFFs
     
      // attach to streaming buffer memory structure
      //CDeviceStorage &tx_mdst = usrp1.tx_mdst();
      CDeviceStorage tx_mdst;
      tx_mdst.attach_shm("/ShmMultiDeviceBufferTx");

      //unsigned int N_SPB_BUFFS = tx_mdst.nbuffptrs();

      usrp1.run_tx_async();  // non-blocking call spawns a transmit thread thread


      // Back ground loop
      boost::system_time next_console_refresh = boost::get_system_time();
      while(!local_kill_switch) {

	if (boost::get_system_time() > next_console_refresh) {
	  //count_processed_buffers = 0;
	  next_console_refresh = boost::get_system_time() + boost::posix_time::microseconds(long(1000e3));
	  std::cerr << ".";
	}

      } //  while(!local_kill_switch)


      usrp1.stop_tx();
      tx_mdst.head_set ( 0 );
      tx_mdst.tail_set ( 0 );

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
      std::string s = "rxfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 
  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxfreq") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_freq(i);
      std::string s = "rxfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 
  }
  else if ( (sToken.size() == 3) && (sToken.at(0) == "rxgain") && (sToken.at(1) == "all")) {
    double set_val = boost::lexical_cast<double>( sToken.at(2) );
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_gain(i, set_val);
      std::string s = "rxgain " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 
  }
  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxgain") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_gain(i);
      std::string s = "rxgain " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 

  }
  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxrate") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double act_val = usrp1.rx_rate(i);
      std::string s = "rxrate " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 

  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "rxchan") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nRxChannels(); i++)
    {
      double freq_val = usrp1.rx_freq(i);
      double rate_val = usrp1.rx_rate(i);
      double gain_val = usrp1.rx_gain(i);
      std::string s = "rxchan " + boost::lexical_cast<std::string>(i) + " " \
	                        + boost::lexical_cast<std::string>(freq_val) + " " \
	                        + boost::lexical_cast<std::string>(rate_val) + " " \
	                        + boost::lexical_cast<std::string>(gain_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 

  }


  else if ( (sToken.size() == 3) && (sToken.at(0) == "txfreq") && (sToken.at(1) == "all")) {
    double set_val = boost::lexical_cast<double>( sToken.at(2) );
    for (unsigned int i = 0; i < usrp1.nTxChannels(); i++)
    {
      double act_val = usrp1.tx_freq(i, set_val);
      std::string s = "txfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 
  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "txfreq") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nTxChannels(); i++)
    {
      double act_val = usrp1.tx_freq(i);
      std::string s = "txfreq " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 
  }
  else if ( (sToken.size() == 3) && (sToken.at(0) == "txgain") && (sToken.at(1) == "all")) {
    double set_val = boost::lexical_cast<double>( sToken.at(2) );
    for (unsigned int i = 0; i < usrp1.nTxChannels(); i++)
    {
      double act_val = usrp1.tx_gain(i, set_val);
      std::string s = "txgain " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
      LOG4CXX_INFO(logger, s);
    } 
  }

  else if ( (sToken.size() == 2) && (sToken.at(0) == "txgain") && (sToken.at(1) == "all")) {
    for (unsigned int i = 0; i < usrp1.nTxChannels(); i++)
    {
      double act_val = usrp1.tx_gain(i);
      std::string s = "txgain " + boost::lexical_cast<std::string>(i) + " " + boost::lexical_cast<std::string>(act_val) + "\n";
      temp += s;
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
      std::vector<char> command(256*100);

      sock->read_some(boost::asio::buffer( command ), error); // wait for the client to query
      if(error == boost::asio::error::eof)
      {
        LOG4CXX_INFO(logger, "Connection closed");
        break;
      }
      else if(error)
	throw boost::system::system_error(error);
      //LOG4CXX_INFO(logger, (char*)&command.front());

      std::string resp;
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
