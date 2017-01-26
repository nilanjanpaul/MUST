#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define RECV_BUFF_BYTES (256*8)

using boost::asio::ip::udp;


class UDPSimple // in the not so simple BOOST
{
public:
  // Constructor - sender
  UDPSimple(
	    const std::string& host, 
	    const std::string& port
	    ) : socket_(io_service_, udp::endpoint(udp::v4(), 0)) {
    udp::resolver resolver(io_service_);
    udp::resolver::query query(udp::v4(), host, port);
    udp::resolver::iterator iter = resolver.resolve(query);
    endpoint_ = *iter;

    port_ = port;

    // initialize constructor variables
    initialize();

    std::cout << "Sender ready: " << host << ":" << port << std::endl;
  }

  // Constructor - receiver
  UDPSimple(
	    const std::string& port
	    ) : socket_(io_service_, udp::endpoint(udp::v4(), boost::lexical_cast<short>(port))) {

    port_ = port;

    // initialize constructor variables
    initialize();

    start_recv_proc();
    std::cout << "Receiver ready on port: " << port << std::endl;
  }


  // initialize constructor variables
  void initialize()
  {
    total_send_pkt_ = current_send_bytes_ = total_send_bytes_ = 0;
    total_recv_pkt_ = current_recv_bytes_ = total_recv_bytes_ = 0;

    is_recv_proc_running_ = false;

    recv_buff_.resize( RECV_BUFF_BYTES );

  }

  void start_recv_proc()
  {
    std::cout << "UDP receive proc started" << std::endl;

    // send the server 1 packet so it knows endpoint
    //socket_.send_to(boost::asio::buffer(recv_buff_.data(), recv_buff_.size()), endpoint_);

    is_recv_proc_running_ = true;
    ptr_recv_proc_ = new boost::thread( boost::bind(&UDPSimple::recv_proc, this) );

  }

  void stop_recv_proc()
  {
    is_recv_proc_running_ = false;
  }

  // Destructor
  ~UDPSimple()
  {
    // stop threads
    stop_recv_proc();
    socket_.close();
  }


  void send(const std::string& msg) {
    current_send_bytes_ = socket_.send_to(boost::asio::buffer(msg, msg.size()), endpoint_);
    total_send_pkt_++;
    total_send_bytes_ += current_send_bytes_;
  }

  void send(void *msg, unsigned int msg_bytes) {
    current_send_bytes_ = socket_.send_to(boost::asio::buffer(msg, msg_bytes), endpoint_);
    total_send_pkt_++;
    total_send_bytes_ += current_send_bytes_;

  }

  void recv()
  {
    udp::endpoint sender_endpoint;
    current_recv_bytes_ = socket_.receive_from(boost::asio::buffer((void*)recv_buff_.data(), recv_buff_.size()), sender_endpoint);
    total_recv_pkt_++;
    total_recv_bytes_ += current_recv_bytes_;
   }

  std::complex<float>* ptr_recv_buff_as_cf32()
  {
    return ( (std::complex<float>*)recv_buff_.data() );
  }

  void set_recv_buff_size(unsigned int size_bytes)
  {
    recv_buff_.resize( size_bytes );
  }
  
  void* ptr_rx()
  {
    return ( (void*)recv_buff_.data() );
  }

  unsigned int recv_bytes_count()
  {
    return (current_recv_bytes_);
  }


  void recv_proc()
  {

    while ( is_recv_proc_running_ == true )
    {
      udp::endpoint sender_endpoint;
      current_recv_bytes_ = socket_.receive_from(boost::asio::buffer(recv_buff_.data(), recv_buff_.size() ), sender_endpoint);
      total_recv_pkt_++;
      total_recv_bytes_ += current_recv_bytes_;
    }
   }


private:
  boost::asio::io_service io_service_;
  udp::socket socket_;
  std::string port_;
  udp::endpoint endpoint_;

  bool is_recv_proc_running_;
  bool is_loopback_proc_running;

  boost::thread *ptr_recv_proc_;

  unsigned int total_send_pkt_;
  unsigned int current_send_bytes_;
  unsigned int total_send_bytes_;

  unsigned int total_recv_pkt_;
  unsigned int current_recv_bytes_;
  unsigned int total_recv_bytes_;
  std::vector<unsigned char> recv_buff_;
};

