#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define RECV_BUFF_BYTES (256*8)

using boost::asio::ip::tcp;


class TCPSimple
{
public:
  // Constructor - sender
  TCPSimple(
	    const std::string& host,
	    const std::string& port)
    : socket_(io_service_)
  {
    tcp::resolver resolver(io_service_);
    tcp::resolver::query query(tcp::v4(), host , port);
    tcp::resolver::iterator iterator = resolver.resolve(query);

    socket_.connect( *iterator );

    //boost::asio::connect(socket_, iterator);

    initialize();

    std::cout << "TCP connection to " << host << ":" << port << " ready!" << std::endl;
  }

#if 0
  // Constructor - receiver
  TCPSimple(
	    const std::string& port
	    ) : socket_(io_service_, udp::endpoint(udp::v4(), boost::lexical_cast<short>(port))) {

    port_ = port;

    // initialize constructor variables
    initialize();

    start_recv_proc();
    std::cout << "Receiver ready on port: " << port << std::endl;
  }
#endif

  // initialize constructor variables
  void initialize()
  {
    recv_buff_.resize( RECV_BUFF_BYTES );

  }

  // Destructor
  ~TCPSimple()
  {
    // stop threads
    //stop_recv_proc();
    boost::system::error_code ec;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket_.close();
  }


  void send_n_recv(const std::string& msg, std::string& msg_reply) {

    const unsigned int MAX_REPLY_LENGTH = (unsigned int)(1024);
    boost::system::error_code ec;

    // send to server
    size_t request_length = msg.size();
    boost::asio::write(socket_, boost::asio::buffer(msg, request_length));

    // reply from server
    char reply[MAX_REPLY_LENGTH];
    memset(reply, '\0', MAX_REPLY_LENGTH);
    size_t reply_length = socket_.read_some(boost::asio::buffer( reply,MAX_REPLY_LENGTH ), ec); // wait for the client to query
    
    if (reply_length == 0) return;
    //std::cout << "Reply is: " << reply << std::endl; 
    msg_reply = reply;
  }

  void send(void *msg, unsigned int msg_bytes) {
  }

  void recv()
  {
  }



private:
  boost::asio::io_service io_service_;
  tcp::socket socket_;
  std::string port_;
  //tcp::endpoint endpoint_;

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

