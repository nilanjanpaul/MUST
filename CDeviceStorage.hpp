#ifndef _CDEVICESTORAGE_H_
#define _CDEVICESTORAGE_H_

#include "CSharedMemSimple.hpp"

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

struct _s_DeviceBufferTimeType {
  double full_sec;
  double frac_sec;
};


class CDeviceStorage {

private:
  unsigned int _nbuffptrs;
  unsigned int _nsamps_per_ch;
  unsigned int _spb;
  unsigned int _nChannels;
  bool _use_shared_memory;
  std::string _shm_uid;
  
  CSharedMemSimple _ShmBuffer;

  typedef std::vector<std::vector<std::complex<float> > > MultiDeviceBufferType;
  MultiDeviceBufferType _MultiDeviceBuffer;

  typedef std::vector<std::complex<float> *> BufferPointerType;
  std::vector<BufferPointerType> _MultiDeviceBufferPtrs;

public:
  // _head and _tail used for indexing _MultiDeviceBufferPtrs - range is [0, _nbuffptrs)
  unsigned int _head;       // index into next _MultiDeviceBufferPtrs for writing
  unsigned int _tail;       // index into oldest _MultiDeviceBufferPtrs for reading



  MultiDeviceBufferType _MultiDeviceWasteBuffer;


  std::vector<struct _s_DeviceBufferTimeType> _BufferTime;
  
  // constructor
  CDeviceStorage() { _use_shared_memory = false; }

  // constructor
  CDeviceStorage(unsigned int nChannels, unsigned int spb, unsigned int num_buff_ptrs)
  {
    init(nChannels, spb, num_buff_ptrs);
  }

  // Destructor
  ~CDeviceStorage() { }

#if 0
  unsigned int head() { return _head; }
  unsigned int tail() { return _tail; }
  void head(unsigned int i) { _head = i; }
  void tail(unsigned int i) { _tail = i; }
#endif

  void shared_memory(std::string shm_uid)
  {
    _shm_uid = shm_uid;
    _use_shared_memory = true;
    
  }

  void init(unsigned int nChannels, unsigned int spb, unsigned int num_buff_ptrs)
  {

    _head = _tail = 0;
    _nbuffptrs = num_buff_ptrs;
    _nChannels = nChannels;
    _spb = spb;

    _BufferTime.clear();
    _BufferTime.resize(num_buff_ptrs);
    assert( _BufferTime.size() == _BufferTime.capacity() );

    _MultiDeviceBufferPtrs.clear();

    _nsamps_per_ch = _spb * _nbuffptrs;

    if (_use_shared_memory) {
      _ShmBuffer.init(_shm_uid, _nChannels * _nsamps_per_ch * sizeof( std::complex<float> ));
      for (size_t j = 0; j < _nbuffptrs; j++)  {
	BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
	for (size_t ch = 0; ch < _nChannels; ch++)
	  bptr.push_back( (std::complex<float>*)_ShmBuffer.ptr() + (_nsamps_per_ch * ch) + (_spb * j) );
	_MultiDeviceBufferPtrs.push_back(bptr);
      }

    }
    else {
      _MultiDeviceBuffer.clear();
      for (unsigned int i = 0; i < _nChannels; i++)
	_MultiDeviceBuffer.push_back( std::vector<std::complex<float> >(_nsamps_per_ch) );

      // verify capacity and size
      assert(_MultiDeviceBuffer.size() == _nChannels );
      for (unsigned int i = 0; i < _nChannels; ++i)
	assert(_MultiDeviceBuffer.at(i).size() == _MultiDeviceBuffer.at(i).capacity());

      for (size_t j = 0; j < _nbuffptrs; j++)  {
	BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
	for (size_t ch = 0; ch < _nChannels; ch++)
	  bptr.push_back( _MultiDeviceBuffer.at(ch).data()+( _spb *j) );
	_MultiDeviceBufferPtrs.push_back(bptr);
      }
    }

    assert(_MultiDeviceBufferPtrs.size() == _nbuffptrs);
    
    for (unsigned int i = 0; i < _nChannels; i++)
      _MultiDeviceWasteBuffer.push_back( std::vector<std::complex<float> >(_spb) );

  } // end of init

  void print_time()
  {
    unsigned int idx = 0;//_head; // head may be updated
    std::cerr << "[" << _BufferTime.at(idx).full_sec + _BufferTime.at(idx).frac_sec << "] " << std::endl;
  }

  BufferPointerType buffer_ptr_idx(unsigned int idx)  { return _MultiDeviceBufferPtrs.at(idx); }
  void* buffer_ptr_ch_idx_(unsigned int ch, unsigned int idx)
  {
    return((void*)_MultiDeviceBufferPtrs.at(idx).at(ch));
  }

  //void* buffer_ptr_ch_(unsigned int ch) { return((void*)_MultiDeviceBuffer.at( ch ).data()); }
  //void* buffer_ptr_(unsigned int ch, unsigned int idx) { return((void*)(_MultiDeviceBuffer.at(ch).data()+( _spb *idx)));  }

  unsigned int nbuffptrs() { return _nbuffptrs; }
  unsigned int nsamps_per_ch() { return _nsamps_per_ch; }
  void print_info()
  {
    std::cerr << "Device storage:"    << std::endl;
    std::cerr << " channels:          " << _nChannels << std::endl; 
    std::cerr << " buffers  / channel " << _nbuffptrs   << std::endl; 
    std::cerr << " samples / buffer   " << _spb         << std::endl; 
    std::cerr << " bytes / sample     " << sizeof(std::complex<float>) << std::endl;
    std::cerr << " shared memory      " << _use_shared_memory << std::endl;
    if (_use_shared_memory) _ShmBuffer.info();
    
  }

};

#endif
