#ifndef _CDEVICESTORAGE_H_
#define _CDEVICESTORAGE_H_

#include "CSharedMemSimple.hpp"

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

struct _s_DeviceBufferFormatType {
  unsigned int nbuffptrs;
  unsigned int nsamps_per_ch;
  unsigned int spb;
  unsigned int nChannels;

  // _head and _tail used for indexing _MultiDeviceBufferPtrs - range is [0, _nbuffptrs)
  unsigned int head;       // index into next _MultiDeviceBufferPtrs for writing
  unsigned int tail;       // index into oldest _MultiDeviceBufferPtrs for reading
};

struct _s_DeviceBufferTimeType {
  double full_sec;
  double frac_sec;
};




class CDeviceStorage {

private:

  bool _use_shared_memory;
  std::string _shm_uid;
  
  struct _s_DeviceBufferFormatType _BufferMetaData, *_mp;

  CSharedMemSimple _ShmBufferMetaData;
  CSharedMemSimple _ShmBuffer;

  typedef std::vector<std::vector<std::complex<float> > > MultiDeviceBufferType;
  MultiDeviceBufferType _MultiDeviceBuffer;                          // used for non-shm operation

  typedef std::vector<std::complex<float> *> BufferPointerType;
  std::vector<BufferPointerType> _MultiDeviceBufferPtrs;

public:



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


  // ??? inline head / tail routines
  unsigned int head() { return _mp->head; }
  unsigned int tail() { return _mp->tail; }
  
  unsigned int head(unsigned int i) { return ( (_mp->head + i) % _mp->nbuffptrs) ; }
  unsigned int tail(unsigned int i) { return ( (_mp->tail + i) % _mp->nbuffptrs) ; }

  unsigned int head_next() { return (_mp->head+1) % _mp->nbuffptrs; }
  unsigned int tail_next() { return (_mp->tail+1) % _mp->nbuffptrs; }

  void head_inc() { _mp->head = (_mp->head+1) % _mp->nbuffptrs; }
  void tail_inc() { _mp->tail = (_mp->tail+1) % _mp->nbuffptrs; }

  void head_set(unsigned int i) { _mp->head = i; }
  void tail_set(unsigned int i) { _mp->tail = i; }

  void shared_memory(std::string shm_uid)
  {
    _shm_uid = shm_uid;
    _use_shared_memory = true;
    
  }

  void init(unsigned int nChannels, unsigned int spb, unsigned int num_buff_ptrs)
  {

    _mp = &_BufferMetaData;
    if (_use_shared_memory) {
      _ShmBufferMetaData.init(_shm_uid+"_meta", sizeof( struct _s_DeviceBufferFormatType ) );
      _mp = (struct _s_DeviceBufferFormatType *)_ShmBufferMetaData.ptr();

    }

    _mp->head = _mp->tail = 0;
    _mp->nbuffptrs = num_buff_ptrs;
    _mp->nChannels = nChannels;
    _mp->spb = spb;

    _BufferTime.clear();
    _BufferTime.resize(num_buff_ptrs);
    assert( _BufferTime.size() == _BufferTime.capacity() );

    _MultiDeviceBufferPtrs.clear();

    _mp->nsamps_per_ch = _mp->spb * _mp->nbuffptrs;

    if (_use_shared_memory) {
      _ShmBuffer.init(_shm_uid, _mp->nChannels * _mp->nsamps_per_ch * sizeof( std::complex<float> ) );
      for (size_t j = 0; j < _mp->nbuffptrs; j++)  {
	BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
	for (size_t ch = 0; ch < _mp->nChannels; ch++)
	  bptr.push_back( (std::complex<float>*)_ShmBuffer.ptr() + (_mp->nsamps_per_ch * ch) + (_mp->spb * j) );
	_MultiDeviceBufferPtrs.push_back(bptr);
      }

    }
    else {
      _MultiDeviceBuffer.clear();
      for (unsigned int i = 0; i < _mp->nChannels; i++)
	_MultiDeviceBuffer.push_back( std::vector<std::complex<float> >(_mp->nsamps_per_ch) );

      // verify capacity and size
      assert(_MultiDeviceBuffer.size() == _mp->nChannels );
      for (unsigned int i = 0; i < _mp->nChannels; ++i)
	assert(_MultiDeviceBuffer.at(i).size() == _MultiDeviceBuffer.at(i).capacity());

      for (size_t j = 0; j < _mp->nbuffptrs; j++)  {
	BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
	for (size_t ch = 0; ch < _mp->nChannels; ch++)
	  bptr.push_back( _MultiDeviceBuffer.at(ch).data()+( _mp->spb *j) );
	_MultiDeviceBufferPtrs.push_back(bptr);
      }
    }

    assert(_MultiDeviceBufferPtrs.size() == _mp->nbuffptrs);
    
    for (unsigned int i = 0; i < _mp->nChannels; i++)
      _MultiDeviceWasteBuffer.push_back( std::vector<std::complex<float> >(_mp->spb) );

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

  unsigned int nbuffptrs() { return _mp->nbuffptrs; }
  unsigned int nsamps_per_ch() { return _mp->nsamps_per_ch; }
  void print_info()
  {
    std::cerr << "Device storage:"    << std::endl;
    std::cerr << " channels:          " << _mp->nChannels << std::endl; 
    std::cerr << " buffers  / channel " << _mp->nbuffptrs   << std::endl; 
    std::cerr << " samples / buffer   " << _mp->spb         << std::endl; 
    std::cerr << " bytes / sample     " << sizeof(std::complex<float>) << std::endl;
    std::cerr << " shared memory      " << _use_shared_memory << std::endl;
    if (_use_shared_memory) {
      _ShmBuffer.info();
      _ShmBufferMetaData.info();
    }
    
  }

};

#endif
