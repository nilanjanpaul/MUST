#ifndef _CDEVICESTORAGE_H_
#define _CDEVICESTORAGE_H_

#include "CSharedMemSimple.hpp"

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl


struct _s_DeviceBufferTimeType {
  double full_sec;
  double frac_sec;
};

struct _s_DeviceBufferFormatType {
  unsigned int nbuffptrs;
  unsigned int nsamps_per_ch;
  unsigned int spb;
  unsigned int nChannels;

  // _head and _tail used for indexing _MultiDeviceBufferPtrs - range is [0, _nbuffptrs)
  unsigned int head;       // index into next _MultiDeviceBufferPtrs for writing
  unsigned int tail;       // index into oldest _MultiDeviceBufferPtrs for reading

  struct _s_DeviceBufferTimeType time;

};


class CDeviceStorage {

private:

  bool _use_shared_memory;
  bool _pause_flag;
  std::string _shm_uid;
  
  struct _s_DeviceBufferFormatType _BufferMetaData, *_mp;

  CSharedMemSimple _ShmBufferMetaData;
  CSharedMemSimple _ShmBuffer;

  typedef std::vector<std::vector<std::complex<float> > > MultiDeviceBufferType;
  MultiDeviceBufferType _MultiDeviceBuffer;                          // used for non-shm operation

  typedef std::vector<std::complex<float> *> BufferPointerType;
  std::vector<BufferPointerType> _MultiDeviceBufferPtrs;


public:

  
  // constructor
  CDeviceStorage()
  {
    _use_shared_memory = false;
    _pause_flag = false;
    _MultiDeviceBufferPtrs.clear();
    _MultiDeviceBuffer.clear();

  }

  // Destructor
  ~CDeviceStorage() { }

  unsigned int spb()           { return _mp->spb; }
  unsigned int nChannels()     { return _mp->nChannels; }
  unsigned int nbuffptrs()     { return _mp->nbuffptrs; }
  unsigned int nsamps_per_ch() { return _mp->nsamps_per_ch; }


  // ??? inline head / tail routines
  unsigned int head() { return _mp->head; }    // get current head index
  unsigned int tail() { return _mp->tail; }    // get current tail index
  
  unsigned int head(unsigned int i) { return ( (_mp->head + i) % _mp->nbuffptrs) ; }    // get head index with offset
  unsigned int tail(unsigned int i) { return ( (_mp->tail + i) % _mp->nbuffptrs) ; }    // get tail index with offset

  void head_inc() { _mp->head = (_mp->head+1) % _mp->nbuffptrs; }           // set head increment by 1
  void tail_inc() { _mp->tail = (_mp->tail+1) % _mp->nbuffptrs; }           // set tail increment by 1

  void head_set(unsigned int i) { _mp->head = i; }                          // set head to new offset
  void tail_set(unsigned int i) { _mp->tail = i; }                          // set tail to new offset

  unsigned int depth() {
    int d = (int)_mp->head - (int)_mp->tail;
    if (d < (int)0) d += _mp->nbuffptrs;
    return (unsigned int)d;
  }

  void pause(bool f) { _pause_flag = f; }
  bool is_pause() { return _pause_flag ; }

  void shared_memory(std::string shm_uid)
  {
    _shm_uid = shm_uid;
    _use_shared_memory = true;
    
  }

  bool check_shm(std::string shm_uid)
  {
    return _ShmBufferMetaData.exist( shm_uid );
  }

  // load existing shared memory structure
  void attach_shm(std::string shm_uid)
  {
    _shm_uid = shm_uid;
    _use_shared_memory = true;
    _ShmBufferMetaData.attach(_shm_uid+"_meta", sizeof( struct _s_DeviceBufferFormatType ) );
    _mp = (struct _s_DeviceBufferFormatType *)_ShmBufferMetaData.ptr();

    _ShmBuffer.attach(_shm_uid, _mp->nChannels * _mp->nsamps_per_ch * sizeof( std::complex<float> ) );
    for (size_t j = 0; j < _mp->nbuffptrs; j++)  {
      BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
      for (size_t ch = 0; ch < _mp->nChannels; ch++)
	bptr.push_back( (std::complex<float>*)_ShmBuffer.ptr() + (_mp->nsamps_per_ch * ch) + (_mp->spb * j) );
      _MultiDeviceBufferPtrs.push_back(bptr);
    }
    assert(_MultiDeviceBufferPtrs.size() == _mp->nbuffptrs);
  }

  void init(unsigned int nChannels, unsigned int spb, unsigned int num_buff_ptrs)
  {
    if (_use_shared_memory) {
      _ShmBufferMetaData.init(_shm_uid+"_meta", sizeof( struct _s_DeviceBufferFormatType ) );
      _mp = (struct _s_DeviceBufferFormatType *)_ShmBufferMetaData.ptr();

      _mp->head = _mp->tail = 0;
      _mp->nbuffptrs = num_buff_ptrs;
      _mp->nChannels = nChannels;
      _mp->spb = spb;
      _mp->nsamps_per_ch = _mp->spb * _mp->nbuffptrs;

      _ShmBuffer.init(_shm_uid, _mp->nChannels * _mp->nsamps_per_ch * sizeof( std::complex<float> ) );
      for (size_t j = 0; j < _mp->nbuffptrs; j++)  {
	BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
	for (size_t ch = 0; ch < _mp->nChannels; ch++)
	  bptr.push_back( (std::complex<float>*)_ShmBuffer.ptr() + (_mp->nsamps_per_ch * ch) + (_mp->spb * j) );
	_MultiDeviceBufferPtrs.push_back(bptr);
      }
      assert(_MultiDeviceBufferPtrs.size() == _mp->nbuffptrs);

    }
    else {
      _mp = &_BufferMetaData;
      _mp->head = _mp->tail = 0;
      _mp->nbuffptrs = num_buff_ptrs;
      _mp->nChannels = nChannels;
      _mp->spb = spb;
      _mp->nsamps_per_ch = _mp->spb * _mp->nbuffptrs;

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

      assert(_MultiDeviceBufferPtrs.size() == _mp->nbuffptrs);
    }
    
    //_MultiDeviceWasteBuffer.clear();
    //for (unsigned int i = 0; i < _mp->nChannels; i++)
    //  _MultiDeviceWasteBuffer.push_back( std::vector<std::complex<float> >(_mp->spb) );

  } // end of init

  void time(double full_sec, double frac_sec)
  {
    _mp->time.full_sec = full_sec;
    _mp->time.frac_sec = frac_sec;
  }

  void print_time()
  {
    std::cerr << "[" << _mp->time.full_sec + _mp->time.frac_sec << "] " << std::endl;
  }

  BufferPointerType buffer_ptr_idx(unsigned int idx)  { return _MultiDeviceBufferPtrs.at(idx); }
  void* buffer_ptr_ch_idx_(unsigned int ch, unsigned int idx)
  {
    return((void*)_MultiDeviceBufferPtrs.at(idx).at(ch));
  }

  //void* buffer_ptr_ch_(unsigned int ch) { return((void*)_MultiDeviceBuffer.at( ch ).data()); }
  //void* buffer_ptr_(unsigned int ch, unsigned int idx) { return((void*)(_MultiDeviceBuffer.at(ch).data()+( _spb *idx)));  }

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
