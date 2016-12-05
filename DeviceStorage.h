#ifndef _MDST_H_
#define _MDST_H_


struct _s_DeviceBufferTimeType {
  double full_sec;
  double frac_sec;
};

struct _s_MultiDeviceStorageType {
  unsigned int head;  // index into next _MultiDeviceBufferPtrs for writing;
  unsigned int tail;  // index into oldest _MultiDeviceBufferPtrs for reading
  unsigned int nbuffptrs;
  unsigned int nsamps_per_ch;

  typedef std::vector<std::vector<std::complex<float> > > MultiDeviceBufferType;
  MultiDeviceBufferType _MultiDeviceBuffer;

  typedef std::vector<std::complex<float> *> BufferPointerType;
  std::vector<BufferPointerType> _MultiDeviceBufferPtrs;

  std::vector<struct _s_DeviceBufferTimeType> _BufferTime;
  
  _s_MultiDeviceStorageType() { }

  _s_MultiDeviceStorageType(unsigned int _nRxChannels, unsigned int _spb, unsigned int num_buff_ptrs)
  {
    init(_nRxChannels, _spb, num_buff_ptrs);
  }

  void init(unsigned int _nRxChannels, unsigned int _spb, unsigned int num_buff_ptrs)
  {
    _BufferTime.clear();
    _BufferTime.resize(num_buff_ptrs);
    assert( _BufferTime.size() == _BufferTime.capacity() );

    _MultiDeviceBuffer.clear();
    _MultiDeviceBufferPtrs.clear();

    nsamps_per_ch = _spb * num_buff_ptrs;

    for (unsigned int i = 0; i < _nRxChannels; i++)
      _MultiDeviceBuffer.push_back( std::vector<std::complex<float> >(nsamps_per_ch) );
    
    // verify capacity and size
    for (unsigned int i = 0; i < _nRxChannels; ++i)
      assert(_MultiDeviceBuffer.at(i).size() == _MultiDeviceBuffer.at(i).capacity());


    for (size_t j = 0; j < num_buff_ptrs; j++)  {
      BufferPointerType bptr; // each bptr points to samps_per_buff amount of memory in _MultiDeviceBuffer
      for (size_t ch = 0; ch < _nRxChannels; ch++)
	bptr.push_back( _MultiDeviceBuffer.at(ch).data()+( _spb *j) );
      _MultiDeviceBufferPtrs.push_back(bptr);
    }

    assert(_MultiDeviceBufferPtrs.size() == num_buff_ptrs);

    head = tail = 0;
    nbuffptrs = num_buff_ptrs;

  } // end of init

  void print_time()
  {
    unsigned int idx = 0;//head; // head may be updated
    std::cerr << "[" << _BufferTime.at(idx).full_sec + _BufferTime.at(idx).frac_sec << "] " << std::endl;
  }
};


typedef struct _s_MultiDeviceStorageType DeviceStorageType;
#endif
