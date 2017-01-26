#ifndef _CSHAREDMEMSIMPLE_HPP_
#define _CSHAREDMEMSIMPLE_HPP_

#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string>


#define SHM_MASTER (true)
#define SHM_SLAVE (false)

#define DBG_OUT(x) std::cerr << #x << " = " << x << std::endl

class CSharedMemSimple
{
private:
  void *rptr_;
  int fd_;
  bool isMaster_;
  std::string shm_uid_;
  unsigned int bytes_allocated_;
  void handle_error(const char *msg)
  {
    do { 
      perror(msg);
      exit(EXIT_FAILURE);
    } while (0);
  }

public:
  // Constructor
  CSharedMemSimple() { };

  // Constructor
  CSharedMemSimple(std::string shm_uid, unsigned int sizeInBytes)
  {
    init(shm_uid, sizeInBytes);
  }

  ~CSharedMemSimple()
  {
    if (isMaster_ == false)
      return;

    munmap(rptr_, bytes_allocated_);
    shm_unlink(shm_uid_.c_str());
  }


  bool exist(std::string shm_uid)
  {
    bool rv;
    struct stat sb;
    std::string test_shm_str = "/dev/shm"+shm_uid;

    rv = (stat(test_shm_str.c_str(), &sb) == -1) ? false : true;
    return rv;
  }

  void attach(std::string shm_uid, unsigned int sizeInBytes)
  {
    isMaster_ = false;
    shm_uid_ = shm_uid;
    bytes_allocated_ = sizeInBytes;

    /* Attach to shared memory object */
    if ((fd_ = shm_open(shm_uid_.c_str(), /* O_CREAT | */O_RDWR, S_IRUSR | S_IWUSR)) == -1) {
      std::cerr << "SHM - non-existent " << shm_uid << std::endl;
      handle_error("open");
    }
    if (ftruncate(fd_, bytes_allocated_) == -1)
      handle_error("ftruncate");

    /* Map shared memory object */
    rptr_ = (void *) mmap(NULL, bytes_allocated_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (rptr_ == MAP_FAILED)
      exit(-1); /* Handle error */
  }

  void init(std::string shm_uid, unsigned int sizeInBytes)
  {
    isMaster_ = true;
    shm_uid_ = shm_uid;
    bytes_allocated_ = sizeInBytes;

    /* Create shared memory object and set its size */
    if ((fd_ = shm_open(shm_uid_.c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR)) == -1) {
      std::cerr << "SHM - non-existent " << shm_uid << std::endl;
      handle_error("open");
    }

    if (ftruncate(fd_, bytes_allocated_) == -1)
      handle_error("ftruncate");

    /* Map shared memory object */
    rptr_ = (void *) mmap(NULL, bytes_allocated_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (rptr_ == MAP_FAILED)
      exit(-1); /* Handle error */
  }

  unsigned int nBytes()
  {
    return (bytes_allocated_);
  }

  void* ptr()
  {
    return (void *)(rptr_);
  }

  void info()
  {
    printf("SHM info: %s, %s\n", shm_uid_.c_str(), (isMaster_) ? "Master" : "Slave"  );
    printf("SHM bytes allocated: %i\n", nBytes()  );
  }
};


#endif
