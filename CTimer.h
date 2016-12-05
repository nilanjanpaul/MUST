#include <iostream>
#include <string.h>

class CTimer
{
  private:
    struct timespec  m_ts1, m_ts2;
    struct timespec  m_tsdiff;
    std::string m_desc;
    struct timespec diff(struct timespec start, struct timespec end)
    {
      struct timespec temp;

      if ((end.tv_nsec - start.tv_nsec) < 0)
	{
	  temp.tv_sec = end.tv_sec - start.tv_sec - 1;
	  temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	}
      else
	{
	  temp.tv_sec = end.tv_sec - start.tv_sec;
	  temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
      return temp;
    }


  public:
    CTimer(std::string ds = "");
    ~CTimer();
    double foo(double x);
};

