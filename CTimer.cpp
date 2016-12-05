#include "CTimer.h"

CTimer::CTimer(std::string ds)
{
  m_desc = ds;
  clock_gettime(CLOCK_MONOTONIC_RAW, &m_ts1);
}

CTimer::~CTimer()
{
  clock_gettime(CLOCK_MONOTONIC_RAW, &m_ts2);
  clock_gettime(CLOCK_MONOTONIC_RAW, &m_ts2);
  m_tsdiff = diff(m_ts1, m_ts2);

  double tt = (double) (m_ts2.tv_nsec - m_ts1.tv_nsec) / 1000000000L + (double) (m_ts2.tv_sec - m_ts1.tv_sec);

  std::cerr << m_desc << "total time " << tt << std::endl;

}


double CTimer::foo(double x)
{
  double s = 0;
  for (long int i = 0; i < (long int)(x*x) ; i++)
    s += x;

  return (s);
}
