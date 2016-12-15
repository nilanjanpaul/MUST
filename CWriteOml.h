#ifndef _CWRITEOML_H_
#define _CWRITEOML_H_

#include <iostream>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include "oml2/omlc.h"


#define FALSE (unsigned int) 0
#define TRUE  (unsigned int) 1


class CWriteOml
{
  private:
    unsigned int mMeasurementPoints;
    unsigned int mIdxNull;
    unsigned int _mp_idx;
    std::string mHostName;

    OmlMPDef* mp_def;
    OmlMP* mp;


    void createMeasurementPoint(OmlMPDef* pOmlMPDef, std::string str, OmlValueT type);

  public:
    CWriteOml() { }

    ~CWriteOml()
    {
      omlc_close();
    }


    void CWriteOML(std::string db_filename, std::string server_name)
    {
      init(db_filename, server_name);
    }    

    void init(std::string db_filename, std::string server_name);
    void start();

    void insert(uint32_t samp, float cf, uint32_t gain, uint32_t len, uint32_t channel, uint32_t peakidx, float peakMag, float estimate_cf)
    {
      // Modify here
      OmlValueU values[mMeasurementPoints];
      omlc_set_long   (values[0], samp);
      omlc_set_double (values[1], cf);
      omlc_set_long   (values[2], gain);
      omlc_set_long   (values[3], len);
      omlc_set_long   (values[4], channel);
      omlc_set_long   (values[5], peakidx);
      omlc_set_double (values[6], peakMag);
      omlc_set_double (values[7], estimate_cf);
      omlc_inject (mp, values);
    }

    void stop()
    {
      omlc_close();
    }


};


void CWriteOml::init(std::string db_filename, std::string server_name)
{
  std::string fname;
  int argc;
  const char** argv;

  //char chostname[32];
  //for (int i = 0; i < 32;++i)
  //  chostname[i] = '\0';
  //gethostname(chostname, 31);
  //mHostName = std::string(chostname);

  std::string mode(server_name.c_str());

  if (mode == "file")
    {
      fname = db_filename;//  + "_" +  mHostName;
      std::cout << fname << std::endl;
      argc = 7;
      const char* argv_file[] = {"./spectrum", "--oml-id",(const char*)"et", "--oml-exp-id",db_filename.c_str(), "--oml-file",fname.c_str()};
      argv = argv_file;
    }
  else
    {
      argc = 7;
      const char* argv_server[] = {"./spectrum", "--oml-id",(const char*)"et", "--oml-domain",db_filename.c_str(), "--oml-collect", server_name.c_str()
      };
      argv = argv_server;
    }

  int result = omlc_init ("frequency_offset_measurement_app", &argc, argv, NULL);
  if (result == -1) {
    std::cerr << "Could not initialize OML\n";
    exit (1);
  }

  _mp_idx = 0;
}

void CWriteOml::start()
{
  int result;
  mMeasurementPoints = 8; // Modify this

  mp_def = new OmlMPDef [(sizeof(OmlMPDef) * (mMeasurementPoints + 1) )];

  // Modify here
  createMeasurementPoint(&mp_def[_mp_idx++],    "sampling",    (OmlValueT)OML_INT32_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "cfreq_MHz",   (OmlValueT)OML_DOUBLE_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "gain_dB",     (OmlValueT)OML_INT32_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "FFTLength",   (OmlValueT)OML_INT32_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "Channel",     (OmlValueT)OML_INT32_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "PeakIdx",     (OmlValueT)OML_INT32_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "PeakMag",     (OmlValueT)OML_DOUBLE_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx++],    "Estimate",    (OmlValueT)OML_DOUBLE_VALUE);
  createMeasurementPoint(&mp_def[_mp_idx],      "NULL",        (OmlValueT)0);

  mp = omlc_add_mp ("peak_bin_detected_stat", mp_def);

  if (mp == NULL) {
    std::cerr << "Error: could not register Measurement Point \"data\"";
    exit (1);
  }

  result = omlc_start();
  if (result == -1) {
    std::cerr << "Error starting up OML measurement streams\n";
    exit (1);
  }
}



void CWriteOml::createMeasurementPoint(OmlMPDef* pOmlMPDef, std::string str, OmlValueT type)
{
  char* cptr;
  if (str == "NULL")
    {
      pOmlMPDef->name = NULL;
      pOmlMPDef->param_types = type;

    }
  else
    {
      cptr = new char[str.size()+1];
      strcpy (cptr, str.c_str());
      pOmlMPDef->name = cptr;
      pOmlMPDef->param_types = type;
    }

}


#endif
