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
    unsigned int _MeasurementPoints;
    std::string mHostName;
    OmlValueU* _values;

    std::map<std::string, std::pair<OmlValueT, OmlValueU*> >            _KTVMap;
    std::map<std::string, std::pair<OmlValueT, OmlValueU*> >::iterator  _KTVMapIter;


    OmlMPDef* mp_def;
    OmlMP* _mp_handle;
    std::string _db_filename;
    std::string _server_name;

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


    void start( std::vector< std::pair<std::string, OmlValueT> >& oml_key_list );


    void set_key(std::string key_str, void* val_ptr);


    void insert()
    {
      omlc_inject (_mp_handle, _values);
    }

    void stop()
    {
      omlc_close();
      free(_values);
    }


};


void CWriteOml::init(std::string db_filename, std::string server_name)
{
  _db_filename = db_filename;
  _server_name = server_name;

  std::string fname;
  int argc;
  const char** argv;
  std::vector<char*> arg_vector;

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
    // Following two lines were being optimized out
    //const char* argv_server[] = {"./spectrum", "--oml-id",(const char*)"et", "--oml-domain",db_filename.c_str(), "--oml-collect", server_name.c_str() };
    //argv = argv_server;

    arg_vector.push_back((char*)"./spectrum");
    arg_vector.push_back((char*)"--oml-id");
    arg_vector.push_back((char*)"et");
    arg_vector.push_back((char*)"--oml-domain");
    arg_vector.push_back((char*)db_filename.c_str());
    arg_vector.push_back((char*)"--oml-collect");
    arg_vector.push_back((char*)server_name.c_str());
    argv = (const char**)&arg_vector[0];

    argc = arg_vector.size(); // argc = 7;

  }

  int result = omlc_init ("_mp_", &argc, argv, NULL);
  if (result == -1) {
    std::cerr << "Could not initialize OML\n";
    exit (1);
  }

}


void CWriteOml::start( std::vector<std::pair<std::string, OmlValueT> >& _OmlKeys )
{
  int result;

  _MeasurementPoints = _OmlKeys.size();

  mp_def = new OmlMPDef [(sizeof(OmlMPDef) * (_MeasurementPoints + 1) )];

  // define measurement points
  unsigned int idx;
  for (idx = 0; idx < _MeasurementPoints; ++idx)
    createMeasurementPoint(&mp_def[idx], _OmlKeys.at(idx).first, (OmlValueT)_OmlKeys.at(idx).second);
  createMeasurementPoint(&mp_def[idx],      "NULL",        (OmlValueT)0);

  _mp_handle = omlc_add_mp (_db_filename.c_str(), mp_def); // using db_filename as tag name for measurement point

  if (_mp_handle == NULL) {
    std::cerr << "Error: could not register Measurement Point \"data\"";
    exit (1);
  }

  result = omlc_start();
  if (result == -1) {
    std::cerr << "Error starting up OML measurement streams\n";
    exit (1);
  }

  // allocate memory measurement points
  _values = (OmlValueU*) malloc(sizeof(OmlValueU) * _MeasurementPoints);
  memset((void*)_values, 0, sizeof(OmlValueU) * _MeasurementPoints );

  // create oml key <==> (type,value) mapping
  _KTVMap.clear();
  for (unsigned int idx = 0; idx < _MeasurementPoints; ++idx) {
    std::pair<OmlValueT, OmlValueU*> TV (_OmlKeys.at(idx).second, (OmlValueU*)&_values[idx] );
    
    std::pair<std::string, std::pair<OmlValueT, OmlValueU*> > KTV( _OmlKeys.at(idx).first, TV );
    _KTVMap.insert( KTV );
  }

}


void CWriteOml::set_key(std::string key_str, void* val_ptr)
{

  _KTVMapIter = _KTVMap.find(key_str);
  if (_KTVMapIter == _KTVMap.end()) {
    std::cerr << key_str << " not found" << std::endl;
    return;  // key not found so return and do nothing
  }

  //key found to look at type are call appropriate oml intrinsic function
  std::pair<OmlValueT, OmlValueU*> TV = _KTVMapIter->second;
  switch( TV.first ) {
  case OML_INT32_VALUE :
    omlc_set_int32   ( *(TV.second), (int32_t) (*((int32_t*)val_ptr)));
    break;

  case OML_UINT32_VALUE :
    omlc_set_uint32   ( *(TV.second), (uint32_t) (*((uint32_t*)val_ptr)));
    break;

  case OML_INT64_VALUE :
    omlc_set_int64   ( *(TV.second), (int64_t) (*((int64_t*)val_ptr)));
    break;
  case OML_DOUBLE_VALUE :
    omlc_set_double   ( *(TV.second), (double) (*((double*)val_ptr)));
    break;
  case OML_STRING_VALUE :
    omlc_set_string( *(TV.second), (char*)val_ptr);
    break;
    // add other cases here

  default :
    std::cerr << "OML - unrecognizeg type, value: " << TV.first << " , " << TV.second << std::endl;
    break;
  }

  return;
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
