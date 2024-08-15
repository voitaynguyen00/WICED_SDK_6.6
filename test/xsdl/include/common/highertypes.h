#ifndef HIGHERTYPES_H_DEFINED


#define HIGHERTYPES_H_DEFINED



#include "basetype.h"



#include <string>
#include <map>
#include <vector>

using namespace std;






typedef map<string,string>      MAPSTR2STR;
typedef map<string,UINT32_t>    MAPSTR2UINT;
typedef vector<string>          VECSTR;

typedef vector<UINT64_t>        VECU64;
typedef vector<UINT32_t>        VECU32;
typedef vector<UINT16_t>        VECU16;
typedef vector<UINT8_t>         VECU8;
typedef vector<INT64_t>         VECI64;
typedef vector<INT32_t>         VECI32;
typedef vector<INT16_t>         VECI16;
typedef vector<INT8_t>          VECI8;


typedef vector<CVariableValue>  VECVARIABLES;




string      ExtractTrimmedString (const char * pFrom, const char * pTo, bool bForceUppercase);
string      Unquote (const char * pQuotedText, char cQuoteChar = '\"');

VECSTR      TokenizeLine (const char * sLine, bool bCapitalize, const char * sSingleCharacterTokensToRecognize = NULL);






// Contains an arbitrary NAME->VALUE pair, where both are strings.
class CNameValuePair
{
public:

    CNameValuePair ()
    {
        m_sName = "";
        m_sValue = "";
    }

    ~CNameValuePair ()
    {
        m_sName = "";
        m_sValue = "";
    }


    CNameValuePair (const CNameValuePair & cp)
    {
        m_sName = cp.m_sName;
        m_sValue = cp.m_sValue;
    }

    CNameValuePair& operator= (const CNameValuePair & cp)
    {
        m_sName = cp.m_sName;
        m_sValue = cp.m_sValue;
        return (*this);
    }

protected:

    string  m_sName;
    string  m_sValue;
};

typedef vector<CNameValuePair>  VECNAMEVALUEPAIRS;






// Contains NAME->VALUE pairs, where both are strings, and names are unique and case-insensitive.
class CNameValueTable
{
public:

    CNameValueTable ();

    ~CNameValueTable ();

    CNameValueTable (const CNameValueTable& cp);

    CNameValueTable& operator= (const CNameValueTable& cp);


    void            Clear ();

    UINT32_t        Count ();


    string          AddValue (const char * sName, const char * sValue); 
                                                            // Returns empty string if successful, 
                                                            // error description otherwise.

    string          RemoveValue (const char * sName); 
                                                            // Returns empty string if successful, 
                                                            // error description otherwise.

    const char *    FindValueFromName (const char * sName); // Returns NULL if name not found

    string          StoreTableToString ();

    string          LoadFromString (const char * sStored); // Returns empty string if successful, 
                                                            // error description otherwise.

    string          LoadFromFile (const char * sFileName);  // Returns empty string if successful, 
                                                            // error description otherwise.

	string			GetAllNamesAndValues (vector<string> & refOutNames, vector<string> & refOutValues);

private:

    MAPSTR2STR      m_mapTable;


    string          MakeNormalizedName (const char * sName);

    string          ParseValuesFromSingleLine (const char * sLine);
};








// Contains an arbitrary NAME,VALUE,ADDRESS32 triplet
class CNameValueAddress
{
public:

    CNameValueAddress ()
    {
        m_sName = "";
        m_sValue = "";
        m_nAddress = 0;
    }

    CNameValueAddress (const char * sName, const char * sValue, UINT32_t nAddress)
    {
        m_sName = sName;
        m_sValue = sValue;
        m_nAddress = nAddress;
    }

    ~CNameValueAddress ()
    {
    }


    CNameValueAddress (const CNameValueAddress & cp)
    {
        m_sName = cp.m_sName;
        m_sValue = cp.m_sValue;
        m_nAddress = cp.m_nAddress;
    }

    CNameValueAddress& operator= (const CNameValueAddress & cp)
    {
        m_sName = cp.m_sName;
        m_sValue = cp.m_sValue;
        m_nAddress = cp.m_nAddress;
        return (*this);
    }

    const char* GetName ()
    {
        return m_sName.c_str();
    }

    const char* GetValue ()
    {
        return m_sValue.c_str();
    }

    UINT32_t    GetAddress ()
    {
        return m_nAddress;
    }



protected:

    string      m_sName;
    string      m_sValue;
    UINT32_t    m_nAddress;
};



typedef vector<CNameValueAddress>       VECNAMEVALUEADDRESS;











#endif

