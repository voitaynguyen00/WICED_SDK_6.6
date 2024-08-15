// Structure fields can be viewed as a single level list (flat) or as a deep hierarchical structure.
// The default mode is flat.
// For the discussion, assume the following example:
//
//      struct A
//      {
//          int32   x, y;
//      }
//
//      struct B
//      {
//          int16   w;
//          int16   z;
//      }
//
//      struct Composite
//      {
//          int16   m;
//          int16   n;
//          A       arQ [2];
//          B       arR [3];
//          int32   arG [4];
//      }
//
// Then, if we are looking at an instance C of type Composite, we will see 16 fields in "flat view":
//      field "m" of type int16
//      field "n" of type int16
//      field "arQ[0].x" of type int32
//      field "arQ[0].y" of type int32
//      field "arQ[1].x" of type int32
//      field "arQ[1].y" of type int32
//      field "arR[0].w" of type int16
//      field "arR[0].z" of type int16
//      field "arR[1].w" of type int16
//      field "arR[1].z" of type int16
//      field "arR[2].w" of type int16
//      field "arR[2].z" of type int16
//      field "arG[0]" of type int32
//      field "arG[1]" of type int32
//      field "arG[2]" of type int32
//      field "arG[3]" of type int32
//
//
//
// Alternatively, in hierarchical mode, instance C will have 5 fields:
//      field "m" of type int16
//      field "n" of type int16
//      field "arQ" of type ARRAY
//              this is a separate object where items can be accessed by index
//              the size here is set to 2; the type is CDSDLInstance
//      field "arR" of type ARRAY
//              this is a separate object where items can be accessed by index
//              the size here is set to 3; the type is CDSDLInstance
//      field "arG" of type ARRAY
//              this is a separate object where items can be accessed by index
//              the size here is set to 4; the type is int32
//
//  In this mode, accessing "arQ" will return a pointer to a CDSDLArray instance with elements
//          [0] containing a CDSDLInstance with fields
//              "x" of type int32
//              "y" of type int32
//          [1] containing a CDSDLInstance with fields
//              "x" of type int32
//              "y" of type int32
//  Similarly, accessing "arR" will return a pointer to a CDSDLArray instance with elements
//          [0] containing a CDSDLInstance with fields
//              "w" of type int16
//              "z" of type int16
//          [1] containing a CDSDLInstance with fields
//              "w" of type int16
//              "z" of type int16
//          [2]containing a CDSDLInstance with fields
//              "w" of type int16
//              "z" of type int16
//  Finally, accessing "arG" will return a pointer to a CDSDLArray instance with elements
//          [0] of type int32
//          [1] of type int32
//          [2] of type int32
//          [3] of type int32


#if defined(_MSC_VER)
	#ifdef XSDL_EXPORTS
		#define XSDL_API __declspec(dllexport)
	#elif defined(XSDL_IMPORT)
		#define XSDL_API __declspec(dllimport)
	//	#define XSDL_LIB_FILE "xsdl" FILE_SUFFIX ".lib"
	//	#if !defined(XSDL_NOAUTOLINK)
	//		#pragma comment(lib, XSDL_LIB_FILE)
	//	#endif
	#else
		#define XSDL_API
	#endif
#elif defined(__GNUG__)	// __GNUC__ && __cplusplus
	#define XSDL_API
#elif defined(__APPLE__)
	#define XSDL_API
#endif

#ifndef DSDLCPPWRAPPER_H_DEFINED
#define DSDLCPPWRAPPER_H_DEFINED

class CDSDLField;
class CDSDLInstance;

class XSDL_API CDSDLBuffer
{
	friend class CDSDLFactory;

public:

	CDSDLBuffer ();
	~CDSDLBuffer ();

	explicit CDSDLBuffer (void* pBuffer);

	bool	Initialize (const char * sHexBytes);

	int		GetByteLength () const;

	int		GetBytesUsedUp () const;

	int		ReadNextByte ();						// Return -1 if invalid state

	int		PeekByteAtIndex (int nZeroBasedIndex);	// Return -1 if invalid state

	const unsigned char* GetByteBuffer() const;

private:

	void*	m_pBuffer;

};

class XSDL_API CDSDLFieldValue
{
    // A class that will keep a single field value of any type, number or text.
public:

	CDSDLFieldValue ();
	~CDSDLFieldValue ();
	
	explicit CDSDLFieldValue (void* pField);

    bool            IsValid () const; 
    // In non-flat mode, an INVALID instance of this object will be used to return field values that cannot exist 
    // (e.g. value of an array field or a substructure field).

	std::string     GetSymbolicName () const; // Non-NULL only if this value is some constant or enum defined in the DSDL

	std::string		AsNative() const;

    unsigned long   AsInteger () const;

    double          AsDouble () const;

    std::string     AsText () const;

private:

	void* m_pField;
};

class XSDL_API CDSDLArray
{
public:

	CDSDLArray();
	~CDSDLArray();

	explicit CDSDLArray(void* pArray);

    int             GetArraySize () const;

    CDSDLField*     GetElement (int nZeroBasedIndex);

private:

	void* m_pArray;
};

class XSDL_API CDSDLField
{
public:

	CDSDLField ();
	~CDSDLField ();

	CDSDLField (void* pField, bool bFlatMode);

	std::string     GetName () const;	

    // In flat mode, this will always return true.
    bool            IsScalar () const;

    // In flat mode, this will always return false.
    bool            IsArray () const;

    // In flat mode, this will always return false.
    bool            IsInstance () const;

	// indicates whether the field is const
	bool			IsConstant() const;

    // In flat mode, this will always return a valid type, e.g. "int32"
    std::string     GetScalarType () const;

    // In flat mode, this will always be valid.
    CDSDLFieldValue* GetValue () const;

    // In flat mode, this will always return NULL.
    CDSDLArray*     GetArrayInfo () const;

    // In flat mode, this will always return NULL.
    CDSDLInstance*  GetSubInstance () const;

    // The Set functions can be used to change a field value.
    // Return string contains an error if unsuccessful.

    std::string     SetValueFromText (const char * sValueAsText);

    std::string     SetValue (CDSDLFieldValue oValue);

    std::string     GetBitPositionWithinContainingInstance (int * nLeftmostBitIndex, int * nNumBits);
    // We count bit indices left to right starting from the msb of the first byte of the Instance 
    // (i.e. bit 0 is the msb of byte 0, bit 7 is the lsb of byte 0,
    //       bit 8 is the msb of byte 1, bit 15 is the msb of byte 1, etc.)

	// Return named property value, NULL if property is not set
	std::string		GetProperty(const char* name) const;

private:

	void* m_pField;			// type is actually field_proxy_t*

	bool m_bFlatMode;
};


class XSDL_API CDSDLInstance
{
public:

	CDSDLInstance ();
	~CDSDLInstance ();

	explicit CDSDLInstance (void* pInst, bool bFlatMode=true);

    void			SetFieldAccessModeFlat (bool bFlatMode /* false --> hierarchical field access */ );

    // In flat mode, this will return the total number of fields.
    // In hierarchical mode will return the count of fields at this level only.
    int				CountFields () const;

    CDSDLField*		GetFieldByIndex (int nZeroBasedIndex) const;

    int				FindFieldIndexFromName (const char * sFieldName) const;

    CDSDLField*		FindFieldFromName (const char * sFieldName) const;

    int				GetTotalByteLength () const;

	CDSDLBuffer*	GetByteBuffer () const;

	std::string		GetByteBufferAsHexStr () const;

	std::string		GetTypeName () const;

	std::string		GetAllFields (std::string const& sDelimiter="\n") const;

	// Return named property value, NULL if property is not set
	std::string		GetProperty(const char* name) const;

private:

	void*	m_pInst;		// type is actually inst_proxy_t*

	bool    m_bFlatMode;

};


class XSDL_API CDSDLInstanceCollection
{
public:

	CDSDLInstanceCollection ();
	~CDSDLInstanceCollection ();

	explicit CDSDLInstanceCollection (void* pCollection);

    int             CountInstances () const;

    CDSDLInstance* GetInstanceByIndex (int nZeroBasedIndex) const;

private:

	void* m_pCollection;
};

class XSDL_API CDSDLFactory
{
public:

	CDSDLFactory ();
	~CDSDLFactory ();

    std::string     Open (const char * sDefinitionFile, const char * sOptions);

    std::string     Close ();

	CDSDLInstance*				CreateInstanceOfType (const char * sTypeName, const char * sFieldInitializerString);

	CDSDLInstanceCollection*	CreateInstancesFromBytes (const char * arBytes, int nNumBytes, int * nNumBytesUsedUp);

	CDSDLInstanceCollection*	CreateInstancesFromHexByteString (const char * sHexDataBytes,
																    int * nNumBytesUsedUp, char** sBytesRemaining);

	CDSDLInstanceCollection*    CreateInstancesFromBufferObject (CDSDLBuffer * pBuffer);

	size_t GetListOfTypes(std::vector<std::string>& types, bool bIgnoreAbstract = false) const;

	size_t GetListOfInstances(std::vector<std::string>& ins) const;

	std::string GetEntityProperty(std::string const& name, std::string const& prop) const;

	bool IsEntityAbstract(std::string const& name) const;

	size_t GetEnumarators(std::string const& name, std::vector<std::pair<std::string, int32_t> >& enumerators) const;

	bool GetEnumarator(std::string const& fname, int32_t& enumerator) const;

private:

	void* m_pFactory;		// type is actually factory_proxy_t*

};

#endif /* DSDLCPPWRAPPER_H_DEFINED */