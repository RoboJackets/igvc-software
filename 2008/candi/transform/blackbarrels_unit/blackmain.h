#ifndef blackmain_h
#define blackmain_h



#if __cplusplus
namespace blackbarrels{

#endif

	void blackmain (void);
	
#if __cplusplus
}
extern "C"{
#endif
/* --- Public C Functions --- */
	void getBlackedImage(Image* infr2,Image* imout);
#if __cplusplus
}
#endif

#endif
