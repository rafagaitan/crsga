#pragma once

#if defined(_MSC_VER)
#pragma warning( disable : 4251 )
#endif

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)
#  if defined( CRSGA_STATIC_LIB ) || defined( USE_STATIC )
#    define CRSGA_EXPORT
#  elif defined( CRSGA_LIBRARY )
#    define CRSGA_EXPORT   __declspec(dllexport)
#  else
#    define CRSGA_EXPORT   __declspec(dllimport)
#  endif /* CRSGA_EXPORT */
#else
#  define CRSGA_EXPORT
#endif  

// set up define for whether member templates
// are supported by VisualStudio compilers.
#ifdef _MSC_VER
# if (_MSC_VER >= 1300)
#  define __STL_MEMBER_TEMPLATES
# endif
#endif

/* Define NULL pointer value */

#ifndef NULL
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif


