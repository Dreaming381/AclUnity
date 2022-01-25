#pragma once

#if _MSC_VER
#define ACL_UNITY_API __declspec(dllexport)
#else
#define ACL_UNITY_API
#endif

extern "C"
{
	//! Returns the packed Major, minor, and patch version
	//! of ACL packed as follows: \n
	//! version = patch \n
	//! version |= minor << 10 \n
	//! version |= Major << 20 \n
	//! If the version is unrecognized, -1 is returned.
	ACL_UNITY_API int getVersion();
}