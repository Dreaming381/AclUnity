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
	//! If the patch is greater than 500, subtract 500 and
	//! you get the experimental patch version.
	ACL_UNITY_API int getVersion();

	//! Returns the packed Major, minor, and patch version
	//! of ACL_Unity packed as follows: \n
	//! version = patch \n
	//! version |= minor << 10 \n
	//! version |= Major << 20 \n
	//! If the version is unrecognized, -1 is returned.
	//! //! If the patch is greater than 500, subtract 500 and
	//! you get the experimental patch version.
	ACL_UNITY_API int getUnityVersion();
}