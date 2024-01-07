#include "acl_unity_defines.h"

#include "acl/core/compressed_tracks_version.h"

using namespace acl;

ACL_UNITY_API int getVersion()
{
	// Sanity check
	if (compressed_tracks_version16::latest == compressed_tracks_version16::v02_01_00)
	{
		return (2 << 20) | (1 << 10) | 0;
	}
	
	return -1;
}

ACL_UNITY_API int getUnityVersion()
{
	return (0 << 20) | (9 << 10) | 0;
}