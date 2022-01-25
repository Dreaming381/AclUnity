#include "acl_unity_defines.h"

#include "acl/core/compressed_tracks_version.h"

using namespace acl;

ACL_UNITY_API int getVersion()
{
	// Sanity check
	if (compressed_tracks_version16::latest == compressed_tracks_version16::v02_00_00)
	{
		return (2 << 20) | 4;
	}
	
	return -1;
}