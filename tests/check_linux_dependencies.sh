#!/bin/bash
# Fails if a Linux build of the plugin depends on the C++ runtime or on a newer glibc than Ubuntu 22.04 provides.
# usage: check_linux_dependencies.sh <library.so>...

MAX_GLIBC="GLIBC_2.35"
status=0

for lib in "$@"; do
	echo "== $lib"
	readelf -d "$lib" | grep NEEDED

	if readelf -d "$lib" | grep NEEDED | grep -q 'libstdc++\|libc++'; then
		echo "::error::$lib depends on a shared C++ runtime"
		status=1
	fi

	required=$(objdump -T "$lib" | grep -o 'GLIBC_[0-9][0-9.]*' | sort -u -V | tail -n 1)
	echo "Highest glibc symbol version required: $required"
	if [ "$(printf '%s\n%s\n' "$MAX_GLIBC" "$required" | sort -V | tail -n 1)" != "$MAX_GLIBC" ]; then
		echo "::error::$lib requires $required, newer than $MAX_GLIBC"
		status=1
	fi

	exported=$(objdump -T "$lib" | awk '$2 == "g" && $4 == ".text" {print $NF}' | sort)
	echo "Exported functions: $(echo "$exported" | wc -l)"
	unexpected=$(echo "$exported" | grep -v -E '^(compress|dispose|get|sample)[A-Za-z]*$')
	if [ -n "$unexpected" ]; then
		echo "::error::$lib exports symbols outside the AclUnity API:"
		echo "$unexpected"
		status=1
	fi
done

exit $status
