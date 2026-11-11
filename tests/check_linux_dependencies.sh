#!/bin/bash
# Fails if a Linux build of the plugin needs newer system libraries than Ubuntu 22.04 provides (glibc 2.35, GCC 12's libstdc++),
# depends on libc++, does not link the C++ runtime it uses, or exports symbols outside the AclUnity API.
# usage: check_linux_dependencies.sh <library.so>...

status=0

# usage: check_max_version <library> <symbol version prefix> <highest allowed version>
check_max_version() {
	local lib=$1 prefix=$2 max=$3
	local required
	required=$(objdump -T "$lib" | grep -o "${prefix}[0-9][0-9.]*" | sort -u -V | tail -n 1)
	if [ -z "$required" ]; then
		echo "No ${prefix} symbol versions required"
		return
	fi
	echo "Highest ${prefix} symbol version required: $required"
	if [ "$(printf '%s\n%s\n' "$max" "$required" | sort -V | tail -n 1)" != "$max" ]; then
		echo "::error::$lib requires $required, newer than $max"
		echo "Symbols requiring a newer version:"
		objdump -T "$lib" | awk -v pattern="${prefix}[0-9.]+" '$0 ~ pattern { match($0, pattern); print substr($0, RSTART, RLENGTH), $NF }' | while read -r version name; do
			if [ "$(printf '%s\n%s\n' "$max" "$version" | sort -V | tail -n 1)" != "$max" ]; then
				echo "  $version $name"
			fi
		done
		status=1
	fi
}

for lib in "$@"; do
	echo "== $lib"
	needed=$(readelf -d "$lib" | grep NEEDED)
	echo "$needed"

	if echo "$needed" | grep -q 'libc++'; then
		echo "::error::$lib depends on libc++, which is not installed on most distributions"
		status=1
	fi
	if objdump -T "$lib" | grep -q '\*UND\*.*__cxa_\|\*UND\*.*__gxx_personality' && ! echo "$needed" | grep -q 'libstdc++'; then
		echo "::error::$lib uses the C++ runtime but does not link libstdc++"
		status=1
	fi

	check_max_version "$lib" "GLIBC_" "GLIBC_2.35"
	check_max_version "$lib" "GLIBCXX_" "GLIBCXX_3.4.30"
	check_max_version "$lib" "CXXABI_" "CXXABI_1.3.13"

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
