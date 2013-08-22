#!/bin/bash
#===============================================================================
#
#  build.sh
#
#  Copyright (C) 2010 by Digi International Inc.
#  All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify it
#  under the terms of the GNU General Public License version 2 as published by
#  the Free Software Foundation.
#
#
#  !Description: Interface script for U-Boot autobuild
#
#===============================================================================

set -e

basedir="$(cd $(dirname ${0}) && pwd)"
toolchain_dir="$(cd ${basedir}/.. && pwd)/toolchain"

buildserver="http://build-linux.digi.com/U-Boot/toolchain"

declare -r AVAILABLE_PLATFORMS="
	ccardmx28js
	ccardmx28js_dbg
	ccardmx28js_test
	ccardwmx28js
	ccardwmx28js_261MHz
	ccardwmx28js_360MHz
	ccardwmx28js_dbg
	ccardwmx28js_test
	ccmx51js
	ccmx51js_128sdram
	ccmx51js_128sdram_dbg
	ccmx51js_128sdram_ext_eth
	ccmx51js_128sdram_test
	ccmx51js_128sdram_test_dbg
	ccmx51js_dbg
	ccmx51js_ext_eth
	ccmx51js_test
	ccmx51js_test_dbg
	ccmx53js
	ccmx53js_4Kpage
	ccmx53js_dbg
	ccmx53js_ext_eth
	ccmx53js_test
	ccmx53js_test_dbg
	ccwmx51js
	ccwmx51js_128sdram
	ccwmx51js_128sdram_dbg
	ccwmx51js_128sdram_ext_eth
	ccwmx51js_128sdram_test
	ccwmx51js_128sdram_test_dbg
	ccwmx51js_dbg
	ccwmx51js_EAK
	ccwmx51js_ext_eth
	ccwmx51js_test
	ccwmx51js_test_dbg
	ccwmx53js
	ccwmx53js_4Kpage
	ccwmx53js_dbg
	ccwmx53js_ext_eth
	ccwmx53js_test
	ccwmx53js_test_dbg
	cpx2
	wr21
"

# <platform> <toolchain> <bootstreams>
while read pl to bs; do
	eval "${pl}_toolchain=\"${to}\""
	eval "${pl}_bootstream=\"${bs}\""
done<<-_EOF_
        ccardmx28js     arm-unknown-linux-gnueabi       y
        ccardwmx28js    arm-unknown-linux-gnueabi       y
        ccmx51js        arm-cortex_a8-linux-gnueabi     n
        ccmx53js        arm-cortex_a8-linux-gnueabi     n
        ccwmx51js       arm-cortex_a8-linux-gnueabi     n
        ccwmx53js       arm-cortex_a8-linux-gnueabi     n
        cpx2            arm-unknown-linux-gnueabi       y
        wr21            arm-unknown-linux-gnueabi       y
_EOF_

while getopts "i:" c; do
	case "${c}" in
		i) install_dir="${OPTARG%/}";;
	esac
done
shift $((${OPTIND} - 1))

# install directory is mandatory
if [ -z "${install_dir}" ]; then
	printf "\n[ERROR] missing \"-i\" (install_dir) option (mandatory)\n\n"
	exit 1
fi

cd "${basedir}"

if [ ! -f .toolchain_installed ]; then
	# DENX toolchain is the default for u-boot-denx repository
	toolchain_tag="denx"
	if uboot_tag="$(git name-rev --tags --name-only --no-undefined HEAD 2>/dev/null)"; then
		uboot_tag="${uboot_tag%%^*}"
	fi
	if uboot_branch="$(git rev-parse --symbolic-full-name @{u} 2>/dev/null)"; then
		uboot_branch="$(echo ${uboot_branch#refs/remotes/$(git remote)/} | tr '/' '_')"
	fi
	for i in ${uboot_tag} ${uboot_branch}; do
		if wget -q --spider "${buildserver}/toolchain-${i}.tar.gz"; then
			toolchain_tag="${i}"
			printf "\n[INFO] Installing toolchain-${i}.tar.gz\n\n"
			break
		else
			printf "\n[WARNING] File toolchain-${i}.tar.gz not found on webserver.\n\n"
		fi
	done
	rm -rf ${toolchain_dir} && mkdir ${toolchain_dir}
	wget -q -O - "${buildserver}/toolchain-${toolchain_tag}.tar.gz" | tar xz -C ${toolchain_dir} -f -
	touch .toolchain_installed
fi

if [ "${#}" = "0" ]; then
	set ${AVAILABLE_PLATFORMS}
fi

OLDPATH="${PATH}"

CPUS="$(echo /sys/devices/system/cpu/cpu[0-9]* | wc -w)"
[ ${CPUS} -gt 1 ] && MAKE_JOBS="-j${CPUS}"

for platform; do
	eval _toolchain_str=\"\${${platform%%_*}_toolchain}\"
	eval _has_bootstreams=\"\${${platform%%_*}_bootstream}\"
	export PATH="${toolchain_dir}/x-tools/${_toolchain_str}/bin:${OLDPATH}"

	printf "\n[PLATFORM: ${platform} - PATH: ${PATH}]\n"

	make distclean
	make "${platform}_config"
	make ${MAKE_JOBS}
	if [ -d "${install_dir}" ]; then
		cp "u-boot-${platform}.bin" "${install_dir}/"
		cp System.map "${install_dir}/u-boot-${platform}.map"
		if [ "${_has_bootstreams}" = "y" ]; then
			cp "u-boot-${platform}.sb" "${install_dir}/"
			cp "u-boot-${platform}-ivt.sb" "${install_dir}/"
		fi
		# Rename 'ccwmx51js' JSK images for backwards compatibility
		if [ "${platform}" = "ccwmx51js" ]; then
			mv ${install_dir}/u-boot-${platform}.bin ${install_dir}/u-boot-${platform}_JSK.bin
			mv ${install_dir}/u-boot-${platform}.map ${install_dir}/u-boot-${platform}_JSK.map
		fi
	fi
done
