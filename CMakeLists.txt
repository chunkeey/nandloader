cmake_minimum_required(VERSION 3.12)

project(nandloader.fw)

set(CMAKE_SYSTEM_NAME "Generic")
set(CMAKE_SYSTEM_PROCESSOR "mips")

set_property(DIRECTORY PROPERTY TARGET_SUPPORTS_SHARED_LIBS FALSE)
set(CMAKE_C_FLAGS "-EB -G 0 -Os -fno-strict-aliasing -fno-common -fomit-frame-pointer -ffunction-sections -pipe -mlong-calls -ffreestanding -nostartfiles -mno-abicalls -fno-pic -march=mips32r2 -Wa,-32 -Wa,-march=mips32r2 -Wa,-mips32r2 -Wa,--trap -mabi=32")
set(CMAKE_C_LINK_FLAGS "-static -Wl,--build-id=none,-EB,-x,--gc-sections,-nostdlib")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

set(OBJCOPY mips-linux-gnu-objcopy)
set(CMAKE_C_COMPILER mips-linux-gnu-gcc)
set(CMAKE_AR mips-linux-gnu-ar)
set(CMAKE_ASM_COMPILER mips-linux-gnu-as)
set(CMAKE_ASM-ATT_COMPILER mips-linux-gnu-as)
set(CMAKE_LINKER mips-linux-gnu-ld)
set(CMAKE_C_LINK_EXECUTABLE "mips-linux-gnu-gcc <OBJECTS> ${CMAKE_C_FLAGS} <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> -o <TARGET>")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(nandloader_src start.S main.c nand_flash.c printf.c bch.c malloc_compat.c sha1.c)

option(BUILD_MR18 "Build NANDLoader for MR18/QCA955x" ON)
option(BUILD_Z1 "Build NANDLoader for Z1/AR9344" OFF)

if (BUILD_MR18 AND (NOT BUILD_Z1))
	set(nandplat_src lowlevel_init_955x.S 955x.c)

	option(BUILD_SGMII_CAL "Build NANDLoader with SGMII/ETH calibration" ON)

	if (BUILD_SGMII_CAL)
		add_compile_definitions("MR18_SGMII_CAL")
	endif()

elseif ((NOT BUILD_MR18) AND BUILD_Z1)
	set(nandplat_src lowlevel_init_934x.S lowlevel_init_934x-1.1.S 934x.c)
else()
	message(FATAL_ERROR "It's either BUILD_MR18 or BUILD_Z1... and not both or neither")
endif()

set_source_files_properties(start.S PROPERTIES LANGUAGE C COMPILE_FLAGS -D__ASSEMBLY__)

set_source_files_properties(lowlevel_init-934x-1.1.S PROPERTIES LANGUAGE C COMPILE_FLAGS -D__ASSEMBLY__)
set_source_files_properties(lowlevel_init-934x.S PROPERTIES LANGUAGE C COMPILE_FLAGS -D__ASSEMBLY__)
set_source_files_properties(lowlevel_init-955x.S PROPERTIES LANGUAGE C COMPILE_FLAGS -D__ASSEMBLY__)

add_executable(nandloader.elf ${nandplat_src} ${nandloader_src})

add_compile_definitions("__KERNEL__")
add_compile_definitions("CONFIG_PAGE_SIZE_4KB")
add_compile_definitions("CONFIG_MIPS_L1_CACHE_SHIFT=5")
add_compile_definitions("CONFIG_32BIT")
add_compile_definitions("CONFIG_RWSEM_GENERIC_SPINLOCK")
add_compile_definitions("CONFIG_TINY_RCU")
add_compile_definitions("CONFIG_ZONE_DMA")
add_compile_definitions("CONFIG_HZ=100")
#add_compile_definitions("CONFIG_ATH_NAND_BR")
add_compile_definitions("MAX_NR_ZONES=2")


add_compile_definitions("BITS_PER_LONG=32")
#add_compile_definitions("_MIPS_SZLONG=32")

target_include_directories(nandloader.elf PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(nandloader.elf PROPERTIES LINKER_LANGUAGE C)

set_target_properties(nandloader.elf PROPERTIES LINK_FLAGS "-Tnandloader.lds")

add_custom_target(
	nandloader.tmp ALL
	${OBJCOPY} --strip-unneeded -O binary nandloader.elf nandloader.tmp
	DEPENDS nandloader.elf
	BYPRODUCTS nandloader.tmp)

add_custom_target(
	nandloader.fw ALL
	cp nandloader.tmp nandloader.fw && dd status=none bs=4 skip=4 if=nandloader.fw | perl checksum.pl | dd bs=4 seek=3 count=1 of=nandloader.fw conv=notrunc status=none
	DEPENDS nandloader.tmp
	BYPRODUCTS nandloader.fw)

