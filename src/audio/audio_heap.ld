/* Newlib uses the RAM between _end and the physical SRAM boundary. LC3 needs
 * this space at runtime, even though its allocations do not appear in the
 * linker's static RAM usage report.
 */
ASSERT(_end + CONFIG_NEWLIB_LIBC_MIN_REQUIRED_HEAP_SIZE <=
       CONFIG_SRAM_BASE_ADDRESS + CONFIG_SRAM_SIZE * 1024,
       "Not enough Newlib heap for audio; reduce static RAM usage")
