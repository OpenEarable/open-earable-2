/* Newlib uses the RAM between _end and the physical SRAM boundary. The seal
 * waveform and LC3 sessions allocate from this space at runtime, so they do
 * not appear in the linker's static RAM usage report.
 */
ASSERT(_end + CONFIG_NEWLIB_LIBC_MIN_REQUIRED_HEAP_SIZE <=
       CONFIG_SRAM_BASE_ADDRESS + CONFIG_SRAM_SIZE * 1024,
       "Not enough Newlib heap for audio; reduce static RAM usage")
