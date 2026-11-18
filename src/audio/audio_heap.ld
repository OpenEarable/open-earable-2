/* Common libc malloc uses the aligned RAM between _end and the SRAM boundary.
 * The seal waveform and LC3 sessions allocate from this space at runtime and
 * therefore do not appear in the linker's static RAM usage report.
 */
ASSERT(ALIGN(_end, 8) + CONFIG_AUDIO_MIN_RUNTIME_HEAP_SIZE <=
       CONFIG_SRAM_BASE_ADDRESS + CONFIG_SRAM_SIZE * 1024,
       "Not enough libc heap for audio; reduce static RAM usage")
