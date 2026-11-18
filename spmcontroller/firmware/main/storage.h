#ifndef STORAGE_H
#define STORAGE_H
// ---------------------------------------------------------------------------
// Session numbering, NVS boot log and the boot-time NOR -> SD dump policy.
// Ported from the Avionics B coprocessor (main.c) and shared byte-identical
// between the Avionics C and Training Board repos.
//
// Boot sequence expected by this module:
//   storage_nvs_init() -> nor_log_init() -> storage_session_begin()
//   -> storage_bootlog_begin() -> mount SD -> storage_boot_dump_and_wipe()
//   -> storage_open_session_file() -> start writer tasks
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <esp_err.h>

// nvs_flash_init() with the usual "erase and retry" fallback.
esp_err_t storage_nvs_init(void);

// Increment and return the persisted session number (1 on a fresh NVS).
uint32_t storage_session_begin(void);
// Raise the persisted counter to at least n (after an on-card name collision).
void storage_session_floor(uint32_t n);

// Boot log: last 10 boots with reset reason and dump outcome, in NVS.
// begin() records this boot (before the dump) and prints the ring; finish()
// records how the dump went. A boot that resets mid-dump stays INCOMPLETE.
void storage_bootlog_begin(uint32_t session, bool nor_ok, uint32_t nor_pending);
void storage_bootlog_finish(bool sd_ok, bool dump_done, bool nothing_to_dump);

// Every boot: dump the whole NOR backlog to <mount>/nor_dump_NNNNNN.bin (never
// overwriting an existing file), and wipe the NOR only after a verified dump.
// Records the outcome in the boot log. Returns true if the NOR is now empty.
bool storage_boot_dump_and_wipe(const char *mount_point, uint32_t session);

// Open <mount>/session_NNNNNN.bin for writing, advancing *session past any
// existing file (never overwrite) and persisting the bump. NULL on failure.
FILE *storage_open_session_file(const char *mount_point, uint32_t *session, size_t buffer_size);

#endif // STORAGE_H
