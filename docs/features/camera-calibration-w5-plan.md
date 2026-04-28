# W5 — Dataset Lifecycle CLI (Detailed Plan)

## Context

W1–W4 record, gate, multi-camera-ingest, and (now) intrinsic-only-record
Kalibr datasets. Every successful `record-kalibr-dataset` calls
`rememberDataset`, which appends to `RuntimeConfig::kalibr_datasets` —
but nothing reads that vector back. The table is **write-only today**.

Operators end up with three problems:

1. **No way to see what's been recorded** without inspecting the SQLite
   file by hand. The future WebUI will want a JSON list it can render.
2. **No way to delete a stale dataset row** when the directory has been
   moved, the operator wants to clear failed runs, or disk is full.
3. **No matching disk cleanup** — every PNG and CSV the recorder wrote
   sticks around forever unless the operator manually `rm -rf`s it.

W5 adds two CLI subcommands that close the gap:

- `posest_daemon list-kalibr-datasets --config PATH [--json]` — print one
  line per dataset (or a JSON array) including an `exists_on_disk` flag.
- `posest_daemon delete-kalibr-dataset --config PATH --id ID
  [--remove-files]` — remove the row, optionally also recursively delete
  the dataset directory.

No schema change. No new structs. Mostly daemon plumbing + tests.

## Decisions baked in

- **Safety rail for `--remove-files`: trust the DB row, require directory
  to exist.** The operator passes `--id ID`, not a path. We look the id
  up in `kalibr_datasets`; if the row's `path` resolves to an existing
  directory, we recursively delete it. If the directory doesn't exist
  (e.g. row is stale because the operator already `rm -rf`d), we still
  remove the SQLite row but skip directory deletion. The operator
  cannot aim deletion at any path not previously inserted by
  `rememberDataset`.
- **JSON output includes SQL row data + an `exists_on_disk` flag.**
  Each entry: `{id, path, created_at, duration_s, camera_ids,
  exists_on_disk}`. No reads of per-dataset `session.json` — that's
  one extra file open per row for fields a future "show one dataset"
  subcommand can return.
- **Unknown id throws.** `delete-kalibr-dataset --id X` where X has no
  matching row throws `std::invalid_argument("no Kalibr dataset with
  id X")`. Consistent with every other config CLI's failure mode;
  scripted use can swallow exit codes if it wants idempotency.

## Critical files to modify

| File | What changes |
|---|---|
| `include/posest/runtime/Daemon.h` | Two new `DaemonCommand` enum values (`ListKalibrDatasets`, `DeleteKalibrDataset`); two new option structs (`ListKalibrDatasetsOptions`, `DeleteKalibrDatasetOptions`); two new fields on `DaemonOptions`. |
| `src/runtime/Daemon.cpp` | Two new subcommand parser branches; two new flag parsers (`--id`, `--json`, `--remove-files`); post-parse validation; `daemonUsage()` lines; two new dispatch branches in `runConfigCommand`. |
| `test/test_daemon.cpp` | Parse tests for both subcommands; rejection tests for missing `--id`; output-format tests via dispatch (using `InMemoryConfigStore` to avoid SQLite plumbing in the unit). |
| `test/test_kalibr_dataset_lifecycle.cpp` (new) | End-to-end dispatch tests over a real `SqliteConfigStore`: list captures stdout and round-trips the JSON; delete without `--remove-files` removes the row but leaves the directory; delete with `--remove-files` removes both; stale-path delete (row points at a deleted directory) succeeds with a row removal and no throw; unknown id throws. |

## Step-by-step

### Step 1 — Option structs + enum values

In `include/posest/runtime/Daemon.h`:

```cpp
enum class DaemonCommand {
    Run,
    CalibrateCamera,
    ImportFieldLayout,
    RecordKalibrDataset,
    MakeKalibrBag,
    CalibrateCameraImu,
    ImportCalibrationTarget,
    ListKalibrDatasets,         // NEW
    DeleteKalibrDataset,        // NEW
};

struct ListKalibrDatasetsOptions {
    bool json{false};
};

struct DeleteKalibrDatasetOptions {
    std::string id;
    bool remove_files{false};
};
```

Add both to `DaemonOptions`:

```cpp
ListKalibrDatasetsOptions list_kalibr_datasets;
DeleteKalibrDatasetOptions delete_kalibr_dataset;
```

### Step 2 — CLI parsing

In `parseDaemonOptions` (`src/runtime/Daemon.cpp`), add the two
subcommand recognizers next to the others:

```cpp
} else if (arg == "list-kalibr-datasets") {
    if (options.command != DaemonCommand::Run) {
        throw std::invalid_argument("only one subcommand may be provided");
    }
    options.command = DaemonCommand::ListKalibrDatasets;
} else if (arg == "delete-kalibr-dataset") {
    if (options.command != DaemonCommand::Run) {
        throw std::invalid_argument("only one subcommand may be provided");
    }
    options.command = DaemonCommand::DeleteKalibrDataset;
}
```

Add three new flag branches alongside the existing `--config` block:

```cpp
} else if (arg == "--json") {
    options.list_kalibr_datasets.json = true;
} else if (arg == "--id") {
    options.delete_kalibr_dataset.id = requireValue(i, argc, argv, arg);
} else if (arg == "--remove-files") {
    options.delete_kalibr_dataset.remove_files = true;
}
```

(`--id` is intentionally distinct from `--field-id`, `--target-id`, etc.;
the existing flags use specific names so reusing `--id` is unambiguous.)

Post-parse validation at the bottom of `parseDaemonOptions`:

```cpp
} else if (options.command == DaemonCommand::DeleteKalibrDataset) {
    if (options.delete_kalibr_dataset.id.empty()) {
        throw std::invalid_argument("delete-kalibr-dataset requires --id");
    }
}
```

`list-kalibr-datasets` has no required flags.

### Step 3 — `daemonUsage()` lines

Append:

```
       posest_daemon list-kalibr-datasets --config PATH [--json]
       posest_daemon delete-kalibr-dataset --config PATH --id ID [--remove-files]
```

### Step 4 — Dispatch: `ListKalibrDatasets`

In `runConfigCommand`, after the existing `RecordKalibrDataset` /
`MakeKalibrBag` branches:

```cpp
if (options.command == DaemonCommand::ListKalibrDatasets) {
    const auto config = config_store.loadRuntimeConfig();
    if (options.list_kalibr_datasets.json) {
        nlohmann::json out = nlohmann::json::array();
        for (const auto& dataset : config.kalibr_datasets) {
            out.push_back({
                {"id", dataset.id},
                {"path", dataset.path},
                {"created_at", dataset.created_at},
                {"duration_s", dataset.duration_s},
                {"camera_ids", dataset.camera_ids},
                {"exists_on_disk",
                 std::filesystem::is_directory(dataset.path)},
            });
        }
        std::cout << out.dump(2) << "\n";
    } else {
        for (const auto& dataset : config.kalibr_datasets) {
            const bool on_disk = std::filesystem::is_directory(dataset.path);
            std::cout << dataset.id << "\t"
                      << dataset.created_at << "\t"
                      << std::fixed << std::setprecision(1)
                      << dataset.duration_s << "s\t"
                      << "cameras=";
            for (std::size_t i = 0; i < dataset.camera_ids.size(); ++i) {
                if (i != 0) std::cout << ",";
                std::cout << dataset.camera_ids[i];
            }
            std::cout << "\t" << (on_disk ? "on_disk" : "missing") << "\n";
        }
    }
    return;
}
```

`std::cout` is already used elsewhere in the daemon (the health-once
path); no new include.

### Step 5 — Dispatch: `DeleteKalibrDataset`

```cpp
if (options.command == DaemonCommand::DeleteKalibrDataset) {
    auto config = config_store.loadRuntimeConfig();
    const auto& cmd = options.delete_kalibr_dataset;

    const auto it = std::find_if(
        config.kalibr_datasets.begin(),
        config.kalibr_datasets.end(),
        [&cmd](const KalibrDatasetConfig& entry) {
            return entry.id == cmd.id;
        });
    if (it == config.kalibr_datasets.end()) {
        throw std::invalid_argument(
            "no Kalibr dataset with id: " + cmd.id);
    }

    const std::filesystem::path dataset_path = it->path;
    config.kalibr_datasets.erase(it);
    config_store.saveRuntimeConfig(config);  // commits row removal first

    if (cmd.remove_files) {
        // Trust the DB row. The path was inserted by rememberDataset and
        // can only be removed-from once (we just erased the row). If the
        // directory has gone missing since the recording (operator
        // already cleaned it up), do nothing — the row removal alone
        // satisfies the operator's intent.
        std::error_code ec;
        if (std::filesystem::is_directory(dataset_path, ec)) {
            std::filesystem::remove_all(dataset_path, ec);
            if (ec) {
                throw std::runtime_error(
                    "failed to remove dataset directory " +
                    dataset_path.string() + ": " + ec.message());
            }
        }
    }
    return;
}
```

The save-then-delete order is intentional: if SQLite save fails we
haven't touched the disk yet; if disk remove fails the row is already
gone (re-running the command with `--remove-files` will be a no-op
because the row is missing — operator can `rm -rf` by hand to recover
the disk space).

### Step 6 — Tests: parse + dispatch

In `test/test_daemon.cpp`, append four parse tests:

- `ParsesListKalibrDatasetsCommand` — parses with and without `--json`.
- `ParsesDeleteKalibrDatasetCommand` — parses `--id X --remove-files`.
- `RejectsDeleteKalibrDatasetMissingId` — throws when `--id` absent.
- `ListAndDeleteShareSubcommandExclusivity` — providing both subcommand
  names on one CLI throws (existing pattern in
  `parseDaemonOptions`).

### Step 7 — End-to-end test file

Create `test/test_kalibr_dataset_lifecycle.cpp`. Use a real
`SqliteConfigStore` over a temp DB and a `FakeCameraFactory` (existing
in `test_daemon.cpp` — extract or duplicate). Each test:

1. **`ListEmitsRowsAsJsonOrText`** — pre-populate two
   `KalibrDatasetConfig` rows (both with a real temp directory on disk
   for one, missing path for the other). Redirect `std::cout` into a
   `std::stringstream`, run `runConfigCommand` with
   `DaemonCommand::ListKalibrDatasets` and `--json`. Parse the captured
   JSON, assert two entries, correct `exists_on_disk` flags. Re-run
   without `--json` and assert plain-text output contains both ids.
2. **`DeleteRemovesRowOnly`** — pre-populate one row pointing at a real
   temp directory. Run delete *without* `--remove-files`. Reload
   config; assert row gone; assert temp directory still exists.
3. **`DeleteRemovesRowAndDirectory`** — same setup; pass
   `--remove-files`; assert row gone AND directory gone.
4. **`DeleteToleratesMissingDirectory`** — pre-populate one row whose
   path does NOT exist on disk. `--remove-files` should still remove
   the row and not throw.
5. **`DeleteThrowsOnUnknownId`** — empty config, `--id missing` →
   `std::invalid_argument`.

These tests reuse `tempDbPath` already in `test_config_schema.cpp`; copy
the helper into the new file or extract to a shared header (extract
later — duplicate for now, smallest diff).

`std::cout` redirection idiom:

```cpp
std::stringstream captured;
auto* prior_buf = std::cout.rdbuf(captured.rdbuf());
posest::runtime::runConfigCommand(options, *store, factory);
std::cout.rdbuf(prior_buf);
```

Wrap in a small RAII guard so a thrown exception still restores
`std::cout`.

### Step 8 — Build + ctest

```
cmake --build --preset conan-release -j
ctest --preset conan-release --output-on-failure
```

Net delta: ~9 new tests (4 parse + 5 lifecycle). All 307 W4 tests must
continue passing.

## Reused functions / utilities

- `loadRuntimeConfig` / `saveRuntimeConfig` (`SqliteConfigStore`) —
  reused unchanged. Saving an unmodified-but-erased `kalibr_datasets`
  vector still goes through the existing `DELETE FROM kalibr_datasets
  ... INSERT INTO kalibr_datasets ...` save path and SQLite's atomic
  transaction.
- `KalibrDatasetConfig` — unchanged.
- `tempDbPath` (`test/test_config_schema.cpp`) — duplicated in the new
  test file for now.
- `FakeCameraFactory` / `FakePipelineFactory` (`test/test_daemon.cpp`)
  — duplicated in the new test file. Neither is invoked by the
  list/delete dispatch paths (no cameras built), but
  `runConfigCommand`'s signature requires the factory references.
- `std::filesystem::is_directory` / `remove_all` — standard library;
  already used in `Daemon.cpp` (e.g.
  `findKalibrCamchain`, `findKalibrResultsCam`).
- `nlohmann::json` — already in scope in `Daemon.cpp` for
  `healthToJson`.

## Verification

1. **Build + ctest:** all 307 tests + ~9 new ones pass.
2. **Parse-level coverage:**
   `./build/Release/posest_tests --gtest_filter='DaemonOptions*Kalibr*'`.
3. **Dispatch-level coverage:**
   `./build/Release/posest_tests --gtest_filter='KalibrDatasetLifecycle*'`.
4. **Manual smoke (post-merge):**
   ```
   posest_daemon record-kalibr-dataset --config posest.db \
       --camera-id cam0 --duration-s 30 --output-dir /tmp/cal-smoke \
       --require-imu no
   posest_daemon list-kalibr-datasets --config posest.db --json | jq .
   # Expect one entry with exists_on_disk=true, camera_ids=["cam0"].
   posest_daemon delete-kalibr-dataset --config posest.db \
       --id /tmp/cal-smoke --remove-files
   posest_daemon list-kalibr-datasets --config posest.db
   # Expect empty output.
   ls /tmp/cal-smoke
   # Expect: ls: cannot access '/tmp/cal-smoke'
   ```

## Out of scope for W5

- A "show one dataset in detail" subcommand that reads `session.json`
  for richer metadata. The current `kalibr_datasets` row is enough for
  list/delete; a future `show-kalibr-dataset --id ID` can layer on.
- Bulk delete (e.g., `--all-stale`, `--older-than 30d`). The future
  WebUI is the natural place to surface multi-select; the CLI keeps
  one-at-a-time semantics.
- Cascade-cleaning of `calibrations` / `camera_extrinsics` rows whose
  `source_file_path` referenced the deleted dataset. The Kalibr output
  is independent of the dataset directory once the calibration has
  been ingested — those rows stay valid.
- Migration to a `dataset_root` policy in `CalibrationToolConfig`. If
  operators later want a "datasets must be under X" guard, that's
  one column-add migration, not a structural change.
