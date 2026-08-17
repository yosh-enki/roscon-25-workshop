#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "domain/target_identity.hpp"
#include "full_self_driving/msg/recovery_status.hpp"

namespace full_self_driving::persistence
{

enum class FaultStage : uint8_t
{
  NONE = 0,
  VALIDATE = 1,
  TEMP_WRITE = 2,
  FLUSH_FSYNC = 3,
  RENAME = 4,
  DIRECTORY_SYNC = 5,
  JOURNAL = 6,
  BACKUP = 7
};

struct StoragePaths
{
  std::string state_directory;
  std::string plan_directory;
  std::string evidence_directory;
  std::string backup_directory;
};

struct PayloadDurableState
{
  uint8_t commanded_state{0};
  bool cargo_loaded{false};
  bool secured{false};
  uint32_t successful_operations{0};
  std::string last_operation_id;
  uint8_t last_result{0};
  bool unknown_result{false};
};

struct ExecutorCheckpointState
{
  std::string phase{"WAITING_FOR_MODE"};
  std::string active_action;
  uint64_t durable_sequence{0};
};

struct MissionSnapshotRecord
{
  std::string schema_version{"1.0.0"};
  std::string mission_id;
  std::string sortie_id;
  uint64_t snapshot_revision{0};
  uint64_t durable_sequence{0};
  std::string resolved_config_hash;
  std::string map_id;
  std::string scenario_id;
  std::string plan_artifact_id;
  std::string working_plan_id;
  uint64_t working_plan_generation{0};
  domain::TargetIdentity target;
  PayloadDurableState payload;
  ExecutorCheckpointState executor;
  std::string checksum;

  std::string compute_checksum() const;
  bool is_valid() const;
};

struct JournalEntry
{
  uint64_t entry_sequence{0};
  std::string event_id;
  std::string idempotency_key;
  std::string mission_id;
  std::string sortie_id;
  std::string snapshot_hash;
  uint8_t severity{0};
  uint8_t source{0};
  std::string component;
  std::string detail;
  uint64_t timestamp_monotonic_ns{0};
  std::string checksum;

  std::string compute_checksum() const;
};

struct BackupRecord
{
  std::string backup_id;
  std::string source_type;
  std::string source_identifier;
  uint64_t revision{0};
  std::string checksum;
  std::string created_at_utc;
};

struct RecoveryResult
{
  bool is_clear{false};
  std::vector<uint16_t> ambiguity_codes;
  full_self_driving::msg::RecoveryStatus status;
};

class PersistenceManager
{
public:
  explicit PersistenceManager(const StoragePaths & paths = StoragePaths());
  ~PersistenceManager() = default;

  void set_paths(const StoragePaths & paths);
  const StoragePaths & get_paths() const { return paths_; }

  void set_fault_injection(FaultStage stage);
  FaultStage get_fault_injection() const { return fault_stage_; }

  bool commit_snapshot(
    const MissionSnapshotRecord & snapshot,
    std::string * out_error = nullptr);

  bool append_journal_entry(
    const JournalEntry & entry,
    std::string * out_error = nullptr);

  bool create_backup(
    const std::string & source_type,
    const std::string & source_identifier,
    const std::string & content,
    uint64_t revision,
    std::string * out_backup_id = nullptr,
    std::string * out_error = nullptr);

  std::optional<MissionSnapshotRecord> load_active_snapshot(
    std::string * out_error = nullptr) const;

  std::vector<JournalEntry> load_journal_entries(
    std::string * out_error = nullptr) const;

  std::vector<BackupRecord> list_backups() const;

  RecoveryResult recover(
    const std::string & current_resolved_config_hash,
    bool has_working_plan_check = true,
    bool is_working_plan_valid = true);

  bool resolve_recovery(
    uint8_t decision,
    uint64_t expected_recovery_revision,
    const std::string & confirmation,
    bool is_disarmed,
    std::string * out_error = nullptr);

  uint64_t get_durable_sequence() const { return durable_sequence_; }
  uint64_t get_recovery_revision() const { return recovery_revision_; }
  const full_self_driving::msg::RecoveryStatus & get_recovery_status() const { return recovery_status_; }

  void reset_state_for_test();

private:
  bool validate_storage_paths(std::string * out_error) const;
  bool atomic_write_file(
    const std::string & target_path,
    const std::string & content,
    std::string * out_error = nullptr);
  std::optional<MissionSnapshotRecord> load_active_snapshot_unlocked(
    std::string * out_error = nullptr) const;

  mutable std::mutex mutex_;
  StoragePaths paths_;
  FaultStage fault_stage_{FaultStage::NONE};

  uint64_t durable_sequence_{0};
  uint64_t recovery_revision_{1};
  full_self_driving::msg::RecoveryStatus recovery_status_;

  std::optional<MissionSnapshotRecord> active_snapshot_;
  std::vector<JournalEntry> journal_;
  std::vector<BackupRecord> backups_;
};

}  // namespace full_self_driving::persistence
