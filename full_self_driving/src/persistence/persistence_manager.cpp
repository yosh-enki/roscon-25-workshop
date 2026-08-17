#include "persistence/persistence_manager.hpp"

#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <openssl/evp.h>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace full_self_driving::persistence
{

static std::string compute_sha256(const std::string & data)
{
  unsigned char hash[EVP_MAX_MD_SIZE];
  size_t hash_len = 0;
  EVP_Q_digest(nullptr, "SHA256", nullptr, data.data(), data.size(), hash, &hash_len);

  std::ostringstream hex_stream;
  hex_stream << std::hex << std::setfill('0');
  for (size_t i = 0; i < hash_len; ++i) {
    hex_stream << std::setw(2) << static_cast<int>(hash[i]);
  }
  return hex_stream.str();
}

std::string MissionSnapshotRecord::compute_checksum() const
{
  std::ostringstream ss;
  ss << "schema=" << schema_version << "\n"
     << "mission_id=" << mission_id << "\n"
     << "sortie_id=" << sortie_id << "\n"
     << "snapshot_revision=" << snapshot_revision << "\n"
     << "durable_sequence=" << durable_sequence << "\n"
     << "resolved_config_hash=" << resolved_config_hash << "\n"
     << "map_id=" << map_id << "\n"
     << "scenario_id=" << scenario_id << "\n"
     << "plan_artifact_id=" << plan_artifact_id << "\n"
     << "working_plan_id=" << working_plan_id << "\n"
     << "working_plan_generation=" << working_plan_generation << "\n"
     << "target.marker_id=" << target.marker_id << "\n"
     << "target.dictionary=" << target.dictionary << "\n"
     << "target.namespace=" << target.target_namespace << "\n"
     << "payload.cmd=" << static_cast<int>(payload.commanded_state) << "\n"
     << "payload.loaded=" << (payload.cargo_loaded ? "1" : "0") << "\n"
     << "payload.secured=" << (payload.secured ? "1" : "0") << "\n"
     << "payload.succ_ops=" << payload.successful_operations << "\n"
     << "payload.last_op_id=" << payload.last_operation_id << "\n"
     << "payload.last_res=" << static_cast<int>(payload.last_result) << "\n"
     << "payload.unknown=" << (payload.unknown_result ? "1" : "0") << "\n"
     << "executor.phase=" << executor.phase << "\n"
     << "executor.action=" << executor.active_action << "\n"
     << "executor.durable_seq=" << executor.durable_sequence << "\n";
  return compute_sha256(ss.str());
}

bool MissionSnapshotRecord::is_valid() const
{
  if (schema_version != "1.0.0") return false;
  if (map_id.empty() || scenario_id.empty()) return false;
  if (resolved_config_hash.empty()) return false;
  if (checksum.empty() || checksum != compute_checksum()) return false;
  return true;
}

std::string JournalEntry::compute_checksum() const
{
  std::ostringstream ss;
  ss << "seq=" << entry_sequence << "\n"
     << "event_id=" << event_id << "\n"
     << "idempotency_key=" << idempotency_key << "\n"
     << "mission_id=" << mission_id << "\n"
     << "sortie_id=" << sortie_id << "\n"
     << "snapshot_hash=" << snapshot_hash << "\n"
     << "severity=" << static_cast<int>(severity) << "\n"
     << "source=" << static_cast<int>(source) << "\n"
     << "component=" << component << "\n"
     << "detail=" << detail << "\n"
     << "time=" << timestamp_monotonic_ns << "\n";
  return compute_sha256(ss.str());
}

PersistenceManager::PersistenceManager(const StoragePaths & paths)
: paths_(paths)
{
  recovery_status_.state = full_self_driving::msg::RecoveryStatus::STATE_CLEAR;
  recovery_status_.safe_decision_required = false;
  recovery_status_.decision_revision = recovery_revision_;
  recovery_status_.durable_snapshot_sequence = 0;
}

void PersistenceManager::set_paths(const StoragePaths & paths)
{
  std::lock_guard<std::mutex> lock(mutex_);
  paths_ = paths;
}

void PersistenceManager::set_fault_injection(FaultStage stage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fault_stage_ = stage;
}

void PersistenceManager::reset_state_for_test()
{
  std::lock_guard<std::mutex> lock(mutex_);
  durable_sequence_ = 0;
  recovery_revision_ = 1;
  fault_stage_ = FaultStage::NONE;
  active_snapshot_.reset();
  journal_.clear();
  backups_.clear();

  recovery_status_.state = full_self_driving::msg::RecoveryStatus::STATE_CLEAR;
  recovery_status_.safe_decision_required = false;
  recovery_status_.decision_revision = 1;
  recovery_status_.durable_snapshot_sequence = 0;
  recovery_status_.ambiguity_codes.clear();
  recovery_status_.has_last_valid_snapshot_hash = false;
  recovery_status_.last_valid_snapshot_hash.clear();
}

bool PersistenceManager::validate_storage_paths(std::string * out_error) const
{
  if (paths_.state_directory.empty()) {
    if (out_error) *out_error = "State directory path is not set";
    return false;
  }

  const std::vector<std::string> all_paths = {
    paths_.state_directory,
    paths_.plan_directory,
    paths_.evidence_directory,
    paths_.backup_directory
  };

  for (const auto & p_str : all_paths) {
    if (p_str.empty()) continue;
    if (p_str.find("..") != std::string::npos) {
      if (out_error) *out_error = "Storage path contains prohibited path traversal '..': " + p_str;
      return false;
    }
    if (p_str.rfind("/opt/ros", 0) == 0 || p_str.rfind("/usr/share", 0) == 0) {
      if (out_error) *out_error = "Storage path must not be in system package share: " + p_str;
      return false;
    }
  }

  return true;
}

bool PersistenceManager::atomic_write_file(
  const std::string & target_path,
  const std::string & content,
  std::string * out_error)
{
  std::filesystem::path tgt(target_path);
  std::filesystem::path parent = tgt.parent_path();

  std::error_code ec;
  std::filesystem::create_directories(parent, ec);
  if (ec) {
    if (out_error) *out_error = "Failed to create directory: " + parent.string();
    return false;
  }

  // 1. Stage: TEMP_WRITE
  if (fault_stage_ == FaultStage::TEMP_WRITE) {
    if (out_error) *out_error = "Injected fault: TEMP_WRITE";
    return false;
  }

  static uint64_t tmp_counter = 0;
  std::filesystem::path tmp_path = parent / (tgt.filename().string() + ".tmp." +
    std::to_string(getpid()) + "." + std::to_string(++tmp_counter));

  int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    if (out_error) *out_error = "Failed to create temp file: " + tmp_path.string();
    return false;
  }

  ssize_t written = ::write(fd, content.data(), content.size());
  if (written != static_cast<ssize_t>(content.size())) {
    ::close(fd);
    ::unlink(tmp_path.c_str());
    if (out_error) *out_error = "Failed to write complete content to temp file";
    return false;
  }

  // 2. Stage: FLUSH_FSYNC
  if (fault_stage_ == FaultStage::FLUSH_FSYNC) {
    ::close(fd);
    ::unlink(tmp_path.c_str());
    if (out_error) *out_error = "Injected fault: FLUSH_FSYNC";
    return false;
  }

  if (::fsync(fd) != 0) {
    ::close(fd);
    ::unlink(tmp_path.c_str());
    if (out_error) *out_error = "Failed to fsync temp file";
    return false;
  }
  ::close(fd);

  // 3. Stage: RENAME
  if (fault_stage_ == FaultStage::RENAME) {
    ::unlink(tmp_path.c_str());
    if (out_error) *out_error = "Injected fault: RENAME";
    return false;
  }

  if (::rename(tmp_path.c_str(), tgt.c_str()) != 0) {
    ::unlink(tmp_path.c_str());
    if (out_error) *out_error = "Failed to atomically rename temp file to target";
    return false;
  }

  // 4. Stage: DIRECTORY_SYNC
  if (fault_stage_ == FaultStage::DIRECTORY_SYNC) {
    if (out_error) *out_error = "Injected fault: DIRECTORY_SYNC";
    return false;
  }

  int dir_fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
  if (dir_fd >= 0) {
    ::fsync(dir_fd);
    ::close(dir_fd);
  }

  return true;
}

bool PersistenceManager::commit_snapshot(
  const MissionSnapshotRecord & snapshot,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!validate_storage_paths(out_error)) {
    return false;
  }

  // Stage: VALIDATE
  if (fault_stage_ == FaultStage::VALIDATE) {
    if (out_error) *out_error = "Injected fault: VALIDATE";
    return false;
  }

  auto snap = snapshot;
  if (snap.checksum.empty()) {
    snap.checksum = snap.compute_checksum();
  }

  if (!snap.is_valid()) {
    if (out_error) *out_error = "Invalid snapshot data or checksum mismatch";
    return false;
  }

  std::ostringstream ss;
  ss << "{\n"
     << "  \"schema_version\": \"" << snap.schema_version << "\",\n"
     << "  \"mission_id\": \"" << snap.mission_id << "\",\n"
     << "  \"sortie_id\": \"" << snap.sortie_id << "\",\n"
     << "  \"snapshot_revision\": " << snap.snapshot_revision << ",\n"
     << "  \"durable_sequence\": " << (snap.durable_sequence > 0 ? snap.durable_sequence : durable_sequence_ + 1) << ",\n"
     << "  \"resolved_config_hash\": \"" << snap.resolved_config_hash << "\",\n"
     << "  \"map_id\": \"" << snap.map_id << "\",\n"
     << "  \"scenario_id\": \"" << snap.scenario_id << "\",\n"
     << "  \"plan_artifact_id\": \"" << snap.plan_artifact_id << "\",\n"
     << "  \"working_plan_id\": \"" << snap.working_plan_id << "\",\n"
     << "  \"working_plan_generation\": " << snap.working_plan_generation << ",\n"
     << "  \"target\": {\n"
     << "    \"marker_id\": " << snap.target.marker_id << ",\n"
     << "    \"dictionary\": \"" << snap.target.dictionary << "\",\n"
     << "    \"target_namespace\": \"" << snap.target.target_namespace << "\"\n"
     << "  },\n"
     << "  \"payload_state\": {\n"
     << "    \"commanded_state\": " << static_cast<int>(snap.payload.commanded_state) << ",\n"
     << "    \"cargo_loaded\": " << (snap.payload.cargo_loaded ? "true" : "false") << ",\n"
     << "    \"secured\": " << (snap.payload.secured ? "true" : "false") << ",\n"
     << "    \"successful_operations\": " << snap.payload.successful_operations << ",\n"
     << "    \"last_operation_id\": \"" << snap.payload.last_operation_id << "\",\n"
     << "    \"last_result\": " << static_cast<int>(snap.payload.last_result) << ",\n"
     << "    \"unknown_result\": " << (snap.payload.unknown_result ? "true" : "false") << "\n"
     << "  },\n"
     << "  \"executor_checkpoint\": {\n"
     << "    \"phase\": \"" << snap.executor.phase << "\",\n"
     << "    \"active_action\": \"" << snap.executor.active_action << "\",\n"
     << "    \"durable_sequence\": " << snap.executor.durable_sequence << "\n"
     << "  },\n"
     << "  \"checksum\": \"" << snap.checksum << "\"\n"
     << "}\n";

  std::filesystem::path target_file =
    std::filesystem::path(paths_.state_directory) / "active_snapshot.json";

  if (!atomic_write_file(target_file.string(), ss.str(), out_error)) {
    return false;
  }

  // Stage: JOURNAL
  if (fault_stage_ == FaultStage::JOURNAL) {
    if (out_error) *out_error = "Injected fault: JOURNAL";
    return false;
  }

  // Commit journal entry
  JournalEntry j_entry;
  j_entry.entry_sequence = journal_.size() + 1;
  j_entry.event_id = "EVT_SNAPSHOT_COMMITTED";
  j_entry.idempotency_key = "snap_" + std::to_string(snap.snapshot_revision);
  j_entry.mission_id = snap.mission_id;
  j_entry.sortie_id = snap.sortie_id;
  j_entry.snapshot_hash = snap.checksum;
  j_entry.severity = 0; // INFO
  j_entry.source = 7;   // SOURCE_PERSISTENCE
  j_entry.component = "PersistenceManager";
  j_entry.detail = "Snapshot committed revision " + std::to_string(snap.snapshot_revision);
  j_entry.timestamp_monotonic_ns = 0;
  j_entry.checksum = j_entry.compute_checksum();

  if (!paths_.evidence_directory.empty()) {
    std::filesystem::path journal_file =
      std::filesystem::path(paths_.evidence_directory) / "mission_journal.jsonl";
    std::ostringstream j_ss;
    j_ss << "{\"seq\":" << j_entry.entry_sequence
         << ",\"event_id\":\"" << j_entry.event_id << "\""
         << ",\"checksum\":\"" << j_entry.checksum << "\"}\n";
    std::ofstream j_out(journal_file.string(), std::ios::app);
    if (j_out) {
      j_out << j_ss.str();
      j_out.flush();
    }
  }

  // Stage: BACKUP
  if (fault_stage_ == FaultStage::BACKUP) {
    if (out_error) *out_error = "Injected fault: BACKUP";
    return false;
  }

  // Success
  durable_sequence_ = (snap.durable_sequence > 0 ? snap.durable_sequence : durable_sequence_ + 1);
  snap.durable_sequence = durable_sequence_;
  active_snapshot_ = snap;
  journal_.push_back(j_entry);

  recovery_status_.durable_snapshot_sequence = durable_sequence_;
  recovery_status_.has_last_valid_snapshot_hash = true;
  recovery_status_.last_valid_snapshot_hash = snap.checksum;

  return true;
}

bool PersistenceManager::append_journal_entry(
  const JournalEntry & entry,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!validate_storage_paths(out_error)) {
    return false;
  }

  if (fault_stage_ == FaultStage::JOURNAL) {
    if (out_error) *out_error = "Injected fault: JOURNAL";
    return false;
  }

  auto ent = entry;
  ent.entry_sequence = journal_.size() + 1;
  if (ent.checksum.empty()) {
    ent.checksum = ent.compute_checksum();
  }

  if (!paths_.evidence_directory.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(paths_.evidence_directory, ec);
    std::filesystem::path journal_file =
      std::filesystem::path(paths_.evidence_directory) / "mission_journal.jsonl";
    std::ofstream j_out(journal_file.string(), std::ios::app);
    if (!j_out) {
      if (out_error) *out_error = "Failed to open journal file for writing";
      return false;
    }
    j_out << "{\"seq\":" << ent.entry_sequence
          << ",\"event_id\":\"" << ent.event_id << "\""
          << ",\"detail\":\"" << ent.detail << "\""
          << ",\"checksum\":\"" << ent.checksum << "\"}\n";
    j_out.flush();
  }

  journal_.push_back(ent);
  return true;
}

bool PersistenceManager::create_backup(
  const std::string & source_type,
  const std::string & source_identifier,
  const std::string & content,
  uint64_t revision,
  std::string * out_backup_id,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!validate_storage_paths(out_error)) {
    return false;
  }

  if (fault_stage_ == FaultStage::BACKUP) {
    if (out_error) *out_error = "Injected fault: BACKUP";
    return false;
  }

  std::string b_id = "bak_" + source_type + "_" + std::to_string(revision) + "_" +
    std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

  std::filesystem::path backup_file =
    std::filesystem::path(paths_.backup_directory) / (b_id + ".bak");

  if (!atomic_write_file(backup_file.string(), content, out_error)) {
    return false;
  }

  BackupRecord rec;
  rec.backup_id = b_id;
  rec.source_type = source_type;
  rec.source_identifier = source_identifier;
  rec.revision = revision;
  rec.checksum = compute_sha256(content);
  rec.created_at_utc = "2026-08-17T00:00:00Z";

  backups_.push_back(rec);
  if (out_backup_id) {
    *out_backup_id = b_id;
  }
  return true;
}

std::optional<MissionSnapshotRecord> PersistenceManager::load_active_snapshot(
  std::string * out_error) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return load_active_snapshot_unlocked(out_error);
}

std::optional<MissionSnapshotRecord> PersistenceManager::load_active_snapshot_unlocked(
  std::string * out_error) const
{
  if (paths_.state_directory.empty()) {
    if (active_snapshot_.has_value()) return active_snapshot_;
    if (out_error) *out_error = "State directory not configured";
    return std::nullopt;
  }

  std::filesystem::path target =
    std::filesystem::path(paths_.state_directory) / "active_snapshot.json";

  if (!std::filesystem::is_regular_file(target)) {
    if (out_error) *out_error = "Active snapshot file not found: " + target.string();
    return std::nullopt;
  }

  std::ifstream in(target.string());
  if (!in) {
    if (out_error) *out_error = "Failed to open active snapshot file";
    return std::nullopt;
  }

  std::stringstream ss;
  ss << in.rdbuf();
  std::string content = ss.str();

  // Parse fields manually from formatted JSON
  MissionSnapshotRecord snap;
  auto find_str_after = [&](const std::string & key, size_t start_pos = 0) -> std::string {
    auto pos = content.find("\"" + key + "\"", start_pos);
    if (pos == std::string::npos) return "";
    auto colon = content.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = content.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = content.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return content.substr(q1 + 1, q2 - q1 - 1);
  };

  auto find_num_after = [&](const std::string & key, size_t start_pos = 0) -> uint64_t {
    auto pos = content.find("\"" + key + "\"", start_pos);
    if (pos == std::string::npos) return 0;
    auto colon = content.find(':', pos);
    if (colon == std::string::npos) return 0;
    std::string tail = content.substr(colon + 1);
    try {
      return std::stoull(tail);
    } catch (...) {
      return 0;
    }
  };

  auto find_bool_after = [&](const std::string & key, size_t start_pos = 0) -> bool {
    auto pos = content.find("\"" + key + "\"", start_pos);
    if (pos == std::string::npos) return false;
    auto colon = content.find(':', pos);
    if (colon == std::string::npos) return false;
    std::string tail = content.substr(colon + 1, 10);
    return tail.find("true") != std::string::npos;
  };

  size_t target_block = content.find("\"target\"");
  size_t payload_block = content.find("\"payload_state\"");
  size_t executor_block = content.find("\"executor_checkpoint\"");

  snap.schema_version = find_str_after("schema_version");
  snap.mission_id = find_str_after("mission_id");
  snap.sortie_id = find_str_after("sortie_id");
  snap.snapshot_revision = find_num_after("snapshot_revision");
  snap.durable_sequence = find_num_after("durable_sequence");
  snap.resolved_config_hash = find_str_after("resolved_config_hash");
  snap.map_id = find_str_after("map_id");
  snap.scenario_id = find_str_after("scenario_id");
  snap.plan_artifact_id = find_str_after("plan_artifact_id");
  snap.working_plan_id = find_str_after("working_plan_id");
  snap.working_plan_generation = find_num_after("working_plan_generation");

  if (target_block != std::string::npos) {
    snap.target.marker_id = static_cast<uint32_t>(find_num_after("marker_id", target_block));
    snap.target.dictionary = find_str_after("dictionary", target_block);
    snap.target.target_namespace = find_str_after("target_namespace", target_block);
  }

  if (payload_block != std::string::npos) {
    snap.payload.commanded_state = static_cast<uint8_t>(find_num_after("commanded_state", payload_block));
    snap.payload.cargo_loaded = find_bool_after("cargo_loaded", payload_block);
    snap.payload.secured = find_bool_after("secured", payload_block);
    snap.payload.successful_operations = static_cast<uint32_t>(find_num_after("successful_operations", payload_block));
    snap.payload.last_operation_id = find_str_after("last_operation_id", payload_block);
    snap.payload.last_result = static_cast<uint8_t>(find_num_after("last_result", payload_block));
    snap.payload.unknown_result = find_bool_after("unknown_result", payload_block);
  }

  if (executor_block != std::string::npos) {
    snap.executor.phase = find_str_after("phase", executor_block);
    snap.executor.active_action = find_str_after("active_action", executor_block);
    snap.executor.durable_sequence = find_num_after("durable_sequence", executor_block);
  }

  snap.checksum = find_str_after("checksum");

  if (!snap.is_valid()) {
    if (out_error) *out_error = "Snapshot on disk failed integrity or checksum check";
    return std::nullopt;
  }

  return snap;
}

std::vector<JournalEntry> PersistenceManager::load_journal_entries(
  std::string * out_error) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (paths_.evidence_directory.empty() || !journal_.empty()) {
    return journal_;
  }

  std::vector<JournalEntry> entries;
  std::filesystem::path journal_file =
    std::filesystem::path(paths_.evidence_directory) / "mission_journal.jsonl";

  if (!std::filesystem::is_regular_file(journal_file)) {
    return entries;
  }

  std::ifstream in(journal_file.string());
  if (!in) {
    if (out_error) *out_error = "Failed to open journal file";
    return entries;
  }

  std::string line;
  uint64_t expected_seq = 1;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    JournalEntry ent;
    ent.entry_sequence = expected_seq++;
    ent.event_id = "EVT_RESTORED";
    ent.checksum = ent.compute_checksum();
    entries.push_back(ent);
  }

  return entries;
}

std::vector<BackupRecord> PersistenceManager::list_backups() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return backups_;
}

RecoveryResult PersistenceManager::recover(
  const std::string & current_resolved_config_hash,
  bool has_working_plan_check,
  bool is_working_plan_valid)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RecoveryResult res;
  res.status = recovery_status_;
  res.status.ambiguity_codes.clear();

  std::string err;
  auto snapshot_opt = load_active_snapshot_unlocked(&err);

  // 1. Check snapshot ambiguity
  std::filesystem::path snapshot_path =
    std::filesystem::path(paths_.state_directory) / "active_snapshot.json";
  if (std::filesystem::exists(snapshot_path) && !snapshot_opt.has_value()) {
    res.ambiguity_codes.push_back(full_self_driving::msg::RecoveryStatus::AMBIGUOUS_SNAPSHOT);
  }

  if (snapshot_opt.has_value()) {
    const auto & snap = *snapshot_opt;
    active_snapshot_ = snap;
    durable_sequence_ = snap.durable_sequence;
    res.status.durable_snapshot_sequence = durable_sequence_;
    res.status.has_last_valid_snapshot_hash = true;
    res.status.last_valid_snapshot_hash = snap.checksum;

    // 2. Check config hash compatibility
    if (!current_resolved_config_hash.empty() &&
        snap.resolved_config_hash != current_resolved_config_hash)
    {
      res.ambiguity_codes.push_back(full_self_driving::msg::RecoveryStatus::AMBIGUOUS_CONFIG_HASH);
    }

    // 3. Check working plan ambiguity
    if (has_working_plan_check && !snap.working_plan_id.empty() && !is_working_plan_valid) {
      res.ambiguity_codes.push_back(full_self_driving::msg::RecoveryStatus::AMBIGUOUS_WORKING_PLAN);
    }

    // 4. Check payload ambiguity
    if (snap.payload.unknown_result) {
      res.ambiguity_codes.push_back(full_self_driving::msg::RecoveryStatus::AMBIGUOUS_PAYLOAD);
    }

    // 5. Check executor ambiguity
    if (snap.executor.phase == "PRECISION_DESCEND" || snap.executor.phase == "PAYLOAD_OPERATION") {
      res.ambiguity_codes.push_back(full_self_driving::msg::RecoveryStatus::AMBIGUOUS_EXECUTOR);
    }

    // 6. Check registry / scope ambiguity
    if (snap.map_id.empty() || snap.scenario_id.empty()) {
      res.ambiguity_codes.push_back(full_self_driving::msg::RecoveryStatus::AMBIGUOUS_REGISTRY);
    }
  }

  if (!res.ambiguity_codes.empty()) {
    res.is_clear = false;
    recovery_status_.state = full_self_driving::msg::RecoveryStatus::STATE_REQUIRED;
    recovery_status_.safe_decision_required = true;
    recovery_status_.ambiguity_codes.clear();
    for (auto code : res.ambiguity_codes) {
      recovery_status_.ambiguity_codes.push_back(code);
    }
    res.status = recovery_status_;
  } else {
    res.is_clear = true;
    recovery_status_.state = full_self_driving::msg::RecoveryStatus::STATE_CLEAR;
    recovery_status_.safe_decision_required = false;
    recovery_status_.ambiguity_codes.clear();
    res.status = recovery_status_;
  }

  return res;
}

bool PersistenceManager::resolve_recovery(
  uint8_t decision,
  uint64_t expected_recovery_revision,
  const std::string & confirmation,
  bool is_disarmed,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_disarmed) {
    if (out_error) *out_error = "Vehicle must be disarmed to resolve recovery";
    return false;
  }

  if (expected_recovery_revision != recovery_revision_) {
    if (out_error) {
      *out_error = "Recovery revision mismatch: expected " +
                   std::to_string(expected_recovery_revision) + ", actual " +
                   std::to_string(recovery_revision_);
    }
    return false;
  }

  if (confirmation.empty()) {
    if (out_error) *out_error = "Confirmation token cannot be empty";
    return false;
  }

  if (decision == full_self_driving::msg::RecoveryStatus::DECISION_UNKNOWN) {
    if (out_error) *out_error = "Unknown decision is not allowed";
    return false;
  }

  recovery_revision_++;
  recovery_status_.state = full_self_driving::msg::RecoveryStatus::STATE_RESOLVED;
  recovery_status_.safe_decision_required = false;
  recovery_status_.has_decision = true;
  recovery_status_.decision = decision;
  recovery_status_.decision_revision = recovery_revision_;
  recovery_status_.ambiguity_codes.clear();

  // Commit recovery journal event
  JournalEntry j_entry;
  j_entry.entry_sequence = journal_.size() + 1;
  j_entry.event_id = "EVT_RECOVERY_RESOLVED";
  j_entry.idempotency_key = "rec_" + std::to_string(recovery_revision_);
  j_entry.detail = "Recovery resolved with decision " + std::to_string(decision);
  j_entry.checksum = j_entry.compute_checksum();
  journal_.push_back(j_entry);

  return true;
}

}  // namespace full_self_driving::persistence
