#include "domain/plan_parser.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <openssl/evp.h>

namespace full_self_driving::domain
{

namespace
{

// Bounded internal JSON DOM for parsing QGC plans
class JsonNode
{
public:
  enum class Type
  {
    Null,
    Bool,
    Number,
    String,
    Array,
    Object
  };

  JsonNode() = default;
  JsonNode(Type type, double num, std::string str, bool boolean)
  : type_(type), num_(num), str_(std::move(str)), bool_(boolean) {}

  static JsonNode parse(const std::string & text, size_t max_depth = PlanParser::kMaxNestingDepth);

  Type type() const { return type_; }
  bool is_object() const { return type_ == Type::Object; }
  bool is_array() const { return type_ == Type::Array; }
  bool is_number() const { return type_ == Type::Number; }
  bool is_string() const { return type_ == Type::String; }
  bool is_bool() const { return type_ == Type::Bool; }
  bool is_null() const { return type_ == Type::Null; }

  bool has(const std::string & key) const
  {
    if (!is_object()) {
      return false;
    }
    for (const auto & kv : object_) {
      if (kv.first == key) {
        return true;
      }
    }
    return false;
  }

  const JsonNode & at(const std::string & key) const
  {
    if (!is_object()) {
      throw std::runtime_error("JsonNode: 'at(key)' called on non-object");
    }
    for (const auto & kv : object_) {
      if (kv.first == key) {
        return kv.second;
      }
    }
    throw std::runtime_error("JsonNode: key not found: " + key);
  }

  size_t size() const
  {
    if (is_array()) {
      return array_.size();
    }
    if (is_object()) {
      return object_.size();
    }
    return 0;
  }

  const JsonNode & at(size_t index) const
  {
    if (!is_array()) {
      throw std::runtime_error("JsonNode: 'at(index)' called on non-array");
    }
    if (index >= array_.size()) {
      throw std::runtime_error("JsonNode: array index out of range");
    }
    return array_[index];
  }

  const std::vector<JsonNode> & array_items() const { return array_; }
  const std::vector<std::pair<std::string, JsonNode>> & object_entries() const { return object_; }

  double as_number() const { return num_; }
  const std::string & as_string() const { return str_; }
  bool as_bool() const { return bool_; }

private:
  static JsonNode parse_value(const std::string & text, size_t & pos, size_t depth, size_t max_depth);
  static void skip_whitespace(const std::string & text, size_t & pos);
  static void expect_literal(const std::string & text, size_t & pos, const char * literal);
  static std::string parse_string(const std::string & text, size_t & pos);
  static JsonNode parse_object(const std::string & text, size_t & pos, size_t depth, size_t max_depth);
  static JsonNode parse_array(const std::string & text, size_t & pos, size_t depth, size_t max_depth);
  static double parse_number(const std::string & text, size_t & pos);

  Type type_{Type::Null};
  double num_{0.0};
  std::string str_;
  bool bool_{false};
  std::vector<JsonNode> array_;
  std::vector<std::pair<std::string, JsonNode>> object_;
};

void JsonNode::skip_whitespace(const std::string & text, size_t & pos)
{
  while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
    ++pos;
  }
}

void JsonNode::expect_literal(const std::string & text, size_t & pos, const char * literal)
{
  for (const char * p = literal; *p != '\0'; ++p) {
    if (pos >= text.size() || text[pos] != *p) {
      throw std::runtime_error(std::string("JsonNode: expected literal '") + literal + "'");
    }
    ++pos;
  }
}

std::string JsonNode::parse_string(const std::string & text, size_t & pos)
{
  if (pos >= text.size() || text[pos] != '"') {
    throw std::runtime_error("JsonNode: expected string");
  }
  ++pos;

  std::string result;
  while (pos < text.size()) {
    const char c = text[pos];
    if (c == '"') {
      ++pos;
      return result;
    }
    if (c == '\\') {
      ++pos;
      if (pos >= text.size()) {
        throw std::runtime_error("JsonNode: bad escape sequence");
      }
      const char esc = text[pos];
      switch (esc) {
        case '"': result += '"'; break;
        case '\\': result += '\\'; break;
        case '/': result += '/'; break;
        case 'b': result += '\b'; break;
        case 'f': result += '\f'; break;
        case 'n': result += '\n'; break;
        case 'r': result += '\r'; break;
        case 't': result += '\t'; break;
        case 'u': {
          // Parse 4 hex digits
          if (pos + 4 >= text.size()) {
            throw std::runtime_error("JsonNode: incomplete unicode escape");
          }
          pos += 4;
          result += '?';
          break;
        }
        default:
          throw std::runtime_error("JsonNode: unsupported escape sequence");
      }
      ++pos;
    } else {
      result += c;
      ++pos;
    }
  }
  throw std::runtime_error("JsonNode: unterminated string");
}

JsonNode JsonNode::parse_object(const std::string & text, size_t & pos, size_t depth, size_t max_depth)
{
  if (depth > max_depth) {
    throw std::runtime_error("JsonNode: maximum nesting depth exceeded");
  }

  JsonNode obj;
  obj.type_ = Type::Object;

  ++pos;
  skip_whitespace(text, pos);

  if (pos < text.size() && text[pos] == '}') {
    ++pos;
    return obj;
  }

  while (true) {
    skip_whitespace(text, pos);
    if (pos >= text.size() || text[pos] != '"') {
      throw std::runtime_error("JsonNode: expected object key");
    }
    std::string key = parse_string(text, pos);

    skip_whitespace(text, pos);
    if (pos >= text.size() || text[pos] != ':') {
      throw std::runtime_error("JsonNode: expected ':' after object key");
    }
    ++pos;

    JsonNode value = parse_value(text, pos, depth + 1, max_depth);
    obj.object_.emplace_back(std::move(key), std::move(value));

    skip_whitespace(text, pos);
    if (pos < text.size() && text[pos] == ',') {
      ++pos;
      continue;
    }
    if (pos < text.size() && text[pos] == '}') {
      ++pos;
      return obj;
    }
    throw std::runtime_error("JsonNode: expected ',' or '}' in object");
  }
}

JsonNode JsonNode::parse_array(const std::string & text, size_t & pos, size_t depth, size_t max_depth)
{
  if (depth > max_depth) {
    throw std::runtime_error("JsonNode: maximum nesting depth exceeded");
  }

  JsonNode arr;
  arr.type_ = Type::Array;

  ++pos;
  skip_whitespace(text, pos);

  if (pos < text.size() && text[pos] == ']') {
    ++pos;
    return arr;
  }

  while (true) {
    JsonNode value = parse_value(text, pos, depth + 1, max_depth);
    arr.array_.push_back(std::move(value));

    skip_whitespace(text, pos);
    if (pos < text.size() && text[pos] == ',') {
      ++pos;
      continue;
    }
    if (pos < text.size() && text[pos] == ']') {
      ++pos;
      return arr;
    }
    throw std::runtime_error("JsonNode: expected ',' or ']' in array");
  }
}

double JsonNode::parse_number(const std::string & text, size_t & pos)
{
  const size_t start = pos;
  while (pos < text.size()) {
    const char c = text[pos];
    if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.' ||
        c == 'e' || c == 'E')
    {
      ++pos;
    } else {
      break;
    }
  }
  if (pos == start) {
    throw std::runtime_error("JsonNode: expected number");
  }
  char * end_ptr = nullptr;
  const double value = std::strtod(text.c_str() + start, &end_ptr);
  if (end_ptr == text.c_str() + start || !std::isfinite(value)) {
    throw std::runtime_error("JsonNode: invalid or non-finite number");
  }
  return value;
}

JsonNode JsonNode::parse_value(const std::string & text, size_t & pos, size_t depth, size_t max_depth)
{
  skip_whitespace(text, pos);
  if (pos >= text.size()) {
    throw std::runtime_error("JsonNode: unexpected end of input");
  }

  switch (text[pos]) {
    case '{': return parse_object(text, pos, depth, max_depth);
    case '[': return parse_array(text, pos, depth, max_depth);
    case '"': return JsonNode(Type::String, 0.0, parse_string(text, pos), false);
    case 't':
      expect_literal(text, pos, "true");
      return JsonNode(Type::Bool, 0.0, "", true);
    case 'f':
      expect_literal(text, pos, "false");
      return JsonNode(Type::Bool, 0.0, "", false);
    case 'n':
      expect_literal(text, pos, "null");
      return JsonNode(Type::Null, 0.0, "", false);
    default:
      if (text[pos] == '-' || std::isdigit(static_cast<unsigned char>(text[pos]))) {
        return JsonNode(Type::Number, parse_number(text, pos), "", false);
      }
      throw std::runtime_error(std::string("JsonNode: unexpected character '") + text[pos] + "'");
  }
}

JsonNode JsonNode::parse(const std::string & text, size_t max_depth)
{
  size_t pos = 0;
  JsonNode value = parse_value(text, pos, 0, max_depth);
  skip_whitespace(text, pos);
  if (pos != text.size()) {
    throw std::runtime_error("JsonNode: trailing characters after document");
  }
  return value;
}

double require_finite_number(const JsonNode & value, const std::string & field)
{
  if (!value.is_number() || !std::isfinite(value.as_number())) {
    throw std::runtime_error("PlanParser: '" + field + "' must be a finite number");
  }
  return value.as_number();
}

float extract_altitude(const JsonNode & node, float fallback)
{
  if (node.is_object()) {
    if (node.has("CameraCalc")) {
      const JsonNode & camera = node.at("CameraCalc");
      if (camera.is_object() && camera.has("DistanceToSurface")) {
        return static_cast<float>(require_finite_number(
          camera.at("DistanceToSurface"), "CameraCalc.DistanceToSurface"));
      }
    }
    for (const auto & kv : node.object_entries()) {
      const float found = extract_altitude(kv.second, fallback);
      if (found != fallback) {
        return found;
      }
    }
  } else if (node.is_array()) {
    for (const auto & child : node.array_items()) {
      const float found = extract_altitude(child, fallback);
      if (found != fallback) {
        return found;
      }
    }
  }
  return fallback;
}

void collect_waypoints(
  const JsonNode & node,
  std::vector<SearchWaypoint> & waypoints,
  float default_alt,
  uint32_t & current_source_index)
{
  if (node.is_array()) {
    for (const auto & child : node.array_items()) {
      collect_waypoints(child, waypoints, default_alt, current_source_index);
    }
    return;
  }

  if (!node.is_object()) {
    return;
  }

  if (node.has("command") && node.has("params")) {
    const JsonNode & cmd_node = node.at("command");
    const JsonNode & params = node.at("params");

    if (cmd_node.is_number() && cmd_node.as_number() == 16.0 && params.is_array() && params.size() >= 7) {
      const double lat = require_finite_number(params.at(4), "waypoint latitude");
      const double lon = require_finite_number(params.at(5), "waypoint longitude");
      double alt = default_alt;
      if (!params.at(6).is_null() && params.at(6).is_number()) {
        double parsed_alt = params.at(6).as_number();
        if (std::isfinite(parsed_alt) && parsed_alt > 0.0) {
          alt = parsed_alt;
        }
      }

      if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        throw std::runtime_error("PlanParser: waypoint latitude/longitude out of range [-90,90], [-180,180]");
      }

      SearchWaypoint wp;
      wp.latitude_deg = lat;
      wp.longitude_deg = lon;
      wp.altitude_m = alt;
      wp.source_index = current_source_index++;
      waypoints.push_back(wp);
      return;
    }
  }

  // Walk child entries
  for (const auto & kv : node.object_entries()) {
    if (kv.first == "params" && kv.second.is_array()) {
      continue;
    }
    collect_waypoints(kv.second, waypoints, default_alt, current_source_index);
  }
}

bool is_transect_or_complex_item(const JsonNode & item)
{
  if (!item.is_object()) return false;
  if (item.has("TransectStyleComplexItem") || item.has("complexItem")) return true;
  if (item.has("type") && item.at("type").is_string() && item.at("type").as_string() == "ComplexItem") return true;
  return false;
}

std::optional<RoutePoint> extract_simple_waypoint(const JsonNode & item, float default_alt)
{
  if (!item.is_object()) return std::nullopt;
  if (item.has("command") && item.has("params")) {
    const JsonNode & cmd_node = item.at("command");
    const JsonNode & params = item.at("params");

    if (cmd_node.is_number() && cmd_node.as_number() == 16.0 && params.is_array() && params.size() >= 7) {
      const double lat = require_finite_number(params.at(4), "waypoint latitude");
      const double lon = require_finite_number(params.at(5), "waypoint longitude");
      double alt = default_alt;
      if (!params.at(6).is_null() && params.at(6).is_number()) {
        double parsed_alt = params.at(6).as_number();
        if (std::isfinite(parsed_alt) && parsed_alt > 0.0) {
          alt = parsed_alt;
        }
      }

      if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        throw std::runtime_error("PlanParser: waypoint latitude/longitude out of range [-90,90], [-180,180]");
      }

      return RoutePoint(lat, lon, alt);
    }
  }
  return std::nullopt;
}

}  // namespace

bool PlanParser::is_safe_basename(const std::string & name)
{
  if (name.empty() || name.size() > 128) {
    return false;
  }
  if (name == "." || name == "..") {
    return false;
  }
  if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos) {
    return false;
  }
  if (name.front() == '.') {
    return false;
  }
  const std::filesystem::path path(name);
  if (path.filename().string() != name || path.extension() != ".plan") {
    return false;
  }

  for (char c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {
      return false;
    }
  }
  return true;
}

std::string PlanParser::compute_sha256(const std::vector<uint8_t> & bytes)
{
  if (bytes.empty()) {
    return "";
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  size_t digest_len = 0;

  if (EVP_Q_digest(
      nullptr, "SHA256", nullptr,
      bytes.data(), bytes.size(),
      digest, &digest_len) != 1)
  {
    throw std::runtime_error("PlanParser: EVP_Q_digest failed");
  }

  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (size_t i = 0; i < digest_len; ++i) {
    ss << std::setw(2) << static_cast<int>(digest[i]);
  }
  return ss.str();
}

std::string PlanParser::compute_sha256(const std::string & str)
{
  const std::vector<uint8_t> bytes(str.begin(), str.end());
  return compute_sha256(bytes);
}

std::string PlanParser::compute_canonical_route_hash(const CanonicalSearchRoute & route)
{
  std::ostringstream ss;
  ss << std::setprecision(17);
  ss << "alt:" << route.default_altitude_m << ";speed:" << route.cruise_speed_m_s << ";waypoints:" << route.waypoints.size() << ";";
  for (const auto & wp : route.waypoints) {
    ss << "[" << wp.latitude_deg << "," << wp.longitude_deg << "," << wp.altitude_m << "," << wp.source_index << "];";
  }
  return compute_sha256(ss.str());
}

PlanParseResult PlanParser::parse_bytes(
  const std::vector<uint8_t> & bytes,
  const std::string & safe_name,
  size_t max_bytes)
{
  PlanParseResult result;
  result.safe_name = safe_name;
  result.byte_length = bytes.size();

  if (bytes.empty()) {
    result.error_code = "EMPTY_PLAN_CONTENT";
    result.error_message = "Plan content bytes cannot be empty";
    return result;
  }

  if (bytes.size() > max_bytes) {
    result.error_code = "PLAN_OVERSIZED";
    result.error_message = "Plan content exceeds maximum size of " + std::to_string(max_bytes) + " bytes";
    return result;
  }

  if (!is_safe_basename(safe_name)) {
    result.error_code = "INVALID_SAFE_NAME";
    result.error_message = "Plan safe_name must be a valid .plan basename without paths";
    return result;
  }

  result.raw_content_sha256 = compute_sha256(bytes);

  std::string json_text(bytes.begin(), bytes.end());
  return parse_string(json_text, safe_name, max_bytes);
}

PlanParseResult PlanParser::parse_string(
  const std::string & json_text,
  const std::string & safe_name,
  size_t max_bytes)
{
  PlanParseResult result;
  result.safe_name = safe_name;
  result.byte_length = json_text.size();
  result.raw_content_sha256 = compute_sha256(json_text);

  std::string map_name = safe_name;
  if (map_name.size() > 5 && map_name.substr(map_name.size() - 5) == ".plan") {
    map_name = map_name.substr(0, map_name.size() - 5);
  }
  result.map_name = map_name;

  if (json_text.empty()) {
    result.error_code = "EMPTY_PLAN_CONTENT";
    result.error_message = "Plan JSON string cannot be empty";
    return result;
  }

  if (json_text.size() > max_bytes) {
    result.error_code = "PLAN_OVERSIZED";
    result.error_message = "Plan JSON text exceeds maximum size of " + std::to_string(max_bytes) + " bytes";
    return result;
  }

  if (!is_safe_basename(safe_name)) {
    result.error_code = "INVALID_SAFE_NAME";
    result.error_message = "Plan safe_name must be a valid .plan basename without paths: " + safe_name;
    return result;
  }

  JsonNode root;
  try {
    root = JsonNode::parse(json_text, kMaxNestingDepth);
  } catch (const std::exception & e) {
    result.error_code = "JSON_PARSE_ERROR";
    result.error_message = std::string("Malformed JSON: ") + e.what();
    return result;
  }

  if (!root.is_object() || !root.has("mission")) {
    result.error_code = "MISSING_MISSION_OBJECT";
    result.error_message = "Missing top-level 'mission' object in plan JSON";
    return result;
  }

  const JsonNode & mission = root.at("mission");
  if (!mission.is_object()) {
    result.error_code = "INVALID_MISSION_OBJECT";
    result.error_message = "'mission' field must be an object";
    return result;
  }

  CanonicalSearchRoute route;
  if (mission.has("cruiseSpeed")) {
    try {
      route.cruise_speed_m_s = static_cast<float>(require_finite_number(
        mission.at("cruiseSpeed"), "mission.cruiseSpeed"));
    } catch (const std::exception & e) {
      result.error_code = "INVALID_CRUISE_SPEED";
      result.error_message = e.what();
      return result;
    }
  }

  route.default_altitude_m = extract_altitude(mission, route.default_altitude_m);

  // Check search planner metadata
  if (mission.has("searchPlanner")) {
    result.has_search_planner_metadata = true;
    const JsonNode & metadata = mission.at("searchPlanner");
    if (metadata.is_object()) {
      if (metadata.has("entryPoint")) {
        const JsonNode & entry = metadata.at("entryPoint");
        if (entry.is_array() && entry.size() >= 2) {
          try {
            double lat = require_finite_number(entry.at(0), "searchPlanner.entryPoint lat");
            double lon = require_finite_number(entry.at(1), "searchPlanner.entryPoint lon");
            if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
              result.entry_point = std::array<double, 2>{lat, lon};
            }
          } catch (...) {}
        }
      }
      if (metadata.has("nextWaypointIndex") && metadata.at("nextWaypointIndex").is_number()) {
        double idx = metadata.at("nextWaypointIndex").as_number();
        if (std::isfinite(idx) && idx >= 0.0) {
          result.next_waypoint_index = static_cast<size_t>(idx);
        }
      }
    }
  }

  if (mission.has("items") && mission.at("items").is_array()) {
    const auto & items = mission.at("items").array_items();

    bool has_complex_survey = false;
    for (const auto & item : items) {
      if (is_transect_or_complex_item(item)) {
        has_complex_survey = true;
        break;
      }
    }

    uint32_t current_source_idx = 0;
    try {
      if (has_complex_survey) {
        bool in_or_past_survey = false;
        bool past_survey = false;
        for (const auto & item : items) {
          if (is_transect_or_complex_item(item)) {
            in_or_past_survey = true;
            collect_waypoints(item, route.waypoints, route.default_altitude_m, current_source_idx);
            past_survey = true;
          } else if (auto simple_wp = extract_simple_waypoint(item, route.default_altitude_m)) {
            if (!in_or_past_survey) {
              result.transit_in_waypoints.push_back(*simple_wp);
            } else if (past_survey) {
              result.transit_out_waypoints.push_back(*simple_wp);
            }
          }
        }
      } else {
        // Flat legacy collection if no complex survey item is present
        collect_waypoints(mission.at("items"), route.waypoints, route.default_altitude_m, current_source_idx);
      }
    } catch (const std::exception & e) {
      result.error_code = "WAYPOINT_EXTRACTION_ERROR";
      result.error_message = e.what();
      return result;
    }
  }

  if (route.waypoints.empty()) {
    result.error_code = "NO_SEARCH_WAYPOINTS";
    result.error_message = "No valid command 16 waypoints found in mission plan";
    return result;
  }

  if (route.waypoints.size() > kMaxWaypoints) {
    result.error_code = "TOO_MANY_WAYPOINTS";
    result.error_message = "Waypoints count exceeds maximum allowed (" + std::to_string(kMaxWaypoints) + ")";
    return result;
  }

  route.canonical_route_sha256 = compute_canonical_route_hash(route);

  result.is_valid = true;
  result.route = std::move(route);
  return result;
}

PlanParseResult PlanParser::parse_file(const std::string & file_path, size_t max_bytes)
{
  PlanParseResult result;
  std::ifstream file(file_path, std::ios::binary);
  if (!file) {
    result.error_code = "FILE_NOT_FOUND";
    result.error_message = "Cannot open plan file: " + file_path;
    return result;
  }

  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::string filename = std::filesystem::path(file_path).filename().string();
  return parse_bytes(bytes, filename, max_bytes);
}

}  // namespace full_self_driving::domain
