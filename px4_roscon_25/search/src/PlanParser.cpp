#include "search/PlanParser.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace search {

void Json::skipWhitespace(const std::string & text, size_t & pos)
{
	while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
		++pos;
	}
}

void Json::expectLiteral(const std::string & text, size_t & pos, const char * literal)
{
	for (const char * p = literal; *p != '\0'; ++p) {
		if (pos >= text.size() || text[pos] != *p) {
			throw std::runtime_error(std::string("Json: expected literal '") + literal + "'");
		}
		++pos;
	}
}

std::string Json::parseString(const std::string & text, size_t & pos)
{
	if (pos >= text.size() || text[pos] != '"') {
		throw std::runtime_error("Json: expected string");
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
				throw std::runtime_error("Json: bad escape sequence");
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
			default:
				throw std::runtime_error("Json: unsupported escape sequence");
			}
			++pos;
		} else {
			result += c;
			++pos;
		}
	}
	throw std::runtime_error("Json: unterminated string");
}

Json Json::parseObject(const std::string & text, size_t & pos)
{
	Json obj;
	obj._type = Json::Type::Object;

	++pos;
	skipWhitespace(text, pos);

	if (pos < text.size() && text[pos] == '}') {
		++pos;
		return obj;
	}

	while (true) {
		skipWhitespace(text, pos);
		if (pos >= text.size() || text[pos] != '"') {
			throw std::runtime_error("Json: expected object key");
		}
		std::string key = parseString(text, pos);

		skipWhitespace(text, pos);
		if (pos >= text.size() || text[pos] != ':') {
			throw std::runtime_error("Json: expected ':' after object key");
		}
		++pos;

		Json value = parseValue(text, pos);
		obj._object.emplace_back(std::move(key), std::move(value));

		skipWhitespace(text, pos);
		if (pos < text.size() && text[pos] == ',') {
			++pos;
			continue;
		}
		if (pos < text.size() && text[pos] == '}') {
			++pos;
			return obj;
		}
		throw std::runtime_error("Json: expected ',' or '}' in object");
	}
}

Json Json::parseArray(const std::string & text, size_t & pos)
{
	Json arr;
	arr._type = Json::Type::Array;

	++pos;
	skipWhitespace(text, pos);

	if (pos < text.size() && text[pos] == ']') {
		++pos;
		return arr;
	}

	while (true) {
		Json value = parseValue(text, pos);
		arr._array.push_back(std::move(value));

		skipWhitespace(text, pos);
		if (pos < text.size() && text[pos] == ',') {
			++pos;
			continue;
		}
		if (pos < text.size() && text[pos] == ']') {
			++pos;
			return arr;
		}
		throw std::runtime_error("Json: expected ',' or ']' in array");
	}
}

double Json::parseNumber(const std::string & text, size_t & pos)
{
	const size_t start = pos;
	while (pos < text.size()) {
		const char c = text[pos];
		if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+' || c == '.' ||
		    c == 'e' || c == 'E') {
			++pos;
		} else {
			break;
		}
	}
	if (pos == start) {
		throw std::runtime_error("Json: expected number");
	}
	char * end_ptr = nullptr;
	const double value = std::strtod(text.c_str() + start, &end_ptr);
	if (end_ptr == text.c_str() + start) {
		throw std::runtime_error("Json: invalid number");
	}
	return value;
}

Json Json::parseValue(const std::string & text, size_t & pos)
{
	skipWhitespace(text, pos);
	if (pos >= text.size()) {
		throw std::runtime_error("Json: unexpected end of input");
	}

	switch (text[pos]) {
	case '{': return parseObject(text, pos);
	case '[': return parseArray(text, pos);
	case '"': return makeString(parseString(text, pos));
	case 't':
		expectLiteral(text, pos, "true");
		return makeBoolean(true);
	case 'f':
		expectLiteral(text, pos, "false");
		return makeBoolean(false);
	case 'n':
		expectLiteral(text, pos, "null");
		return makeNull();
	default:
		if (text[pos] == '-' || std::isdigit(static_cast<unsigned char>(text[pos]))) {
			return makeNumber(parseNumber(text, pos));
		}
		throw std::runtime_error(
			std::string("Json: unexpected character '") + text[pos] + "'");
	}
}

Json Json::makeNull()
{
	return Json(Type::Null, 0.0, "", false);
}

Json Json::makeBoolean(bool value)
{
	return Json(Type::Bool, 0.0, "", value);
}

Json Json::makeNumber(double value)
{
	if (!std::isfinite(value)) {
		throw std::runtime_error("Json: cannot serialize a non-finite number");
	}
	return Json(Type::Number, value, "", false);
}

Json Json::makeString(std::string value)
{
	return Json(Type::String, 0.0, std::move(value), false);
}

Json Json::makeArray(std::vector<Json> values)
{
	Json result;
	result._type = Type::Array;
	result._array = std::move(values);
	return result;
}

Json Json::makeObject()
{
	Json result;
	result._type = Type::Object;
	return result;
}

Json Json::parse(const std::string & text)
{
	size_t pos = 0;
	Json value = parseValue(text, pos);
	skipWhitespace(text, pos);
	if (pos != text.size()) {
		throw std::runtime_error("Json: trailing characters after document");
	}
	return value;
}

bool Json::has(const std::string & key) const
{
	if (!isObject()) {
		return false;
	}
	for (const auto & kv : _object) {
		if (kv.first == key) {
			return true;
		}
	}
	return false;
}

Json & Json::at(const std::string & key)
{
	if (!isObject()) {
		throw std::runtime_error("Json: 'at(key)' called on non-object");
	}
	for (auto & kv : _object) {
		if (kv.first == key) {
			return kv.second;
		}
	}
	throw std::runtime_error("Json: key not found: " + key);
}

const Json & Json::at(const std::string & key) const
{
	if (!isObject()) {
		throw std::runtime_error("Json: 'at(key)' called on non-object");
	}
	for (const auto & kv : _object) {
		if (kv.first == key) {
			return kv.second;
		}
	}
	throw std::runtime_error("Json: key not found: " + key);
}

void Json::set(const std::string & key, Json value)
{
	if (!isObject()) {
		throw std::runtime_error("Json: 'set(key)' called on non-object");
	}
	for (auto & kv : _object) {
		if (kv.first == key) {
			kv.second = std::move(value);
			return;
		}
	}
	_object.emplace_back(key, std::move(value));
}

Json & Json::at(size_t index)
{
	if (!isArray()) {
		throw std::runtime_error("Json: 'at(index)' called on non-array");
	}
	if (index >= _array.size()) {
		throw std::runtime_error("Json: array index out of range");
	}
	return _array[index];
}

const Json & Json::at(size_t index) const
{
	if (!isArray()) {
		throw std::runtime_error("Json: 'at(index)' called on non-array");
	}
	if (index >= _array.size()) {
		throw std::runtime_error("Json: array index out of range");
	}
	return _array[index];
}

namespace {

void appendEscapedString(const std::string & value, std::string & output)
{
	static constexpr char kHex[] = "0123456789abcdef";
	output.push_back('"');
	for (const unsigned char c : value) {
		switch (c) {
		case '"': output += "\\\""; break;
		case '\\': output += "\\\\"; break;
		case '\b': output += "\\b"; break;
		case '\f': output += "\\f"; break;
		case '\n': output += "\\n"; break;
		case '\r': output += "\\r"; break;
		case '\t': output += "\\t"; break;
		default:
			if (c < 0x20U) {
				output += "\\u00";
				output.push_back(kHex[(c >> 4U) & 0x0FU]);
				output.push_back(kHex[c & 0x0FU]);
			} else {
				output.push_back(static_cast<char>(c));
			}
		}
	}
	output.push_back('"');
}

void appendIndent(std::string & output, int depth)
{
	output.append(static_cast<size_t>(depth) * 2U, ' ');
}

void appendJson(const Json & value, std::string & output, int depth)
{
	switch (value.type()) {
	case Json::Type::Null:
		output += "null";
		break;
	case Json::Type::Bool:
		output += value.asBoolean() ? "true" : "false";
		break;
	case Json::Type::Number: {
		std::ostringstream number;
		number << std::setprecision(17) << value.asNumber();
		output += number.str();
		break;
	}
	case Json::Type::String:
		appendEscapedString(value.asString(), output);
		break;
	case Json::Type::Array: {
		output.push_back('[');
		if (!value.empty()) {
			output.push_back('\n');
			for (size_t i = 0; i < value.size(); ++i) {
				appendIndent(output, depth + 1);
				appendJson(value.at(i), output, depth + 1);
				if (i + 1U < value.size()) {
					output.push_back(',');
				}
				output.push_back('\n');
			}
			appendIndent(output, depth);
		}
		output.push_back(']');
		break;
	}
	case Json::Type::Object: {
		output.push_back('{');
		const auto & entries = value.objectEntries();
		if (!entries.empty()) {
			output.push_back('\n');
			for (size_t i = 0; i < entries.size(); ++i) {
				appendIndent(output, depth + 1);
				appendEscapedString(entries[i].first, output);
				output += ": ";
				appendJson(entries[i].second, output, depth + 1);
				if (i + 1U < entries.size()) {
					output.push_back(',');
				}
				output.push_back('\n');
			}
			appendIndent(output, depth);
		}
		output.push_back('}');
		break;
	}
	}
}

double requireFiniteNumber(const Json & value, const std::string & field)
{
	if (value.type() != Json::Type::Number || !std::isfinite(value.asNumber())) {
		throw std::runtime_error("PlanParser: '" + field + "' must be a finite number");
	}
	return value.asNumber();
}

void collectWaypoints(const Json & node, SearchPlan & plan)
{
	if (node.isArray()) {
		for (const auto & child : node.arrayItems()) {
			collectWaypoints(child, plan);
		}
		return;
	}

	if (!node.isObject()) {
		return;
	}

	if (node.has("command") && node.has("params")) {
		const Json & command_value = node.at("command");
		const Json & params = node.at("params");
		if (command_value.type() == Json::Type::Number &&
		    command_value.asNumber() == 16.0 && params.isArray() && params.size() >= 7U) {
			const double lat = requireFiniteNumber(params.at(4), "waypoint latitude");
			const double lon = requireFiniteNumber(params.at(5), "waypoint longitude");
			if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
				throw std::runtime_error("PlanParser: waypoint latitude/longitude is out of range");
			}
			plan.waypoints.push_back({lat, lon});
		}
	}

	for (const auto & kv : node.objectEntries()) {
		if (kv.first == "params" && kv.second.isArray()) {
			continue;
		}
		collectWaypoints(kv.second, plan);
	}
}

float extractAltitude(const Json & node, float fallback)
{
	if (node.isObject()) {
		if (node.has("CameraCalc")) {
			const Json & camera = node.at("CameraCalc");
			if (camera.isObject() && camera.has("DistanceToSurface")) {
				return static_cast<float>(requireFiniteNumber(
					camera.at("DistanceToSurface"), "CameraCalc.DistanceToSurface"));
			}
		}
		for (const auto & kv : node.objectEntries()) {
			const float found = extractAltitude(kv.second, fallback);
			if (found != fallback) {
				return found;
			}
		}
	} else if (node.isArray()) {
		for (const auto & child : node.arrayItems()) {
			const float found = extractAltitude(child, fallback);
			if (found != fallback) {
				return found;
			}
		}
	}
	return fallback;
}

}  // namespace

std::string Json::dump() const
{
	std::string output;
	appendJson(*this, output, 0);
	return output;
}

SearchPlan parseSearchPlan(const std::string & plan_file)
{
	std::ifstream input(plan_file);
	if (!input) {
		throw std::runtime_error("PlanParser: cannot open plan file: " + plan_file);
	}

	std::stringstream buffer;
	buffer << input.rdbuf();
	const Json root = Json::parse(buffer.str());

	if (!root.isObject() || !root.has("mission")) {
		throw std::runtime_error("PlanParser: 'mission' not found in plan file");
	}

	SearchPlan plan;
	const Json & mission = root.at("mission");
	if (!mission.isObject()) {
		throw std::runtime_error("PlanParser: 'mission' must be an object");
	}

	if (mission.has("cruiseSpeed")) {
		plan.cruise_speed = static_cast<float>(requireFiniteNumber(
			mission.at("cruiseSpeed"), "mission.cruiseSpeed"));
	}
	plan.altitude = extractAltitude(mission, plan.altitude);

	if (mission.has("searchPlanner")) {
		plan.has_search_planner_metadata = true;
		const Json & metadata = mission.at("searchPlanner");
		if (!metadata.isObject()) {
			throw std::runtime_error("PlanParser: 'mission.searchPlanner' must be an object");
		}
		if (metadata.has("entryPoint")) {
			const Json & entry = metadata.at("entryPoint");
			if (!entry.isArray() || entry.size() < 2U) {
				throw std::runtime_error(
					"PlanParser: 'searchPlanner.entryPoint' must contain latitude and longitude");
			}
			const double lat = requireFiniteNumber(entry.at(0), "searchPlanner.entryPoint latitude");
			const double lon = requireFiniteNumber(entry.at(1), "searchPlanner.entryPoint longitude");
			if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
				throw std::runtime_error("PlanParser: search planner entry point is out of range");
			}
			plan.entry_point = std::array<double, 2>{lat, lon};
		}
		if (metadata.has("nextWaypointIndex")) {
			const double index_value = requireFiniteNumber(
				metadata.at("nextWaypointIndex"), "searchPlanner.nextWaypointIndex");
			if (index_value < 0.0 || std::floor(index_value) != index_value) {
				throw std::runtime_error(
					"PlanParser: searchPlanner.nextWaypointIndex must be a non-negative integer");
			}
			plan.next_waypoint_index = static_cast<std::size_t>(index_value);
		}
	}

	if (mission.has("items") && mission.at("items").isArray()) {
		collectWaypoints(mission.at("items"), plan);
	}

	if (plan.waypoints.empty()) {
		throw std::runtime_error("PlanParser: no waypoints (command 16) found in plan file");
	}
	if (plan.next_waypoint_index > plan.waypoints.size()) {
		throw std::runtime_error(
			"PlanParser: searchPlanner.nextWaypointIndex exceeds waypoint count");
	}

	return plan;
}

void updateSearchPlanEntryPoint(
	const std::string & plan_file,
	const std::array<double, 2> & entry_point,
	std::size_t next_waypoint_index)
{
	if (!std::isfinite(entry_point[0]) || !std::isfinite(entry_point[1]) ||
	    entry_point[0] < -90.0 || entry_point[0] > 90.0 ||
	    entry_point[1] < -180.0 || entry_point[1] > 180.0) {
		throw std::runtime_error("PlanParser: invalid search planner entry point");
	}

	const SearchPlan current_plan = parseSearchPlan(plan_file);
	if (next_waypoint_index > current_plan.waypoints.size()) {
		throw std::runtime_error("PlanParser: next waypoint index exceeds waypoint count");
	}

	std::ifstream input(plan_file);
	if (!input) {
		throw std::runtime_error("PlanParser: cannot reopen plan file: " + plan_file);
	}
	std::stringstream buffer;
	buffer << input.rdbuf();
	Json root = Json::parse(buffer.str());
	Json & mission = root.at("mission");
	if (!mission.isObject()) {
		throw std::runtime_error("PlanParser: 'mission' must be an object");
	}

	Json planner = Json::makeObject();
	planner.set("entryPoint", Json::makeArray({
		Json::makeNumber(entry_point[0]),
		Json::makeNumber(entry_point[1])}));
	planner.set("nextWaypointIndex", Json::makeNumber(
		static_cast<double>(next_waypoint_index)));
	mission.set("searchPlanner", std::move(planner));

	const std::filesystem::path plan_path(plan_file);
	const std::filesystem::path temporary_path = plan_path.string() + ".tmp";
	{
		std::ofstream output(temporary_path, std::ios::trunc);
		if (!output) {
			throw std::runtime_error(
				"PlanParser: cannot write temporary working plan: " + temporary_path.string());
		}
		output << root.dump() << '\n';
		if (!output) {
			throw std::runtime_error(
				"PlanParser: failed while writing temporary working plan: " + temporary_path.string());
		}
	}

	std::error_code error;
	std::filesystem::rename(temporary_path, plan_path, error);
	if (error) {
		std::filesystem::remove(temporary_path);
		throw std::runtime_error(
			"PlanParser: cannot replace working plan '" + plan_file + "': " + error.message());
	}
}

}  // namespace search
