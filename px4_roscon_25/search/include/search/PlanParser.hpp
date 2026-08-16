#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace search {

/**
 * @brief A small JSON DOM used to read and update a QGroundControl .plan.
 *
 * This is intentionally not a general-purpose JSON library. It supports the
 * constructs used by a QGC plan and the SearchPlanner metadata written to the
 * working plan.
 */
class Json
{
public:
	enum class Type {
		Null,
		Bool,
		Number,
		String,
		Array,
		Object,
	};

	Json() = default;
	Json(Type type, double num, std::string str, bool boolean)
		: _type(type), _num(num), _string(std::move(str)), _boolean(boolean) {}

	static Json parse(const std::string & text);
	static Json makeNull();
	static Json makeBoolean(bool value);
	static Json makeNumber(double value);
	static Json makeString(std::string value);
	static Json makeArray(std::vector<Json> values);
	static Json makeObject();

	Type type() const { return _type; }
	bool isObject() const { return _type == Type::Object; }
	bool isArray() const { return _type == Type::Array; }

	// Object access
	bool has(const std::string & key) const;
	Json & at(const std::string & key);  // throws if missing / not an object
	const Json & at(const std::string & key) const;
	void set(const std::string & key, Json value);

	// Array access
	size_t size() const { return _array.size(); }
	bool empty() const { return _array.empty(); }
	Json & at(size_t index);             // throws if out of range / not an array
	const Json & at(size_t index) const;
	const std::vector<Json> & arrayItems() const { return _array; }

	// Scalars
	bool asBoolean() const { return _boolean; }
	double asNumber() const { return _num; }
	const std::string & asString() const { return _string; }

	// Object entries
	const std::vector<std::pair<std::string, Json>> & objectEntries() const { return _object; }

	/// Serialize this JSON value as a readable JSON document.
	std::string dump() const;

private:
	static Json parseValue(const std::string & text, size_t & pos);
	static void skipWhitespace(const std::string & text, size_t & pos);
	static void expectLiteral(const std::string & text, size_t & pos, const char * literal);
	static std::string parseString(const std::string & text, size_t & pos);
	static Json parseObject(const std::string & text, size_t & pos);
	static Json parseArray(const std::string & text, size_t & pos);
	static double parseNumber(const std::string & text, size_t & pos);

	Type _type{Type::Null};
	double _num{0.0};
	std::string _string;
	bool _boolean{false};
	std::vector<Json> _array;
	std::vector<std::pair<std::string, Json>> _object;
};

/**
 * @brief Data extracted from a QGroundControl plan.
 */
struct SearchPlan
{
	std::vector<std::array<double, 2>> waypoints;  // {latitude, longitude}
	float altitude{15.0F};                         // [m] above home (relative)
	float cruise_speed{5.0F};                      // [m/s], informational

	// SearchPlanner metadata stored in the working plan. The index identifies
	// the first original waypoint that still needs to be searched.
	std::optional<std::array<double, 2>> entry_point;
	std::size_t next_waypoint_index{0U};
	bool has_search_planner_metadata{false};
};

/**
 * @brief Parse a QGroundControl .plan and extract command-16 waypoints.
 *
 * If the plan contains SearchPlanner metadata, it is read from:
 * mission.searchPlanner.entryPoint and mission.searchPlanner.nextWaypointIndex.
 */
SearchPlan parseSearchPlan(const std::string & plan_file);

/**
 * @brief Persist the latest Search entry point and progress in a working plan.
 *
 * The file is rewritten atomically through a sibling temporary file.
 */
void updateSearchPlanEntryPoint(
	const std::string & plan_file,
	const std::array<double, 2> & entry_point,
	std::size_t next_waypoint_index);

}  // namespace search
