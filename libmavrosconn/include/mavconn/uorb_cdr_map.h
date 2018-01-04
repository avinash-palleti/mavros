#pragma once

#include <map>
#include <mavconn/home_position_.h>
namespace uorbmap
{
static std::map<std::string, int> uorbMap;

static void initialize_uorb_map()
{
	// Initialize map between class names and topic_id
	uorbMap.insert(std::make_pair(typeid(home_position_).name(), 33));
}
}
