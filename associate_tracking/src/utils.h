#pragma once
#include <string>

namespace utils
{
	std::string replace_all(std::string str, const std::string old_value, const std::string new_value)
	{
		std::string result = str;
		while (true) {
			std::string::size_type pos(0);
			if ((pos = result.find(old_value)) != std::string::npos)
				result.replace(pos, old_value.length(), new_value);
			else break;
		}
		return result;
	}

	void split_last(const std::string& s, std::string& last, const char flag = '/') {
		std::istringstream iss(s);
		std::string temp;

		while (std::getline(iss, temp, flag)) {
			last = temp;
		}
		return;
	}

	std::vector<std::string> split(const std::string s, const char flag = ' ') {
		std::istringstream iss(s);
		std::vector<std::string> result;
		std::string temp;

		while (std::getline(iss, temp, flag)) {
			result.push_back(temp);
		}
		return result;
	}
};