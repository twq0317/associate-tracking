/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019,
*  XIAOSHU Technology  Co., Ltd
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the institute nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: WQ.Tang
*********************************************************************/

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

	std::string float2string(float f)
	{
		std::stringstream ss;
		ss << f;
		return ss.str();
	}
};

