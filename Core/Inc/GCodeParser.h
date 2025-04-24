/**
 ******************************************************************************
 * @file           : GCodeParser.h
 * @brief          :
 * @author         : Chris Hauser
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef GCODEPARSER_H
#define GCODEPARSER_H

#include <string>
#include <map>
#include <cctype>
#include <cstdlib>
#include "stdint.h"
#include <optional>

class Robot;

class GCodeParser {
public:
	GCodeParser(Robot* myParent) : parent(myParent) {}
	struct GCodeCommand {
		char commandType = '\0';  //G, M
		uint16_t commandValue = 0;
		std::map<char, float> parameters; //X, Y, Z, F
	} result;

	bool parseGCodeLineAndPushInBuffer(const char *input);
	bool parseGCode(const char *input);
	bool transformAndPushParameter(Robot* rob);

private:
	Robot* parent = nullptr;
	std::optional<float> getParam(const std::map<char, float> &params,
			char key);

};

#endif /* GCODEPARSER_H */
