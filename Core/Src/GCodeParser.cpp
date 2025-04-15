/**
 ******************************************************************************
 * @file           : GCodeParser.cpp
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

#include "GCodeParser.h"
#include "move.h"
#include <map>

/**
 * @brief Trennt String in Befehle auf und übergibt Befehl an Roboter
 * @param *input: Array welcher den G-Code-String enthält
 * @retval bool: true wenn erfolgreich, false sonst
 */
bool GCodeParser::parseGCodeLineAndPushInBuffer(const char *input) {
	if (parseGCode(input))
		if (transformAndPushParameter(parent))
			return true;
	return false;
}

/**
 * @brief Trennt einen String auf und extrahiert die Befehle. Ignoriert Kommentare
 * @param *input: Array welcher den G-Code-String enthält
 * @retval bool: true wenn erfolgreich, false wenn Kommentar
 */
bool GCodeParser::parseGCode(const char *input) {
	std::string str(input);

	// Ignoriere Zeile, wenn sie mit einem Kommentar beginnt (z.B. ';' oder '(') oder leer ist
	if (str.empty() || str[0] == ';' || str[0] == '(' || str[0] == '\n') {
		return false;
	}

	size_t i = 0;
	while (i < str.length()) {
		// Ignoriere Leerzeichen
		if (isspace(str[i])) {
			i++;
			continue;
		}

		// Ignoriere Kommentare innerhalb der Zeile (alles ab ';' oder '(')
		if (str[i] == ';' || str[i] == '(') {
			break;
		}

		char prefix = toupper(str[i]);
		i++;

		// Lese Float-Zahl nach dem Buchstaben
		std::string numberStr;
		while (i < str.length()
				&& (isdigit(str[i]) || str[i] == '.' || str[i] == '-')) {
			numberStr += str[i++];
		}

		float value = std::stof(numberStr);

		if (prefix == 'G' || prefix == 'M') {
			result.commandType = prefix;
			result.commandValue = value;
		} else {
			result.parameters[prefix] = value;
		}
	}

	return true;
}

/**
 * @brief Übergibt die einzelnen Befehle sofern vorhanden
 * @param *rob: Pointer auf Roboter
 * @retval bool: true wenn erfolgreich, false sonst
 */
bool GCodeParser::transformAndPushParameter(Robot *rob) {
	bool status = false;
	;
	switch (result.commandType) {
	case 'G': { // TODO G0 G1 G28 etc.
		switch (result.commandValue) {
		case 0: {
			Robot::MoveParams param;
			param.x = getParam(result.parameters, 'X');
			param.y = getParam(result.parameters, 'Y');
			param.accel = getParam(result.parameters, 'A');
			param.printing = std::nullopt;
			param.speed = DEFAULT_SPEED;
			status = rob->moveToPos(param);
			break;
		}
		case 1: {
			Robot::MoveParams param;
			param.x = getParam(result.parameters, 'X');
			param.y = getParam(result.parameters, 'Y');
			param.accel = getParam(result.parameters, 'A');
			param.printing = getParam(result.parameters, 'S');
			param.speed = getParam(result.parameters, 'F');
			status = rob->moveToPos(param);
			break;
		}
		default: {
			assert("unbekannter Befehl");
			status = false;
			break;
		}
		}
		break;
	}
	case 'M': {
		switch (result.commandValue) {
		case 0: {
			break;
		}
		case 1: {
			break;
		}
		case 5: {
			rob->motorMaster.motorX.tmc.disable();
			rob->motorMaster.motorY.tmc.disable();
			rob->printhead.stop();
			break;
		}
		}
		break;
	}
	default: {
		assert("unbekannter Befehl");
		status = false;
		break;
	}
	}
	return status;
}

/**
 * @brief Gibt den
 * @param &parameterMap: Referenz auf map mit Parametern und zugehörigen Werten
 * @param key: Parameter, der gefunden werden soll
 * @retval Parameterwert wenn vorhanden, nullopt sonst
 */
std::optional<float> GCodeParser::getParam(
		const std::map<char, float> &parameterMap, char key) {
	if (parameterMap.count(key)) {
		return parameterMap.at(key);
	}
	return std::nullopt;
}
