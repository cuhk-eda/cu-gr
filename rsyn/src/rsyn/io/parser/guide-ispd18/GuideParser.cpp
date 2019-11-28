/* Copyright 2014-2018 Rsyn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <iostream>
#include "rsyn/io/parser/guide-ispd18/GuideParser.h"

void GuideParser::parse(std::string& guidePath, GuideDscp& guideDscp) {
	clsIS.open(guidePath.c_str());
	if (!clsIS.is_open()) {
		std::cout << "[ERROR] File '" << guidePath << "' could not be opened.\n";
		exit(1);
	} // end if

	while (!clsIS.eof()) {
		std::string netName;
		if(!readNet(netName))
			continue;
		std::deque<GuideNetDscp> & nets = guideDscp.clsNetGuides;
		nets.push_back(GuideNetDscp());
		GuideNetDscp & net = nets.back();
		net.clsNetName = netName;
		readLayerGuide(net);
	} // end while 
	clsIS.close();
} // end method 

// -----------------------------------------------------------------------------

bool GuideParser::readLine(std::vector<std::string>& tokens) {
	tokens.clear();
	std::string line;
	std::getline(clsIS, line);
	std::string token = "";

	for (unsigned i = 0; i < line.size(); ++i) {
		char current = line[i];
		if (std::isspace(current)) {
			if (!token.empty()) {
				tokens.push_back(token);
				token.clear();
			} // end if 
		} else {
			token.push_back(current);
		} //vend if-else 
	} // end for 
	if (!token.empty())
		tokens.push_back(token);

	return !clsIS.eof();
} // end method 

// -----------------------------------------------------------------------------

bool GuideParser::readNet(std::string & net) {
	std::vector<std::string> tokens;
	readLine(tokens);
	if (tokens.empty())
		return false;
	net = tokens.front();
	return true;
} // end method 

// -----------------------------------------------------------------------------

bool GuideParser::isStartLayer(std::string & token) {
	return token.compare("(") == 0;
} // end method 

// -----------------------------------------------------------------------------

bool GuideParser::isEndLayer(std::string & token) {
	return token.compare(")") == 0;
} // end method 

// -----------------------------------------------------------------------------

bool GuideParser::readLayerGuide(GuideNetDscp & dscp) {
	while (true) {
		std::vector<std::string> tokens;
		readLine(tokens);
		if (isStartLayer(tokens.front()))
			continue;
		if (isEndLayer(tokens.front()))
			return true;
		if(tokens.size() < 5) { 
			std::cout<<"WARNING: skipping parsing a layer guide of net "<<dscp.clsNetName
				<<". The guide definition has less then four points or it do not has defined the layer name.\n";
			continue;
		} // end if 
			
		dscp.clsLayerDscps.push_back(GuideLayerDscp());
		GuideLayerDscp & layer = dscp.clsLayerDscps.back();
		Bounds & bds = layer.clsLayerGuide;
		bds[LOWER][X] = std::stoi(tokens[0]);
		bds[LOWER][Y] = std::stoi(tokens[1]);
		bds[UPPER][X] = std::stoi(tokens[2]);
		bds[UPPER][Y] = std::stoi(tokens[3]);
		layer.clsLayer = tokens[4];
	} // end while 
	return false;
} // end method 

// -----------------------------------------------------------------------------