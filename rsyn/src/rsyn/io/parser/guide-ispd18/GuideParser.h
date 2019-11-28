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
 
#ifndef ISPD18GUIDEPARSER_H
#define	ISPD18GUIDEPARSER_H
#include <fstream>
#include <vector>
#include "GuideDescriptor.h"
#include "rsyn/util/Bounds.h"
/*net1230
(
95010 71819 100710 91201 Metal2
95010 83220 100710 91201 Metal1
89310 71819 100710 77520 Metal3
95010 71819 100710 77520 Metal2
89310 71819 95010 77520 Metal2
89310 71819 95010 77520 Metal2
89310 71819 95010 77520 Metal1
)
 */

class GuideParser {
protected:
	 std::ifstream clsIS;
public:
	GuideParser() = default;
	void parse(std::string & guidePath, GuideDscp & guideDscp);
	
protected:
	bool readLine(std::vector<std::string>& tokens);
	bool readNet(std::string & net);
	bool isStartLayer(std::string & token);
	bool isEndLayer(std::string & token);
	bool readLayerGuide(GuideNetDscp & dscp);
	
};




#endif /* ISPD18GUIDEPARSER_H */