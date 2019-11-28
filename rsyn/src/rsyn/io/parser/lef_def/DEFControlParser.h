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
 
#ifndef DEFCONTROLPARSER_H
#define	DEFCONTROLPARSER_H

#include <string>
	using std::string;
#include <iostream>
	using std::cout;
#include <vector>
	using std::vector;

#include "rsyn/io/legacy/PlacerInternals.h"

//! DEF files must be parsed after LEF files
	
#include "rsyn/phy/util/DefDescriptors.h"

class DEFControlParser {
public:
	DEFControlParser();
	void parseDEF(const std::string &filename, DefDscp &defDscp) ;
	void writeDEF(const std::string &filename, const std::string designName, const std::vector<DefComponentDscp> &components);
	void writeFullDEF(const std::string &filename, const DefDscp & defDscp);
	virtual ~DEFControlParser();
	
	static std::string unescape(const std::string &str);
	
}; // end class

#endif	/* DEFCONTROLPARSER_H */

