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
 
#ifndef RSYN_ISPD2018_READER_H
#define RSYN_ISPD2018_READER_H

#include <Rsyn/Session>

namespace Rsyn  {
	
class RoutingGuide;
	
class ISPD2018Reader : public Reader {
public:
	ISPD2018Reader() = default;
	virtual bool load(const Rsyn::Json& params) override;
	
private:
	Session session;
	
	std::string lefFile;
	std::string defFile;
	std::string guideFile;
	LefDscp lefDescriptor;
	DefDscp defDescriptor;
	RoutingGuide *routingGuide;
	
	void parsingFlow();
	void parseLEFFile();
	void parseDEFFile();
	void parseGuideFile();
	void populateDesign();
};

}

#endif /* ISPD2018READER_H */

