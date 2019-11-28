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
 
#ifndef RSYN_SESSION_H
#define RSYN_SESSION_H

#include <string>

#include "rsyn/session/Service.h"
#include "rsyn/session/Reader.h"

#include "rsyn/util/Json.h"

#include "rsyn/core/Rsyn.h"
#include "rsyn/util/Units.h"

namespace Rsyn {

class PhysicalDesign;

typedef std::function<Service *()> ServiceInstantiatonFunction;
typedef std::function<Reader *()> ReaderInstantiatonFunction;

////////////////////////////////////////////////////////////////////////////////
// Session Data
////////////////////////////////////////////////////////////////////////////////

struct SessionData {
	////////////////////////////////////////////////////////////////////////////
	// Session Variables
	////////////////////////////////////////////////////////////////////////////
	
	std::map<std::string, Rsyn::Json> clsSessionVariables;

	////////////////////////////////////////////////////////////////////////////
	// Services
	////////////////////////////////////////////////////////////////////////////
	
	// Generic functions used to instantiate optimizers by name.
	std::unordered_map<std::string, ServiceInstantiatonFunction> clsServiceInstanciationFunctions;
	std::unordered_map<std::string, Service *> clsRunningServices;

	////////////////////////////////////////////////////////////////////////////
	// Loader
	////////////////////////////////////////////////////////////////////////////
	
	// Generic functions used to instantiate optimizers by name.
	std::unordered_map<std::string, ReaderInstantiatonFunction> clsReaders;
	
	////////////////////////////////////////////////////////////////////////////
	// Design/Library
	////////////////////////////////////////////////////////////////////////////

	Rsyn::Design clsDesign;
	Rsyn::Library clsLibrary;

	////////////////////////////////////////////////////////////////////////////
	// Configuration
	////////////////////////////////////////////////////////////////////////////
	std::string clsInstallationPath;
	
	////////////////////////////////////////////////////////////////////////////
	// Misc
	////////////////////////////////////////////////////////////////////////////
	std::list<std::string> clsPaths;
}; // end struct

////////////////////////////////////////////////////////////////////////////////
// Session Proxy
////////////////////////////////////////////////////////////////////////////////

class Session : public Rsyn::Proxy<SessionData> {
public:
	Session() {
		data = sessionData;
	};
	
	Session(std::nullptr_t) { 
		data = nullptr;
	}
	
	Session &operator=(const Session &other) {
		data = other.data;
		return *this;
	}	
	
	static void init();

	//! @note To prevent "static variable order initialization fiasco", the
	//        static variable signaling that the engine was initialized is
	//        stored inside this function. In this way, we can guarantee it will
	//        be initialized to false before being used.
	static bool checkInitialized(const bool markAsInitialized = false) {
		static bool sessionInitialized = false;
		if (markAsInitialized)
			sessionInitialized = true;
		return sessionInitialized;
	} // end method

private:

	static SessionData * sessionData;

	////////////////////////////////////////////////////////////////////////////
	// Session Variables
	////////////////////////////////////////////////////////////////////////////

public:

	static void setSessionVariable(const std::string &name, const Rsyn::Json &value) {
		sessionData->clsSessionVariables[name] = value;
	} // end method

	static void unsetSessionVariable(const std::string &name) {
		sessionData->clsSessionVariables.erase(name);
	} // end method

	static const bool getSessionVariableAsBool(const std::string &name, const bool defaultValue = false) {
		auto it = sessionData->clsSessionVariables.find(name);
		return (it != sessionData->clsSessionVariables.end())? it->second.get<bool>() : defaultValue;
	} // end method

	static const int getSessionVariableAsInteger(const std::string &name, const int defaultValue = 0) {
		auto it = sessionData->clsSessionVariables.find(name);
		return (it != sessionData->clsSessionVariables.end())? it->second.get<int>() : defaultValue;
	} // end method

	static const float getSessionVariableAsFloat(const std::string &name, const float defaultValue = 0.0f) {
		auto it = sessionData->clsSessionVariables.find(name);
		return (it != sessionData->clsSessionVariables.end())? it->second.get<float>() : defaultValue;
	} // end method

	static const std::string getSessionVariableAsString(const std::string &name, const std::string &defaultValue = "") {
		auto it = sessionData->clsSessionVariables.find(name);
		return (it != sessionData->clsSessionVariables.end())? it->second.get<std::string>() : defaultValue;
	} // end method

	static const Rsyn::Json getSessionVariableAsJson(const std::string &name, const Rsyn::Json &defaultValue = {}) {
		auto it = sessionData->clsSessionVariables.find(name);
		return (it != sessionData->clsSessionVariables.end())? it->second : defaultValue;
	} // end method
	
	////////////////////////////////////////////////////////////////////////////
	// Services
	////////////////////////////////////////////////////////////////////////////

	// Register services.
	static void registerServices();

public:
	
	// Helper class to allow seamless casting from a Service pointer to any type
	// which implements operator=(Service *) is is directly compatible.
	
	class ServiceHandler {
	private:
		Service *clsService;
		
	public:
		ServiceHandler(Service *service) : clsService(service) {}
		
		template<typename T>
		operator T *() {
			T * pointer = dynamic_cast<T *>(clsService);
			if (pointer != clsService) {
				std::cout << "ERROR: Trying to cast a service to the wrong type.\n";
				throw Exception("Trying to cast a service to the wrong type.");
			} // end if
			return pointer;
		} // end operator
	}; // end class
		
	// Register a service.
	template<typename T>
	static void registerService(const std::string &name) {
		auto it = sessionData->clsServiceInstanciationFunctions.find(name);
		if (it != sessionData->clsServiceInstanciationFunctions.end()) {
			std::cout << "ERROR: Service '" << name << "' was already "
					"registered.\n";
			std::exit(1);
		} else {
			sessionData->clsServiceInstanciationFunctions[name] = []() -> Service *{
				return new T();
			};
		} // end else
	} // end method
	
	// Start a service.
	static bool startService(const std::string &name, const Rsyn::Json &params = {}, const bool dontErrorOut = false);
	
	// Gets a running service.
	static ServiceHandler getService(const std::string &name,
			const ServiceRequestType requestType = SERVICE_MANDATORY) {
		Service *service = getServiceInternal(name);
		if (!service && (requestType == SERVICE_MANDATORY)) {
			std::cout << "ERROR: Service '" << name << "' was not started.\n";
			throw Exception("ERROR: Service '" + name + "' was not started");
		} // end if
		return ServiceHandler(service);
	} // end method

	// Checks if a service is registered.
	static bool isServiceRegistered(const std::string &name) {
		auto it = sessionData->clsServiceInstanciationFunctions.find(name);
		return  (it != sessionData->clsServiceInstanciationFunctions.end());
	} // end method

	// Checks if a service is running.
	static bool isServiceRunning(const std::string &name) {
		return getServiceInternal(name) != nullptr;
	} // end method
	
private:
	
	static Service * getServiceInternal(const std::string &name) {
		auto it = sessionData->clsRunningServices.find(name);
		return it == sessionData->clsRunningServices.end()? nullptr : it->second;
	} // end method

	static void listService(std::ostream & out = std::cout) {
		out<<"List of services ";
		out<<"([R] -> Running, [S] -> Stopped):\n";
		// print only running services
		for (std::pair<std::string, ServiceInstantiatonFunction> srv : sessionData->clsServiceInstanciationFunctions) {
			if (!isServiceRunning(srv.first))
				continue;
			out << "\t[R] " << srv.first << "\n";
		} // end for 
		// print only stopped services 
		for (std::pair<std::string, ServiceInstantiatonFunction> srv : sessionData->clsServiceInstanciationFunctions) {
			if (isServiceRunning(srv.first))
				continue;
			out << "\t[S] "<<srv.first << "\n";
		} // end for 
		out << "--------------------------------------\n";
	} /// end method 
	
	////////////////////////////////////////////////////////////////////////////
	// Reader
	////////////////////////////////////////////////////////////////////////////
private:
	static void listReader(std::ostream & out = std::cout) {
		out<<"List of registered Readers:\n";
		for(std::pair<std::string, ReaderInstantiatonFunction> reader : sessionData->clsReaders) {
			out<<"\t"<<reader.first<<"\n";
		} // end for 
		out<<"--------------------------------------\n";
	} /// end method

	// Register loader.
	static void registerReaders();

public:
		
	// Register a loader.
	template<typename T>
	static void registerReader(const std::string &name) {
		auto it = sessionData->clsReaders.find(name);
		if (it != sessionData->clsReaders.end()) {
			std::cout << "ERROR: Reader '" << name << "' was already "
					"registered.\n";
			std::exit(1);
		} else {
			sessionData->clsReaders[name] = []() -> Reader *{
				return new T();
			};
		} // end else
	} // end method
	
	// Run an loader.
	static void runReader(const std::string &name, const Rsyn::Json &params = {});
	
	////////////////////////////////////////////////////////////////////////////
	// Misc
	////////////////////////////////////////////////////////////////////////////

	static Rsyn::Design getDesign();
	static Rsyn::Library getLibrary();
	static Rsyn::Module getTopModule();
	static Rsyn::PhysicalDesign getPhysicalDesign();

	static const std::string &getInstallationPath() { return sessionData->clsInstallationPath; }

	////////////////////////////////////////////////////////////////////////////
	// Utilities
	////////////////////////////////////////////////////////////////////////////

private:

	static std::string mergePathAndFileName(const std::string &path, const std::string &fileName);

public:

	//! @brief Find a file in the current path. If found, returns its absolute
	//!        path, an empty string otherwise.
	//! @param extraPath can be used to specify an extra path location besides
	//!        the one stored internally in the current path list.
	std::string findFile(const std::string fileName, const std::string extraPath = "");
	
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Startup
////////////////////////////////////////////////////////////////////////////////

// Helper class used to perform component initialization during the application
// startup. Declare a startup object in a cpp file:
//
// Rsyn::Startup startup([]{
//     Rsyn::Session::registerService(...);
//     Rsyn::Session::registerMessage(...);
// }); // end startup
//
// This will construct a global object that will be called during the
// application initialization.

class Startup {
public:
	Startup(std::function<void()> f) {
		if (!Session::checkInitialized())
			Session::init();
		f();
	} // end constructor
}; // end class

} // end namespace

#endif /* INFRA_ICCAD15_SESSION_H_ */
