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
 
//////////////////////////////////////////////////////////////////
//
//
//  Helper functions and classes to parse the ISPD 2013 contest
//  benchmark files.
//
//  This code is provided for description purposes only. The contest
//  organizers cannot guarantee that the provided code is free of
//  bugs or defects. !!!! USE THIS CODE AT YOUR OWN RISK !!!!!
//
//
//  The contestants are free to use these functions as-is or make
//  modifications. If the contestants choose to use the provided
//  code, they are responsible for making sure that it works as
//  expected.
//
//  The code provided here has no real or implied warranties.
//
//
////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <limits>
#include <cassert>
#include <cctype>
#include <cstring>
#include <algorithm>
#include <stdexcept>

#include "rsyn/io/parser/parser_helper.h"

namespace ISPD13 {

bool is_special_char (char c) {
    
    static const char specialChars[] = {'(', ')', ',', ';', '/', '#', '[', ']', '{', '}', '*', '\"', '\\'} ;
    
    for (unsigned i=0; i < sizeof(specialChars); ++i) {
        if (c == specialChars[i])
            return true ;
    }
    
    return false ;
}

// Read the next line and return it as a list of tokens skipping white space and special characters
// The return value indicates success/failure.
bool read_line_as_tokens (istream& is, vector<string>& tokens,
                          bool includeSpecialChars = false) {
    
    tokens.clear() ;
    
    string line ;
    std::getline (is, line) ;
    
    while (is && tokens.empty()) {
        
        string token = "" ;
        
        for (unsigned i=0; i < line.size(); ++i) {
            char currChar = line[i] ;
            bool isSpecialChar = is_special_char(currChar) ;
            
            if (std::isspace (currChar) || isSpecialChar) {
                
                if (!token.empty()) {
                    // Add the current token to the list of tokens
                    tokens.push_back(token) ;
                    token.clear() ;
                }
                
                if (includeSpecialChars && isSpecialChar) {
                    tokens.push_back(string(1, currChar)) ;
                }
                
            } else {
                // Add the char to the current token
                token.push_back(currChar) ;
            }
            
        }
        
        if (!token.empty())
            tokens.push_back(token) ;
        
        
        if (tokens.empty())
            // Previous line read was empty. Read the next one.
            std::getline (is, line) ;
    }
    
    //for (int i=0; i < tokens.size(); ++i)
    //  cout << tokens[i] << " " ;
    //cout << endl ;
    
    return !tokens.empty() ;
}



bool VerilogParser::read_module (string& moduleName) {
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    while (valid) {
        
        if (tokens.size() == 2 && tokens[0] == "module") {
            moduleName = tokens[1] ;
            
            break ;
        }
        
        valid = read_line_as_tokens (is, tokens) ;
    }
    
    // Read and skip the port names in the module definition
    // until we encounter the tokens {"Start", "PIs"}
    while (valid && !(tokens.size() == 2 && tokens[0] == "Start" && tokens[1] == "PIs")) {
        
        valid = read_line_as_tokens (is, tokens) ;
        assert (valid) ;
    }
    
    return valid ;
}

bool VerilogParser::read_primary_input (string& primaryInput) {
    
    primaryInput = "" ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens.size() == 2) ;
    
    if (valid && tokens[0] == "input") {
        primaryInput = tokens[1] ;
        
    } else {
        assert (tokens[0] == "Start" && tokens[1] == "POs") ;
        return false ;
    }
    
    return valid ;
}



bool VerilogParser::read_primary_output (string& primaryOutput) {
    
    primaryOutput = "" ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens.size() == 2) ;
    
    if (valid && tokens[0] == "output") {
        primaryOutput = tokens[1] ;
        
    } else {
        assert (tokens[0] == "Start" && tokens[1] == "wires") ;
        return false ;
    }
    
    return valid ;
}


bool VerilogParser::read_wire (string& wire) {
    
    wire = "" ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens.size() == 2) ;
    
    if (valid && tokens[0] == "wire") {
        wire = tokens[1] ;
        
    } else {
        assert (tokens[0] == "Start" && tokens[1] == "cells") ;
        return false ;
    }
    
    return valid ;
}


bool VerilogParser::read_cell_inst (string& cellType, string& cellInstName,
                                    vector<std::pair<string, string> >& pinNetPairs) {
    
    cellType = "" ;
    cellInstName = "" ;
    pinNetPairs.clear() ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    
    if (tokens.size() == 1) {
        assert (tokens[0] == "endmodule") ;
        return false ;
    }
    
    assert (tokens.size() >= 4) ; // We should have cellType, instName, and at least one pin-net pair
    
    cellType = tokens[0] ;
    cellInstName = tokens[1] ;
    
    for (unsigned i=2; i < tokens.size()-1; i += 2) {
        
        assert (tokens[i][0] == '.') ; // pin names start with '.'
        string pinName = tokens[i].substr(1) ; // skip the first character of tokens[i]
        
        pinNetPairs.push_back(std::make_pair(pinName, tokens[i+1])) ;
    }
    
    return valid ;
}

bool VerilogParser::read_module_tau15 (string& moduleName) {
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    while (valid) {
        
        if (tokens.size() == 2 && tokens[0] == "module") {
            moduleName = tokens[1] ;
            
            break ;
        }
        
        valid = read_line_as_tokens (is, tokens) ;
    }
    
    // Read and skip the port names in the module definition
    // until we encounter the tokens {"Start", "PIs"}
    while (valid && !count(tokens.begin(), tokens.end(), ";") == 1) {
        valid = read_line_as_tokens (is, tokens, true) ;
        assert (valid) ;
    }
    
    return valid ;
}


bool VerilogParser::read_module_ispd15 (string& moduleName) {
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    while (valid) {
        
        if (tokens[0] == "module") {
            moduleName = tokens[1] ;
            
            break ;
        }
        
        valid = read_line_as_tokens (is, tokens) ;
    }
    
    // Read and skip the port names in the module definition
    // until we encounter the tokens {"Start", "PIs"}
    while (valid && !count(tokens.begin(), tokens.end(), ";") == 1) {
        valid = read_line_as_tokens (is, tokens, true) ;
        assert (valid) ;
    }
    
    return valid ;
}

tauVerilog VerilogParser::read_line_tau15(string &aux){
	bool valid;
	do{
		valid = read_line_as_tokens (is, tauTokens) ;
	
		if(tauTokens[0]  == "input"){
			aux = tauTokens[1] ;
			return INPUT_TAU15;
		} else if (tauTokens [0] == "output"){
			aux = tauTokens[1] ;
			return OUTPUT_TAU15;
		} else if (tauTokens [0] == "Start") {
			valid = false;
		} else if (tauTokens [0] == "wire"){
			aux = tauTokens[1] ;
			return WIRE_TAU15;
		} else if(tauTokens [0] == "endmodule"){
			return ENDMOD_TAU15;
		}
		else{
			return GATE_TAU15;
		}
	}while(!valid);
	
	throw std::runtime_error("parsing error");
}

bool VerilogParser::read_cell_inst_tau15 (string& cellType, string& cellInstName,
                                    vector<std::pair<string, string> >& pinNetPairs) {
    
    cellType = "" ;
    cellInstName = "" ;
    pinNetPairs.clear() ;
    
    assert (tauTokens.size() >= 4) ; // We should have cellType, instName, and at least one pin-net pair
    
    cellType = tauTokens[0] ;
    cellInstName = tauTokens[1] ;
    
    for (unsigned i=2; i < tauTokens.size()-1; i += 2) {
        
        assert (tauTokens[i][0] == '.') ; // pin names start with '.'
        string pinName = tauTokens[i].substr(1) ; // skip the first character of tokens[i]
        
        pinNetPairs.push_back(std::make_pair(pinName, tauTokens[i+1])) ;
    }
    
    return true ;
}


// Read clock definition
// Return value indicates if the last read was successful or not.
bool SdcParser::read_clock (string& clockName, string& clockPort, double& period) {
    
    clockName = "" ;
    clockPort = "" ;
    period = 0.0 ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    while (valid) {
        
        if (tokens.size() == 7 && tokens[0] == "create_clock" && tokens[1] == "-name") {
            
            clockName = tokens[2] ;
            
            assert (tokens[3] == "-period") ;
            period = std::atof(tokens[4].c_str()) ;
            
            assert (tokens[5] == "get_ports") ;
            clockPort = tokens[6] ;
            break ;
        }
        
        valid = read_line_as_tokens (is, tokens) ;
    }
    
    // Skip the next comment line to prepare for the next stage
    bool valid2 = read_line_as_tokens (is, tokens) ;
    assert (valid2) ;
    assert (tokens.size() == 2) ;
    assert (tokens[0] == "input" && tokens[1] == "delays") ;
    
    return valid ;
}

// Read input delay
// Return value indicates if the last read was successful or not.
bool SdcParser::read_input_delay (string& portName, double& delay) {
    
    portName = "" ;
    delay = 0.0 ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens.size() >= 2) ;
    
    if (valid && tokens[0] == "set_input_delay") {
		// Workaround.. a better parser must be implemented later
		//assert (tokens.size() == 6) ;
        
        delay = std::atof(tokens[1].c_str()) ;
        
        assert (tokens[2] == "get_ports") ;
        
        portName = tokens[3];
		if( tokens.size() > 6 )
			portName += "[" + tokens[4] + "]";
        
		// Workaround.. a better parser must be implemented later
        //assert (tokens[4] == "-clock") ;
        
    } else {
        
        assert (tokens.size() == 2) ;
        assert (tokens[0] == "input" && tokens[1] == "drivers") ;
        
        return false ;
        
    }
    
    return valid ;
}


// Read output delay
// Return value indicates if the last read was successful or not.
bool SdcParser::read_output_delay (string& portName, double& delay) {
    
    portName = "" ;
    delay = 0.0 ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens.size() >= 2) ;
    
    if (valid && tokens[0] == "set_output_delay") {
		// Workaround.. a better parser must be implemented later
		//assert (tokens.size() == 6) ;
        
        delay = std::atof(tokens[1].c_str()) ;
        
        assert (tokens[2] == "get_ports") ;
        
        portName = tokens[3];
		if( tokens.size() > 6 )
			portName += "[" + tokens[4] + "]";
        
		// Workaround.. a better parser must be implemented later
        //assert (tokens[4] == "-clock") ;
        
    } else {
        
        assert (tokens.size() == 2) ;
        assert (tokens[0] == "output" && tokens[1] == "loads") ;
        
        return false ;
        
    }
    
    return valid ;
}


// Read driver info for the input port
// Return value indicates if the last read was successful or not.
bool SdcParser::read_driver_info (string& inPortName, string& driverSize, string& driverPin,
                                  double& inputTransitionFall, double& inputTransitionRise) {
    
    inPortName = "" ;
    driverSize = "" ;
    driverPin = "" ;
    inputTransitionFall = 0.0 ;
    inputTransitionRise = 0.0 ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens.size() >= 2) ;
    
    if (valid && tokens[0] == "set_driving_cell") {
		// Workaround, a better parser must be implemented in the future	
		assert( tokens.size() == 11 || tokens.size() == 12 );         
		if(tokens.size() == 11) {
		    assert (tokens[1] == "-lib_cell") ;
		    
		    driverSize = tokens[2] ;
		    
		    assert (tokens[3] == "-pin") ;
		    driverPin = tokens[4] ;
		    
		    assert (tokens[5] == "get_ports") ;
		    inPortName = tokens[6] ;
		    
		    assert (tokens[7] == "-input_transition_fall") ;
		    inputTransitionFall = std::atof(tokens[8].c_str()) ;
		    
		    assert (tokens[9] == "-input_transition_rise") ;
		    inputTransitionRise = std::atof(tokens[10].c_str()) ;
		}
		else if(tokens.size() != 11) {
        	assert (tokens[1] == "-lib_cell") ;
		    
		    driverSize = tokens[2] ;
		    
		    assert (tokens[3] == "-pin") ;
		    driverPin = tokens[4] ;
		    
		    assert (tokens[5] == "get_ports") ;
		    inPortName = tokens[6] + "[" + tokens[7] + "]";
		    
		    assert (tokens[8] == "-input_transition_fall") ;
		    inputTransitionFall = std::atof(tokens[9].c_str()) ;
		    
		    assert (tokens[10] == "-input_transition_rise") ;
		    inputTransitionRise = std::atof(tokens[11].c_str()) ;
		}
    } else {
        
        assert (tokens.size() == 2) ;
        assert (tokens[0] == "output" && tokens[1] == "delays") ;
        
        return false ;
    }
    
    return valid ;
}



// Read output load
// Return value indicates if the last read was successful or not.
bool SdcParser::read_output_load (string& outPortName, double& load) {
    
    outPortName = "" ;
    load = 0.0 ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    if (valid && tokens[0] == "set_load") {
	    // Workaround, a better parser must be implemented in the future	
		//assert (tokens.size() == 5) ;
        
        assert (tokens[1] == "-pin_load") ;
        load = std::atof(tokens[2].c_str()) ;
        
        assert (tokens[3] == "get_ports") ;
        outPortName = tokens[4] ;
        
		if( tokens.size() > 5 )
			outPortName += "[" + tokens[5] + "]";
		
    } else {
        
        assert (!valid) ;
        return false ;
    }
    
    return valid ;
}

// The return value indicates whether the *CONN section has been read or not
bool SpefParser::read_connections (vector<SpefConnection>& connections) {
    
    connections.clear() ; // in case the input is not empty
    bool terminateEarly = false ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
    
    // Skip the lines that are not "*CONN"
    while (valid && !(tokens.size() == 2 && tokens[0] == "*" && tokens[1] == "CONN")) {
        
        // The following if condition checks for nets without any connections
        // This is needed for clock nets.
        if (tokens.size() == 2 && tokens[0] == "*" && tokens[1] == "END") {
            terminateEarly = true ;
            break ;
        }
        
        valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
    }
    
    assert (valid) ; // end of file not expected here
    
    if (terminateEarly)
        return false ;
    
    while (valid) {
        valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
        
        if (tokens.size() == 2 && tokens[0] == "*" && tokens[1] == "CAP")
            break ; // the beginning of the next section
	
	if (tokens.size() == 2 && tokens[0] == "*" && tokens[1] == "END")
           return false ; // spef file for iccad 2015 contest
	
	//for(auto str: tokens)
	//	cout<<str<<endl;
        //cout<<"############################# "<<__func__<<" "<<__FILE__<<endl;
        // Line format: "*nodeType nodeName direction"
        // Note that nodeName can be either a single token or 3 tokens
        
        assert (tokens.size() == 4 || tokens.size() == 6) ;
        assert (tokens[0] == "*") ;
        
        SpefConnection curr ;
        int tokenIndex = 1 ;
        
        assert (tokens[tokenIndex].size() == 1) ; // should be a single character
        curr.nodeType = tokens[tokenIndex++][0] ;
        assert (curr.nodeType == 'P' || curr.nodeType == 'I') ;
        
        curr.nodeName.n1 = tokens[tokenIndex++] ;
        if (tokens[tokenIndex] == ":") {
            ++tokenIndex ; // skip the current token
            curr.nodeName.n2 = tokens[tokenIndex++] ;
        }
        
        assert (tokens[tokenIndex].size() == 1) ; // should be a single character
        curr.direction = tokens[tokenIndex++][0] ;
        assert (curr.direction == 'I' || curr.direction == 'O') ;
        
        connections.push_back(curr) ;
    }
    
    return true ;
}

void SpefParser::read_capacitances (vector<SpefCapacitance>& capacitances) {
    
    capacitances.clear() ; // in case the input is not empty
    
    vector<string> tokens ;
    bool valid = true ;
    while (valid) {
        
        valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
        
        if (tokens.size() == 2 && tokens[0] == "*" && tokens[1] == "RES")
            break ; // the beginning of the next section
        
        // Line format: "index nodeName cap"
        // Note that nodeName can be either a single token or 3 tokens
        
        assert (tokens.size() == 3 || tokens.size() == 5) ;
        
        SpefCapacitance curr ;
        int tokenIndex = 1 ;
        
        curr.nodeName.n1 = tokens[tokenIndex++] ;
        if (tokens[tokenIndex] == ":") {
            ++tokenIndex ; // skip the current token
            curr.nodeName.n2 = tokens[tokenIndex++] ;
        }
        
        curr.capacitance = std::atof(tokens[tokenIndex++].c_str()) ;
        assert (curr.capacitance >= 0) ;
        
        capacitances.push_back(curr) ;
    }
}

void SpefParser::read_resistances (vector<SpefResistance>& resistances) {
    
    resistances.clear() ; // in case the input is not empty
    
    vector<string> tokens ;
    bool valid = true ;
    while (valid) {
        
        valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
        
        if (tokens.size() == 2 && tokens[0] == "*" && tokens[1] == "END")
            break ; // end for this net
        
        // Line format: "index fromNodeName toNodeName res"
        // Note that each nodeName can be either a single token or 3 tokens
        
        assert (tokens.size() >= 4 && tokens.size() <= 8) ;
        
        SpefResistance curr ;
        int tokenIndex = 1 ;
        
        curr.fromNodeName.n1 = tokens[tokenIndex++] ;
        if (tokens[tokenIndex] == ":") {
            ++tokenIndex ; // skip the current token
            curr.fromNodeName.n2 = tokens[tokenIndex++] ;
        }
        
        curr.toNodeName.n1 = tokens[tokenIndex++] ;
        if (tokens[tokenIndex] == ":") {
            ++tokenIndex ; // skip the current token
            curr.toNodeName.n2 = tokens[tokenIndex++] ;
        }
        
        curr.resistance = std::atof(tokens[tokenIndex++].c_str()) ;
        assert (curr.resistance >= 0) ;
        
        resistances.push_back(curr) ;
    }
}

// Read the spef data for the next net.
// Return value indicates if the last read was successful or not.
bool SpefParser::read_net_data (SpefNet& spefNet) {
    
    spefNet.clear() ;
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
    
    // Read until a valid D_NET line is found
    while (valid) {
        if (tokens.size() == 4 && tokens[0] == "*" && tokens[1] == "D_NET") {
            spefNet.netName = tokens[2] ;
            spefNet.netLumpedCap = std::atof(tokens[3].c_str()) ;
            
            bool readConns = read_connections (spefNet.connections) ;

            if (readConns) {
                read_capacitances (spefNet.capacitances) ;
                read_resistances (spefNet.resistances) ;
            }
            
            return true ;
        } else {
			if ( tokens[0] == "*" && tokens[1] == "D_NET" )
				cout << "[BUG] @ SpefParser::read_net_data: possibly wrong-named net starting with '" << tokens[2] << "'\n";
		} // end else
        
        valid = read_line_as_tokens (is, tokens, true /*include special chars*/) ;
    }
    
    return false ; // a valid net was not read
}


// Read timing info for the next pin or port
// Return value indicates if the last read was successful or not.
// If the line read corresponds to a pin, then name1 and name2 will be set to the cell
// instance name and the pin name, respectively.
// If the line read corresponds to a port, then name1 will be set to the port name, and
// name2 will be set to "".
bool TimingParser::read_timing_line (string& name1, string& name2,
                                     double& riseSlack, double& fallSlack,
                                     double& riseTransition, double& fallTransition,
                                     double& riseArrival, double& fallArrival) {
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    if (!valid)
        return false ;
    
    assert (tokens.size() >= 7) ;
    
    int tokenIndex = 0 ;
    name1 = tokens[tokenIndex++] ;
    name2 = "" ;
    if (tokens.size() == 8) {
        // line corresponds to a cell pin
        name2 = tokens[tokenIndex++] ;
    }
    
    riseSlack = std::atof(tokens[tokenIndex++].c_str()) ;
    fallSlack = std::atof(tokens[tokenIndex++].c_str()) ;
    riseTransition = std::atof(tokens[tokenIndex++].c_str()) ;
    fallTransition = std::atof(tokens[tokenIndex++].c_str()) ;
    riseArrival = std::atof(tokens[tokenIndex++].c_str()) ;
    fallArrival = std::atof(tokens[tokenIndex++].c_str()) ;
    
    return true ;
    
}

bool TimingParser::read_timing_tau15 (	string & str1, string &str2, 
					double &val1, double &val2, double &val3, double &val4){
	
	str1 = "";
	str2 = "";
	val1 = 0;
	val2 = 0;
	val3 = 0;
	val4 = 0;
	
	vector<string> tokens ;
	bool valid = read_line_as_tokens (is, tokens) ;
	int tokenSize = tokens.size();
	
	if (!valid)
		return false ;
	
	 int tokenIndex = 0 ;
    str1 = tokens[tokenIndex++] ;
    str2 = tokens[tokenIndex++] ;

    val1 = std::atof(tokens[tokenIndex++].c_str()) ;
    if(tokenIndex < tokenSize)
	    val2 = std::atof(tokens[tokenIndex++].c_str()) ;
    if(tokenIndex < tokenSize)
	    val3 = std::atof(tokens[tokenIndex++].c_str()) ;
    if(tokenIndex < tokenSize)
	    val4 = std::atof(tokens[tokenIndex++].c_str()) ;
    
    return true;
}

bool OperationsParser::read_ops_tau15 (	ISPD13::CommandTypeEnum &cmdType, ISPD13::QueryTypeEnum &queryType, 
					string &pin, string &net, string &file, string &gate, string &cell, bool &early, bool &rise, int &paths ){
	
	pin = net = file = gate = cell = "";
	early = rise = true;
	paths = 1;
	cmdType = UNKNOWN;
	queryType = NOT_VALID;
	
	vector<string> tokens ;
	bool valid = read_line_as_tokens (is, tokens) ;
	int tokenSize = tokens.size();
	
	if (!valid)
		return false ;
	
	int tokenIndex = 0 ;
    string str1 = tokens[tokenIndex++];
		
	if ( str1 == "report_at" ) {
		queryType = REPORT_AT;
		assert( tokens[tokenIndex++] == "-pin" );
		pin = tokens[tokenIndex++];
		while ( tokenIndex < tokenSize ) {
			string tmp = tokens[tokenIndex++];
			if ( tmp == "-fall") {
				rise = false;
			} else if ( tmp == "-late" ) {
				early = false;
			}
		}
	} else if ( str1 == "report_rat" ) {
		queryType = REPORT_RAT;
		assert( tokens[tokenIndex++] == "-pin" );
		pin = tokens[tokenIndex++];
		while ( tokenIndex < tokenSize ) {
			string tmp = tokens[tokenIndex++];
			if ( tmp == "-fall") {
				rise = false;
			} else if ( tmp == "-late" ) {
				early = false;
			}
		}
	} else if ( str1 == "report_slack" ) {
		queryType = REPORT_SLACK;
		assert( tokens[tokenIndex++] == "-pin" );
		pin = tokens[tokenIndex++];
		while ( tokenIndex < tokenSize ) {
			string tmp = tokens[tokenIndex++];
			if ( tmp == "-fall") {
				rise = false;
			} else if ( tmp == "-late" ) {
				early = false;
			}
		}
	} else if ( str1 == "report_worst_paths" ) {
		queryType = REPORT_PATHS;
		while ( tokenIndex < tokenSize ) {
			assert ( tokenIndex + 1 < tokenSize );
			string tmp1 = tokens[tokenIndex++];
			string tmp2 = tokens[tokenIndex++];
			if ( tmp1 == "-pin" ) {
				//read pin char after ':' when it's internal pin
				if ( tokenIndex == tokenSize-1 || tokenIndex == tokenSize-3 ) {
					pin = tmp2 + ":" + tokens[tokenIndex++];
				} else
					pin = tmp2;
			} else if ( tmp1 == "-numPaths" ) {
				paths = (int) std::atof(tmp2.c_str());
			}
		}
	} else if ( str1 == "insert_gate") {
		
		cmdType = INSERT_GATE;
		gate = tokens[tokenIndex++];
		cell = tokens[tokenIndex++];
		
	} else if ( str1 == "repower_gate") {
		
		cmdType = REPOWER_GATE;
		gate = tokens[tokenIndex++];
		cell = tokens[tokenIndex++];
		
	} else if ( str1 == "remove_gate") {
		
		cmdType = REMOVE_GATE;
		gate = tokens[tokenIndex++];
		
	} else if ( str1 == "insert_net") {
		
		cmdType = INSERT_NET;
		net = tokens[tokenIndex++];
		
	} else if ( str1 == "remove_net") {
		
		cmdType = REMOVE_NET;
		net = tokens[tokenIndex++];
		
	} else if ( str1 == "read_spef") {
		
		cmdType = READ_SPEF;
		file = tokens[tokenIndex++];
		
	} else if ( str1 == "connect_pin") {
		
		cmdType = CONNECT_PIN;
		pin = tokens[tokenIndex++];
		net = tokens[tokenIndex++];
		
	} else if ( str1 == "disconnect_pin") {
		
		cmdType = DISCONNECT_PIN;
		pin = tokens[tokenIndex++];
		
	}
		
    
    return true;
}

// Read ceff values for the next pin or port
// Return value indicates if the last read was successful or not.
// If the line read corresponds to a pin, then name1 and name2 will be set to the cell
// instance name and the pin name, respectively.
// If the line read corresponds to a port, then name1 will be set to the port name, and
// name2 will be set to "".
bool CeffParser::read_ceff_line (string& name1, string& name2, double& riseCeff, double& fallCeff) {
    
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    if (!valid)
        return false ;
    
    assert (tokens.size() >= 3) ;
    
    int tokenIndex = 0 ;
    name1 = tokens[tokenIndex++] ;
    name2 = "" ;
    if (tokens.size() == 4) {
        // line corresponds to a cell pin
        name2 = tokens[tokenIndex++] ;
    }
    
    riseCeff = std::atof(tokens[tokenIndex++].c_str()) ;
    fallCeff = std::atof(tokens[tokenIndex++].c_str()) ;
    
    return true ;
}


// No need to parse the 3D LUTs, because they will be ignored
void LibParser::_skip_lut_3D () {
    
    std::vector<string> tokens ;
    
    bool valid = read_line_as_tokens (is, tokens) ;
    assert (valid) ;
    if ( tokens[0] == "values" ) return;
    assert (tokens[0] == "index_1") ;
    assert (tokens.size() >= 2) ;
    unsigned size1 = tokens.size() - 1 ;
    
    valid = read_line_as_tokens (is, tokens) ;
    assert (valid) ;
    assert (tokens[0] == "index_2") ;
    assert (tokens.size() >= 2) ;
    unsigned size2 = tokens.size() - 1 ;
    
    valid = read_line_as_tokens (is, tokens) ;
    assert (valid) ;
    assert (tokens[0] == "index_3") ;
    assert (tokens.size() >= 2) ;
    unsigned size3 = tokens.size() - 1 ;
    
    valid = read_line_as_tokens (is, tokens) ;
    assert (valid) ;
    assert (tokens.size() == 1 && tokens[0] == "values") ;
    
    for (unsigned i=0; i < size1; ++i) {
        for (unsigned j=0; j < size2; ++j) {
            
            valid = read_line_as_tokens (is, tokens) ;
            assert (valid) ;
            assert (tokens.size() == size3) ;
        }
    }
    
}

void LibParser::_begin_read_lut (LibParserLUT& lut) {
    
    std::vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens[0] == "index_1") ;
    assert (tokens.size() >= 2) ;
    
    unsigned size1 = tokens.size()-1 ;
    lut.loadIndices.resize(size1) ;
    for (unsigned i=0; i < tokens.size()-1; ++i) {
        
        lut.loadIndices[i] = std::atof(tokens[i+1].c_str()) ;
    }
    
    valid = read_line_as_tokens (is, tokens) ;
    
    assert (valid) ;
    assert (tokens[0] == "index_2") ;
    assert (tokens.size() >= 2) ;
    
    int size2 = tokens.size()-1 ;
    lut.transitionIndices.resize(size2) ;
    for (unsigned i=0; i < tokens.size()-1; ++i) {
        
        lut.transitionIndices[i] = std::atof(tokens[i+1].c_str()) ;
    }
    
    valid = read_line_as_tokens (is, tokens) ;
    assert (valid) ;
    assert (tokens.size() == 1 && tokens[0] == "values") ;
    
    lut.tableVals.initialize(size1, size2) ;
    for (unsigned i=0 ; i < lut.loadIndices.size(); ++i) {
        valid = read_line_as_tokens (is, tokens) ;
        assert (valid) ;
        assert (tokens.size() == lut.transitionIndices.size()) ;
        
        for (unsigned j=0; j < lut.transitionIndices.size(); ++j) {
            lut.tableVals(i, j) = std::atof(tokens[j].c_str()) ;
            
        }
    }
    
    
}

void LibParser::_begin_read_timing_info (string toPin, LibParserPinInfo& pin, LibParserTimingInfo& timing) {
    
    timing.toPin = toPin ;
    
    bool finishedReading = false ;
	bool setup = false;
    
    std::vector<string> tokens ;
    while (!finishedReading) {
        
        bool valid = read_line_as_tokens (is, tokens) ;
        assert (valid) ;
        assert (tokens.size() >= 1) ;
        
        if (tokens[0] == "cell_fall") {
            _begin_read_lut (timing.fallDelay) ;
            
        } else if (tokens[0] == "cell_rise") {
            _begin_read_lut (timing.riseDelay) ;
            
        } else if (tokens[0] == "fall_transition") {
            _begin_read_lut (timing.fallTransition) ;
            
        } else if (tokens[0] == "rise_transition") {
            _begin_read_lut (timing.riseTransition) ;
            
        } else if (tokens[0] == "fall_constraint") {
            
            //_skip_lut_3D() ; // will ignore fall constraints
            bool valid = read_line_as_tokens (is, tokens) ;
            //assert (valid) ;
            //assert ( tokens[0] == "values" );
			if ( setup )
				_begin_read_lut (pin.fallSetup) ;
				//pin.fallSetup = std::atof(tokens[1].c_str());
			else
				_begin_read_lut (pin.fallHold) ;
				//pin.fallHold = std::atof(tokens[1].c_str());
			pin.isTimingEndpoint = true;
			//cout << "setup/hold: " << std::atof(tokens[1].c_str()) << endl;
        		
        } else if (tokens[0] == "rise_constraint") {
            
           // _skip_lut_3D() ; // will ignore rise constraints
            bool valid = read_line_as_tokens (is, tokens) ;
            assert (valid) ;
            assert ( tokens[0] == "values" );
			if ( setup )
				_begin_read_lut (pin.riseSetup) ;
				//pin.riseSetup = std::atof(tokens[1].c_str());
			else
				_begin_read_lut (pin.riseHold) ;
				//pin.riseHold = std::atof(tokens[1].c_str());
			//cout << pin.name << " \tsetup/hold: " << std::atof(tokens[1].c_str()) << endl;
            
        } else if (tokens[0] == "timing_sense") {
            timing.timingSense = tokens[1] ;
            
        } else if (tokens[0] == "related_pin") {
            
            assert (tokens.size() == 2) ;
            timing.fromPin = tokens[1] ;
            
        } else if (tokens[0] == "End") {
            
            assert (tokens.size() == 2) ;
            assert (tokens[1] == "timing") ;
            finishedReading = true ;
            
        } else if (tokens[0] == "timing_type") {
			if ( tokens[1] == "setup_rising") 
				setup = true;
			else setup = false;
			
        } else if (tokens[0] == "related_output_pin") {
            // ignore data
            
        } else {
            
            cout << "Error: Unknown keyword: " << tokens[0] << endl ;
            assert (false) ; // unknown keyword
        }
        
    }
    
    
}


void LibParser::_begin_read_pin_info (string pinName, LibParserCellInfo& cell, LibParserPinInfo& pin) {
    
    pin.name = pinName ;
    pin.isClock = false ;
    pin.maxCapacitance = std::numeric_limits<double>::max() ;
	//cout << "\tpin: " << pinName << endl;
    
    bool finishedReading = false ;
    
    std::vector<string> tokens ;
    while (!finishedReading) {
        
        bool valid = read_line_as_tokens (is, tokens) ;
        assert (valid) ;
        assert (tokens.size() >= 1) ;
		//cout << tokens[0] << "\t" << tokens[1] << endl;
        
        if (tokens[0] == "direction") {
            
            assert (tokens.size() == 2) ;
            if (tokens[1] == "input")
                pin.isInput = true ;
            else if (tokens[1] == "output")
                pin.isInput = false ;
            else
                assert (false) ; // undefined direction
            
        } else if (tokens[0] == "capacitance") {
            
            assert (tokens.size() == 2) ;
            pin.capacitance = std::atof(tokens[1].c_str()) ;
            
        } else if (tokens[0] == "max_capacitance") {
            
            assert (tokens.size() == 2) ;
            pin.maxCapacitance = std::atof(tokens[1].c_str()) ;
            
            
        } else if (tokens[0] == "timing") {
            
            cell.timingArcs.push_back(LibParserTimingInfo()) ; // add an empty TimingInfo object
            _begin_read_timing_info (pinName, pin, cell.timingArcs.back()) ; // pass the empty object to the function to be filled
            
        } else if (tokens[0] == "clock") {
            
            pin.isClock = true ;
			cell.isSequential = true;
            
        } else if (tokens[0] == "End") {
            
            assert (tokens.size() == 2) ;
            assert (tokens[1] == "pin") ;
            finishedReading = true ;
            
        } else if (tokens[0] == "function") {
            
            // ignore data
            
        } else if (tokens[0] == "min_capacitance") {
            
            // ignore data
            
        } else if (tokens[0] == "nextstate_type") {
            
            // ignore data
            
        } else {
            cout << "Error: Unknown keyword: " << tokens[0] << endl ;
            assert (false) ; // unknown keyword
            
        }
        
    }
    
    
}

void LibParser::_begin_read_cell_info (string cellName, LibParserCellInfo& cell) {
    
    cell.name = cellName ;
    cell.isSequential = false ;
    cell.dontTouch = false ;
    
    bool finishedReading = false ;
    //cout << "Cell: " << cellName << endl;
		
    std::vector<string> tokens ;
    while (!finishedReading) {
        
        bool valid = read_line_as_tokens (is, tokens) ;
		//cout << "size: " << tokens.size() << endl;
        assert (valid) ;
        assert (tokens.size() >= 1) ;
		
		//cout << tokens[0] << endl;
        if (tokens[0] == "cell_leakage_power") {
            
            assert (tokens.size() == 2) ;
            cell.leakagePower = std::atof(tokens[1].c_str()) ;
            
        } else if (tokens[0] == "cell_footprint") {
            
            assert (tokens.size() == 2) ;
            cell.footprint = tokens[1] ;
            
        } else if (tokens[0] == "area") {
            
            assert (tokens.size() == 2) ;
            cell.area = std::atof(tokens[1].c_str()) ;
            
        } else if (tokens[0] == "clocked_on") {
            
            cell.isSequential = true ;
            
        } else if (tokens[0] == "dont_touch") {
            
            cell.dontTouch = true ;
            
        } else if (tokens[0] == "pin") {
            
			assert (tokens.size() == 2) ;
			
            cell.pins.push_back(LibParserPinInfo()) ; // add empty PinInfo object
            _begin_read_pin_info (tokens[1], cell, cell.pins.back()) ; // pass the new PinInfo object to be filled
            
        } else if (tokens[0] == "End") {
            
            assert (tokens.size() == 3) ;
            assert (tokens[1] == "cell") ;
            assert (tokens[2] == cellName) ;
            finishedReading = true ;
            
        } else if (tokens[0] == "cell_footprint") {
            
            // ignore data
            
        } else if (tokens[0] == "ff") {
            
            // ignore data
            
        } else if (tokens[0] == "next_state") {
            
            // ignore data
            
        } else if (tokens[0] == "dont_use") {
            
            // ignore data
            
        } else {
            
            cout << "Error: Unknown keyword: " << tokens[0] << endl ;
            assert (false) ; // unknown keyword
        }
    }
    
}


// Read the default max_transition defined for the library.
// Return value indicates if the last read was successful or not.
// This function must be called in the beginning before any read_cell_info function call.
bool LibParser::read_default_max_transition (double& maxTransition) {
    
    maxTransition = 0.0 ;
    vector<string> tokens ;
    
    bool valid = read_line_as_tokens (is, tokens) ;
    
    while (valid) {
        
        if (tokens.size() == 2 && tokens[0] == "default_max_transition") {
            maxTransition = std::atof(tokens[1].c_str()) ;
            return true ;
        }
        
        valid = read_line_as_tokens (is, tokens) ;
    }
    
    return false ;
}



// Read the next standard cell definition.
// Return value indicates if the last read was successful or not.
bool LibParser::read_cell_info (LibParserCellInfo& cell) {
    
    vector<string> tokens ;
    bool valid = read_line_as_tokens (is, tokens) ;
    
    
    while (valid) {
        
        if (tokens.size() == 2 && tokens[0] == "cell") {
            _begin_read_cell_info (tokens[1], cell) ;
            
            return true ;
        }
        
        valid = read_line_as_tokens (is, tokens) ;
    }
    
    return false ;
}

ostream& operator<< (ostream& os, LibParserLUT& lut) {
    
    if (lut.loadIndices.empty() && lut.transitionIndices.empty() && lut.tableVals.isEmpty())
        return os ;
    
    // We should have either all empty or none empty.
    assert (!lut.loadIndices.empty() && !lut.transitionIndices.empty() && !lut.tableVals.isEmpty()) ;
    
    assert (lut.tableVals.getNumElements() == lut.loadIndices.size()) ;
    assert (lut.tableVals.getNumRows() == lut.transitionIndices.size()) ;
    
    cout << "\t" ;
    for (unsigned i=0; i < lut.transitionIndices.size(); ++i) {
        cout << lut.transitionIndices[i] << "\t" ;
    }
    cout << endl ;
    
    
    for (unsigned i=0; i < lut.loadIndices.size(); ++i) {
        cout << lut.loadIndices[i] << "\t" ;
        
        for (unsigned j=0; j < lut.transitionIndices.size(); ++j)
            cout << lut.tableVals(i, j) << "\t" ;
        
        cout << endl ;
        
    }
    
    return os ;
}


ostream& operator<< (ostream& os, LibParserTimingInfo& timing) {
    
    cout << "Timing info from " << timing.fromPin << " to " << timing.toPin << ": " << endl ;
    cout << "Timing sense: " << timing.timingSense << endl ;
    
    cout << "Fall delay LUT: " << endl ;
    cout << timing.fallDelay ;
    
    cout << "Rise delay LUT: " << endl ;
    cout << timing.riseDelay ;
    
    cout << "Fall transition LUT: " << endl ;
    cout << timing.fallTransition ;
    
    cout << "Rise transition LUT: " << endl ;
    cout << timing.riseTransition ;
    
    return os ;
}


ostream& operator<< (ostream& os, LibParserPinInfo& pin) {
    
    cout << "Pin " << pin.name << ":" << endl ;
    cout << "capacitance: " << pin.capacitance << endl ;
    cout << "maxCapacitance: " << pin.maxCapacitance << endl ;
    cout << "isInput? " << (pin.isInput ? "true" : "false") << endl ;
    cout << "isClock? " << (pin.isClock ? "true" : "false") << endl ;
    cout << "End pin" << endl ;
    
    return os ;
}


ostream& operator<< (ostream& os, LibParserCellInfo& cell) {
    
    cout << "Library cell " << cell.name << ": " << endl ;
    
    cout << "Footprint: " << cell.footprint << endl ;
    cout << "Leakage power: " << cell.leakagePower << endl ;
    cout << "Area: " << cell.area << endl ;
    cout << "Sequential? " << (cell.isSequential ? "yes" : "no") << endl ;
    cout << "Dont-touch? " << (cell.dontTouch ? "yes" : "no") << endl ;
    
    cout << "Cell has " << cell.pins.size() << " pins: " << endl ;
    for (unsigned i=0; i < cell.pins.size(); ++i) {
        cout << cell.pins[i] << endl ;
    }
    
    cout << "Cell has " << cell.timingArcs.size() << " timing arcs: " << endl ;
    for (unsigned i=0; i < cell.timingArcs.size(); ++i) {
        cout << cell.timingArcs[i] << endl ;
    }
    
    cout << "End of cell " << cell.name << endl << endl ;
    
    return os ;
}



// Example function that uses VerilogParser class to parse the given ISPD-13 verilog
// file. The extracted data is simply printed out in this example.
void test_verilog_parser (string filename) {
    
    VerilogParser vp (filename) ;
    
    string moduleName ;
    bool valid = vp.read_module (moduleName) ;
    assert (valid) ;
    
    cout << "Module " << moduleName << endl << endl ;
    
    do {
        string primaryInput ;
        valid = vp.read_primary_input (primaryInput) ;
        
        if (valid)
            cout << "Primary input: " << primaryInput << endl ;
        
    } while (valid) ;
    
    cout << endl ;
    
    do {
        string primaryOutput ;
        valid = vp.read_primary_output (primaryOutput) ;
        
        if (valid)
            cout << "Primary output: " << primaryOutput << endl ;
        
    } while (valid) ;
    
    cout << endl ;
    
    do {
        string net ;
        valid = vp.read_wire (net) ;
        
        if (valid)
            cout << "Net: " << net << endl ;
        
    } while (valid) ;
    
    
    cout << endl ;
    cout << "Cell insts: " << std::endl ;
    
    do {
        string cellType, cellInst ;
        vector<std::pair<string, string> > pinNetPairs ;
        
        valid = vp.read_cell_inst (cellType, cellInst, pinNetPairs) ;
        
        if (valid) {
            cout << cellType << " " << cellInst << " " ;
            for (unsigned i=0; i < pinNetPairs.size(); ++i) {
                cout << "(" << pinNetPairs[i].first << " " << pinNetPairs[i].second << ") " ;
            }
            
            cout << endl ;
        }
        
    } while (valid) ;
    
    
}

// Example function that uses SdcParser class to parse the given ISPD-13 sdc
// file. The extracted data is simply printed out in this example.
void test_sdc_parser (string filename) {
    
    SdcParser sp (filename) ;
    
    string clockName ;
    string clockPort ;
    double period ;
    bool valid = sp.read_clock (clockName, clockPort, period) ;
    
    assert(valid) ;
    cout << "Clock " << clockName << " connected to port " << clockPort
    << " has period " << period << endl ;
    
    do {
        string portName ;
        double delay ;
        
        valid = sp.read_input_delay (portName, delay) ;
        
        if (valid)
            cout << "Input port " << portName << " has delay " << delay << endl ;
        
    } while (valid) ;
    
    
    do {
        string portName ;
        string driverSize ;
        string driverPin ;
        double inputTransitionFall ;
        double inputTransitionRise ;
        
        valid = sp.read_driver_info (portName, driverSize, driverPin,
                                     inputTransitionFall, inputTransitionRise) ;
        
        if (valid) {
            cout << "Input port " << portName << " is assumed to be connected to the "
            << driverPin << " pin of lib cell " << driverSize << endl ;
            cout << "This virtual driver is assumed to have input transitions: "
            << inputTransitionFall << " (fall) and " << inputTransitionRise
            << " (rise)" << endl ;
        }
        
        
    } while (valid) ;
    
    do {
        string portName ;
        double delay ;
        
        valid = sp.read_output_delay (portName, delay) ;
        
        if (valid)
            cout << "Output port " << portName << " has delay " << delay << endl ;
        
    } while (valid) ;
    
    
    do {
        string portName ;
        double load ;
        
        valid = sp.read_output_load (portName, load) ;
        
        if (valid)
            cout << "Output port " << portName << " has load " << load << endl ;
        
    } while (valid) ;
    
    
}

// Example function that uses SpefParser class to parse the given ISPD-13 spef
// file. The extracted data is simply printed out in this example.
void test_spef_parser (string filename) {
    
    SpefParser sp (filename) ;
    
    SpefNet spefNet ;
    bool valid = sp.read_net_data (spefNet) ;
    
    int readCnt = 0 ;
    while (valid) {
        
        ++readCnt ;
        
        // print out the contents of the spefNet just read
        cout << "Net: " << spefNet.netName << endl ;
        cout << "Net lumped cap: " << spefNet.netLumpedCap << endl ;
        
        cout << "Connections: " << endl ;
        for (int i=0; i < spefNet.connections.size(); ++i) {
            cout << spefNet.connections[i] << endl ;
        }
        
        cout << "Capacitances: " << endl ;
        for (int i=0; i < spefNet.capacitances.size(); ++i) {
            cout << spefNet.capacitances[i] << endl ;
        }
        
        cout << "Resistances: " << endl ;
        for (int i=0; i < spefNet.resistances.size(); ++i) {
            cout << spefNet.resistances[i] << endl ;
        }
        
        cout << endl ;
        valid = sp.read_net_data (spefNet) ;
    }
    
    cout << "Read " << readCnt << " nets in the spef file." << endl ;
}

// Example function that uses TimingParser class to parse the given ISPD-13 timing
// file. The extracted data is simply printed out in this example.
void test_timing_parser (string filename) {
    
    TimingParser tp (filename) ;
    
    bool valid = false ;
    while (true) {
        
        string name1, name2 ;
        double riseSlack, fallSlack, riseTransition, fallTransition, riseArrival, fallArrival ;
        
        valid = tp.read_timing_line (name1, name2, riseSlack, fallSlack,
                                     riseTransition, fallTransition, riseArrival, fallArrival) ;
        
        if (!valid)
            break ;
        
        if (name2 != "") {
            // timing info of a pin
            // name1: cellInstance, name2: pin
            std::cout << name1 << "/" << name2 << " "
            << riseSlack << " " << fallSlack << " "
            << riseTransition << " " << fallTransition << " "
            << riseArrival << " " << fallArrival << endl ;
            
        } else {
            // timing of a port
            // name1: port name
            std::cout << name1 << " "
            << riseSlack << " " << fallSlack << " "
            << riseTransition << " " << fallTransition << " "
            << riseArrival << " " << fallArrival << endl ;
        }
    }
    
}


// Example function that uses CeffParser class to parse the given ISPD-13 ceff
// file. The extracted data is simply printed out in this example.
void test_ceff_parser (string filename) {
    
    CeffParser tp (filename) ;
    
    bool valid = false ;
    while (true) {
        
        string name1, name2 ;
        double riseCeff, fallCeff ;
        
        valid = tp.read_ceff_line (name1, name2, riseCeff, fallCeff) ;
        
        if (!valid)
            break ;
        
        if (name2 != "") {
            // ceff values of a pin
            // name1: cellInstance, name2: pin
            std::cout << name1 << "/" << name2 << " "
            << riseCeff << " " << fallCeff << endl ;
            
        } else {
            // timing of a port
            // name1: port name
            std::cout << name1 << " "
            << riseCeff << " " << fallCeff << endl ;
        }
    }
    
}



// Example function that uses LibParser class to parse the given ISPD-13 lib
// file. The extracted data is simply printed out in this example.
void test_lib_parser (string filename) {
    
    LibParser lp (filename) ;
    
    double maxTransition = 0.0 ;
    bool valid = lp.read_default_max_transition(maxTransition) ;
    
    assert (valid) ;
    cout << "The default max transition defined is " << maxTransition << endl ;
    
    int readCnt = 0 ;
    do {
        LibParserCellInfo cell ;
        valid = lp.read_cell_info (cell) ;
        
        if (valid) {
            ++readCnt ;
            
            cout << cell << endl ;
        }
        
    } while (valid) ;
    
    cout << "Read " << readCnt << " number of library cells" << endl ;
}

ostream& operator<< (ostream& os, const SpefNodeName& n) {
    os << n.n1 << ((n.n2 != "") ? ":" : "") << n.n2 ;
    
    return os ;
}

ostream& operator<< (ostream& os, const SpefConnection& c) {
    os << c.nodeType << " " << c.nodeName << " " << c.direction ;
    return os ;
};

ostream& operator<< (ostream& os, const SpefCapacitance& c) {
    os << c.nodeName << " " << c.capacitance ;
    return os ;
}

ostream& operator<< (ostream& os, const SpefResistance& r) {
    os << r.fromNodeName << " " << r.toNodeName << " " << r.resistance ;
    return os ;
}



bool OPSParser::read_operation (OPSInfo &ops) {

	vector<string> tokens ;
	bool valid = read_line_as_tokens (is, tokens) ;
	int tokenSize = tokens.size();
	
	if (!valid)
		return false ;	
	string opt = tokens[tokenSize++];
	ops.opsType = opt;
	
	if(opt == "report_at"){
		ops.pin = tokens[++tokenSize];
		//if()
		
	}
	//TODO
	return true;
}

} // end namespace
