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
 
/* 
 * File:   AsciiProgressBar.h
 * Author: gaflach
 *
 * Created on September 6, 2015, 1:11 AM
 */

#ifndef ASCII_PROGRESS_BAR_H
#define	ASCII_PROGRESS_BAR_H

#include <algorithm>
#include <sstream>
#include <iomanip>

static const int DEFAULT_N = 40;

class AsciiProgressBar {
private:
	const int N;
	
	int clsMaxValue;
	int clsCurrentValue;
	int clsDrawingThreshold;
	int clsNextDraw;
	
	void updateDrawingThreshold() {
		const double precision = 1.0 / N;
		clsDrawingThreshold = (int) (precision * clsMaxValue);
	} // end method
	
	void draw() {
		const int percentage = clsMaxValue > 0? (100*clsCurrentValue)/clsMaxValue : 100;
		const int middle = (N*percentage) / 100;
		
		std::cout << "\r";
		std::cout << std::setw(3) << percentage << "% ";
		std::cout << "[";
		for (int i = 0; i < middle; i++) {
			std::cout << "=";
		} // end for
		for (int i = middle; i < N; i++) {
			std::cout << " ";
		} // end for
		std::cout << "]";
		std::cout << std::flush;
	} // end method
	
	void reset(const int maxValue = 0) {
		clsMaxValue = maxValue;
		clsCurrentValue = 0;
		updateDrawingThreshold();
		clsNextDraw = clsDrawingThreshold;
	} // end method
	
public:
	
	AsciiProgressBar() : N(DEFAULT_N) {
		reset();
	} // end constructor
			 
	AsciiProgressBar(const int maxValue, const int numColumns = DEFAULT_N) : N(numColumns) {
		reset(maxValue);
	} // end constructor
	
	void setMaxValue(const int maxValue) { 
		clsMaxValue = maxValue; 
		updateDrawingThreshold();
	} // end method
	
	void setCurrentValue(const int value) { 
		clsCurrentValue = std::min(value, clsMaxValue); 
	} // end method
	
	void progress(const int increment = 1) { 
		clsCurrentValue = std::min(clsCurrentValue + increment, clsMaxValue);
		if (clsNextDraw >= clsDrawingThreshold) {
			draw();
			clsNextDraw = 0;
		} else {
			clsNextDraw += increment;
		} // end else
	} // end method
	
	void finish() {
		clsCurrentValue = clsMaxValue;
		clsNextDraw = 0;
		draw();
		std::cout << "\n";
	} // end method
	
}; // end class

#endif	/* ASCIIPROGRESSBAR_H */

