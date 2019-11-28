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
 
#ifndef COLORIZE_H
#define	COLORIZE_H

#include <cmath>
#include <array>

// Diverging Color Maps for Scientific Visualization
// http://www.kennethmoreland.com/color-maps/ColorMapsExpanded.pdf

class Colorize {
public:

	static void colorTemperature( const double weight, int &r, int &g, int &b ) {
		// Mangeta <-> Blue <-> Cian <-> Green <-> Yellow <-> Red
		const int numColors = 6;
		static const int R[numColors] = { 255,   0,   0,   0, 255, 255 };
		static const int G[numColors] = {   0,   0, 255, 255, 255,   0 };
		static const int B[numColors] = { 255, 255, 255,   0,   0,   0 };
		
		/* Color Range based on weight
		 * 1.0 -> Vermelho - Red
		 * 0.9 -> Laranja - Orange
		 * 0.8 -> Amarelo - Yellow
		 * 0.7 -> Verde Claro - Light Green
		 * 0.6 -> Verde Harlequin - Harlequin green
		 * 0.5 -> Verde Spring  - Spring Green
		 * 0.4 -> Azul Ciano - Ciano Blue
		 * 0.3 -> Azul Azure - Azure Blue
		 * 0.2 -> Azul - Blue
		 * 0.1 -> Azul Indigo - Indigo Blue
		 * 0.0 -> Rosa - Pink 
		 */

//		// Baseado no mapa de cores "jet" do Matlab
//		const int numColors = 16;
//		static const int R[numColors] = {   0,   0,   0,   0,   0,   0,  66, 132, 189, 255, 255, 255, 255, 255, 189, 132};
//		static const int G[numColors] = {   0,   0,  66, 132, 189, 255, 255, 255, 255, 255, 189, 132,  66,   0,   0,   0};
//		static const int B[numColors] = { 189, 255, 255, 255, 255, 255, 189, 132,  66,   0,   0,   0,   0,   0,   0,   0};		
		
		double temp;
		
		if ( std::isnan(weight) ) {
			r = g = b = 0;
		} else if ( std::isinf(weight) ) {
			r = g = b = 255;
		} else {
			if ( weight > 1.0 )
				temp = (numColors-1);
			else if ( weight < 0 )
				temp = 0;
			else
				temp = (numColors-1)*weight;

			const int i0 = (int) std::floor( temp );
			const int i1 = (int) std::ceil( temp );
			const double alpha = temp - (int)(temp);

			r = (int) std::floor( R[i0]*(1-alpha) + R[i1]*(alpha) + 0.5 );
			g = (int) std::floor( G[i0]*(1-alpha) + G[i1]*(alpha) + 0.5 );
			b = (int) std::floor( B[i0]*(1-alpha) + B[i1]*(alpha) + 0.5 );
		} // end else
	} // end method
	
	
	static void colorTemperatureWarmCold( const double weight, int &r, int &g, int &b ) {
		static const std::array<int, 33> R{59, 68, 77, 87, 98, 108, 119, 130, 141, 152, 163, 174, 184, 194, 204, 213, 221, 229, 236, 241, 245, 247, 247, 247, 244, 241, 236, 229, 222, 213, 203, 192, 180}; 
		static const std::array<int, 33> G{76, 90, 104, 117, 130, 142, 154, 165, 176, 185, 194, 201, 208, 213, 217, 219, 221, 216, 211, 204, 196, 187, 177, 166, 154, 141, 127, 112, 96, 80, 62, 40, 4};
		static const std::array<int, 33> B{192, 204, 215, 225, 234, 241, 247, 251, 254, 255, 255, 253, 249, 244, 238, 230, 221, 209, 197, 185, 173, 160, 148, 135, 123, 111, 99, 88, 77, 66, 56, 47, 38}; 

		const int numColors = R.size();
		
		double temp;
		
		if ( std::isnan(weight) ) {
			r = g = b = 0;
		} else if ( std::isinf(weight) ) {
			r = g = b = 255;
		} else {
			if ( weight > 1.0 )
				temp = (numColors-1);
			else if ( weight < 0 )
				temp = 0;
			else
				temp = (numColors-1)*weight;

			const int i0 = (int) std::floor( temp );
			const int i1 = (int) std::ceil( temp );
			const double alpha = temp - (int)(temp);

			r = (int) std::floor( R[i0]*(1-alpha) + R[i1]*(alpha) + 0.5 );
			g = (int) std::floor( G[i0]*(1-alpha) + G[i1]*(alpha) + 0.5 );
			b = (int) std::floor( B[i0]*(1-alpha) + B[i1]*(alpha) + 0.5 );
		} // end else
	} // end method	
	
	
};

#endif

