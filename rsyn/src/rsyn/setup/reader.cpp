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
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <Rsyn/Session>

// Readers
#include "rsyn/io/reader/ISPD2018Reader.h"

// Registration
namespace Rsyn {

void Session::registerReaders() {
	registerReader<Rsyn::ISPD2018Reader>("ispd18");
	registerReader<Rsyn::ISPD2018Reader>("ispd19"); // ispd19 files are the same from ispd18 contest
} // end method
} // end namespace
