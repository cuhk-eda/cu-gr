CUGR
======================================
CUGR is a VLSI global routing tool developed by the research team supervised by Prof. Evangeline F. Y. Young at The Chinese University of Hong Kong (CUHK).
Different from previous global routers whose quality is usually measured by wirelength and resource overflow,
CUGR is a detailed routability-driven global router and its solution quality is solely determined by the final detailed routing results.
In particular, our global router adopts several efficient and effective methods to generate a set of connected rectangles to guide the detailed router:
* A sophisticated probability-based cost scheme
* An optimal 3D pattern routing technique that combines 2D pattern routing and layer assignment
* A multi-level maze routing utilizes two levels of routing
* A patching technique that adds useful route guides to further improve the detailed routability.
* ...

More details are in the following paper:

* Jinwei Liu, Chak-Wa Pui, Fangzhou Wang, Evangeline F. Y. Young,
["CUGR: Detailed-Routability-Driven 3D Global Routing with Probabilistic Resource Model"](https://ieeexplore.ieee.org/document/9218646),
ACM/IEEE Design Automation Conference (DAC), San Francisco, CA, USA, July 19-23, 2020.

(CUGR supports ICCAD'19 benchmarks ([v2](http://iccad-contest.org/2019/Problem_C/iccad19_benchmarks_v2.tar.gz), [hidden](http://iccad-contest.org/2019/Problem_C/iccad19_hidden_benchmarks.tar.gz)).
This version of code is consistent with the DAC paper.)

## 1. How to Build

**Step 1:** Download the source code. For example,
```bash
$ git clone https://github.com/cuhk-eda/cu-gr
```

**Step 2:** Go to the project root and build by
```bash
$ cd cu-gr
$ scripts/build.py -o release
```

Note that this will generate two folders under the root, `build` and `run` (`build` contains intermediate files for build/compilation, while `run` contains binaries and auxiliary files).
More details are in [`scripts/build.py`](scripts/build.py).

### 1.1. Dependencies

* [GCC](https://gcc.gnu.org/) (version >= 5.5.0) or other working c++ compliers
* [CMake](https://cmake.org/) (version >= 2.8)
* [Boost](https://www.boost.org/) (version >= 1.58)
* [Python](https://www.python.org/) (version 3, optional, for utility scripts)
* [Innovus®](https://www.cadence.com/content/cadence-www/global/en_US/home/tools/digital-design-and-signoff/soc-implementation-and-floorplanning/innovus-implementation-system.html) (version 18.1, optional, for design rule checking and evaluation)
* [Rsyn](https://github.com/RsynTeam/rsyn-x) (a trimmed version is used, already added under folder `rsyn`)
* [Dr. CU](https://github.com/cuhk-eda/dr-cu) (v4.1.1, optional, official detailed router for ICCAD'19 Contest, [binary](http://iccad-contest.org/2019/Problem_C/drcu_june19.zip) is already included under the root)

## 2. How to Run

### 2.1. Toy Test

#### Run Binary Directly

Go to the `run` directory and run the binary `iccad19gr` with a toy case `ispd18_sample`:
```bash
$ cd run
$ ./iccad19gr -lef ../toys/iccad2019c/ispd18_sample/ispd18_sample.input.lef -def ../toys/iccad2019c/ispd18_sample/ispd18_sample.input.def -output ispd18_sample.solution.guide -threads 8
```

#### Run with a Wrapping Script

Instead of running the binary directly, you may also use a wrapping script `run.py` to save typing and do more:
```bash
$ cd run
$ ./run.py 8s -p ../toys/
```

If Innovus® has been properly installed in your OS, an evaluation can be launched by
```bash
$ ./run.py 8s -s eval -p ../toys/
```
In the end, a result table will be printed in the terminal.

Furthermore, the solution can be visualized by
```bash
$ ./run.py 8s -s view -p ../toys/
```
which gives:

![ispd18_sample.solution.png](/toys/iccad2019c/ispd18_sample/ispd18_sample.solution.png)

The three steps, `route`, `eval` and `view` of `run.py` can also be invoked in a single line:
```bash
$ ./run.py 8s -s route eval view -p ../toys/
```
More usage about `run.py` can be known by the option `-h`.

### 2.2. Batch Test

The benchmarks can be downloaded from [the hompage of ISPD'18 Contest ](http://www.ispd.cc/contests/18/#benchmarks).
You may let `run.py` know the benchmark path by setting OS environmental variable `BENCHMARK_PATH` or specifying it under option `-p`.
Then,
```bash
$ cd run
$ ./run.py <benchmark_name...|all> -s route eval [option...]
```

## 3. Modules

* `ispd18eval`: scripts and other files for evaluation, provided by [ICCAD'19 Contest](http://iccad-contest.org/2019/Problem_C/eval.zip)
* `ispd19eval`: scripts and other files for evaluation, provided by [ICCAD'19 Contest](http://iccad-contest.org/2019/Problem_C/eval.zip)
* `rsyn`: code from [Rsyn](https://github.com/RsynTeam/rsyn-x) for file IO
* `scripts`: utility python scripts
* `src`: C++ source code
    * `db`: database, including the global grid graph and the net information
    * `single_net`: routing a single net, including querying the global grid graph, building the local grid graph, running maze routing, and some post processing
    * `multi_net`: routing all nets with "rip-up and rereoute" and multithreading
    * `utils`: some utility code
* `toys`: toy test cases
* `drcu`: default detailed router for evaluation


## 4. Results

Experiments are performed on a 64-bit Linux workstation with Intel Xeon Silver 4114 CPU (2.20GHz, 40 cores) and 256GB memory.
Consistent with the contest, eight threads are used.

|   Design  | Wire Length  & Via | Non-Preferred  Usage |   Short  | Min-Area  & Spacing | DR Score | GR  Runtime (sec) |
|:---------:|:------------------:|:--------------------:|:--------:|:-------------------:|:--------:|:---------------:|
|  `ispd2018_test5`  |      15613663      |        166994        |  330425  |        288500       | 16089196 |        68       |
|  `ispd2018_test5_metal5` |      15807997      |        135293        |  261150  |        224000       | 16210303 |        85       |
|  `ispd2018_test8`  |      37441058      |        269993        |  209470  |        144000       | 37908815 |       236       |
|  `ispd2018_test8_metal5` |      36746610      |        336768        |  194510  |        129500       | 37293962 |       300       |
|  `ispd2018_test10` |      39061258      |        882371        |  669965  |        471000       | 40600501 |       334       |
| `ispd2018_test10_metal5` |      40246090      |        1413120       |  4021620 |        685500       | 46300610 |       373       |
|  `ispd2019_test7`  |      77286072      |        1428396       |  9680620 |       6883000       | 88577731 |       506       |
|  `ispd2019_test7_metal5` |      70848996      |        1535876       |  9943260 |       6686000       | 82169293 |       377       |
|  `ispd2019_test8`  |      119199593      |        1338449       |  7780220 |       6103000       | 128412302 |       365       |
|  `ispd2019_test8_metal5` |      116062781      |        1493314       |  8561400 |       6089000       | 126429212 |       588       |
|  `ispd2019_test9`  |      184246497      |        2181774       | 14765850 |       10847000      | 201270655 |       528       |
|  `ispd2019_test9_metal5` |      179242111      |        2323850       | 16020280 |       10948000      | 197937335 |       658       |

## 5. License

READ THIS LICENSE AGREEMENT CAREFULLY BEFORE USING THIS PRODUCT. BY USING THIS PRODUCT YOU INDICATE YOUR ACCEPTANCE OF THE TERMS OF THE FOLLOWING AGREEMENT. THESE TERMS APPLY TO YOU AND ANY SUBSEQUENT LICENSEE OF THIS PRODUCT.



License Agreement for CUGR



Copyright (c) 2020 by The Chinese University of Hong Kong



All rights reserved



CU-SD LICENSE (adapted from the original BSD license) Redistribution of the any code, with or without modification, are permitted provided that the conditions below are met.



Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.



Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.



Neither the name nor trademark of the copyright holder or the author may be used to endorse or promote products derived from this software without specific prior written permission.



Users are entirely responsible, to the exclusion of the author, for compliance with (a) regulations set by owners or administrators of employed equipment, (b) licensing terms of any other software, and (c) local, national, and international regulations regarding use, including those regarding import, export, and use of encryption software.



THIS FREE SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR ANY CONTRIBUTOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, EFFECTS OF UNAUTHORIZED OR MALICIOUS NETWORK ACCESS; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
