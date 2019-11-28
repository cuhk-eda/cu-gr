CU-GR
======================================
CU-GR is a VLSI global routing tool developed by the research team supervised by Prof. Evangeline F. Y. Young in The Chinese University of Hong Kong (CUHK).
Different from previous global routers whose quality is usually measured by wirelength and resource overflow, 
CU-GR is a detailed routability-driven global router and its solution quality is solely determined by the final detailed routing results.
In particular, our global router adopts several efficient and effective methods to generate a set of connected rectangles to guide the detailed router:
* A sophisticated probability-based cost scheme
* An optimal 3D pattern routing technique that combines 2D pattern routing and layer assignment
* A multi-level maze routing utilizes two levels of routing
* A patching technique that adds useful route guides to further improve the detailed routability.
* ...

(CU-GR supports ICCAD'19 benchmarks ([v2](http://iccad-contest.org/2019/Problem_C/iccad19_benchmarks_v2.tar.gz), [hidden](http://iccad-contest.org/2019/Problem_C/iccad19_hidden_benchmarks.tar.gz)).
This version of code is consistent with the one submitted to contest.)

## 1. How to Build

**Step 1:** Download the source code. For example,
```bash
$ git clone https://github.com/cuhk-route/cu-gr
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
~~~
$ cd run
$ ./iccad19gr -lef ../toys/iccad2019c/ispd18_sample/ispd18_sample.input.lef -def ../toys/iccad2019c/ispd18_sample/ispd18_sample.input.def -output ispd18_sample.solution.guide -threads 8
~~~

#### Run with a Wrapping Script

Instead of running the binary directly, you may also use a wrapping script `run.py` to save typing and do more:
~~~
$ cd run
$ ./run.py 8s -p ../toys/
~~~

If Innovus® has been properly installed in your OS, an evaluation can be launched by
~~~
$ ./run.py 8s -s eval -p ../toys/
~~~
In the end, a result table will be printed in the terminal.

Furthermore, the solution can be visualized by
~~~
$ ./run.py 8s -s view -p ../toys/
~~~
which gives:

![ispd18_sample.solution.png](/toys/iccad2019c/ispd18_sample/ispd18_sample.solution.png)

The three steps, `route`, `eval` and `view` of `run.py` can also be invoked in a single line:
~~~
$ ./run.py 8s -s route eval view -p ../toys/
~~~
More usage about `run.py` can be known by the option `-h`.

### 2.2. Batch Test

The benchmarks can be downloaded from [the hompage of ISPD'18 Contest ](http://www.ispd.cc/contests/18/#benchmarks).
You may let `run.py` know the benchmark path by setting OS environmental variable `BENCHMARK_PATH` or specifying it under option `-p`.
Then,
```
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

|          Design          |      WL     |    #sv   |   og WL  | og #v |   ot WL  | ot #v |   ww WL  | #short | short a | #min a | #prl | #eol | #cut | #cnr | #spc | #o | total score | GR time (sec) | DR time (sec) |
|:------------------------:|:-----------:|:--------:|:--------:|:-----:|:--------:|:-----:|:--------:|:------:|:-------:|:------:|:----:|:----:|:----:|:----:|:----:|:--:|:-----------:|:-------------:|:-------------:|
| `ispd2018_test1`         |   429184.7  |   31734  |  994.55  |  201  |  421.05  |   0   |  6082.68 |   N/A  |    0    |    0   |  N/A |  N/A |  N/A |  N/A |   2  |  0 |   286549.1  |     3.298     |     17.333    |
| `ispd2018_test2`         |  7801281.98 |  315968  | 22672.45 |  3651 |  6053.18 |   0   | 61983.22 |   N/A  |   1.02  |    0   |  N/A |  N/A |  N/A |  N/A |  38  |  0 |   4643419   |     26.438    |    134.026    |
| `ispd2018_test3`         |  8730663.47 |  316059  |  46748.8 |  3925 |  6673.28 |   0   |  62309.8 |   N/A  |  52.72  |    0   |  N/A |  N/A |  N/A |  N/A |  80  |  0 |  5180128.72 |     30.468    |     221.33    |
| `ispd2018_test4`         | 26351761.14 |  727346  | 144860.2 | 11461 | 18045.55 |   0   | 192094.5 |   N/A  |  45.12  |   24   |  N/A |  N/A |  N/A |  N/A |  618 |  0 | 15331572.62 |     71.922    |    517.058    |
| `ispd2018_test5`         | 27518645.24 |  927739  | 95320.67 |  6484 |  5875.49 |   3   | 51152.33 |   N/A  |    95   |   68   |  N/A |  N/A |  N/A |  N/A |  474 |  0 | 16089195.85 |     82.873    |    1370.89    |
| `ispd2018_test6`         | 35595474.43 |  1388113 |  43793.5 |  6058 | 12267.86 |   16  | 72891.48 |   N/A  |   2.95  |   100  |  N/A |  N/A |  N/A |  N/A |  612 |  0 | 21060331.12 |    113.582    |    860.145    |
| `ispd2018_test7`         | 64914559.96 |  2289560 |  85394.5 | 10066 | 26655.64 |   0   | 113870.3 |   N/A  |  155.29 |   144  |  N/A |  N/A |  N/A |  N/A |  101 |  0 |  37459203.5 |    263.426    |    2287.396   |
| `ispd2018_test8`         | 65484439.43 |  2346183 |  119044  | 11411 | 26826.97 |   0   | 117916.8 |   N/A  |  137.89 |   158  |  N/A |  N/A |  N/A |  N/A |  129 |  0 |  37908814.8 |    259.904    |    2259.881   |
| `ispd2018_test9`         | 54365426.23 |  2341193 | 102045.5 | 10481 | 21877.33 |   0   | 110092.9 |   N/A  |   2.8   |   197  |  N/A |  N/A |  N/A |  N/A |  123 |  0 | 32260057.17 |    178.315    |    1492.528   |
| `ispd2018_test10`        | 68089179.61 |  2496469 |  700693  | 28370 | 31764.17 |   0   | 146273.6 |   N/A  |  377.51 |   265  |  N/A |  N/A |  N/A |  N/A |  701 |  0 | 40600501.49 |    349.399    |    2535.34    |
| `ispd2018_test5_metal5`  | 27958584.44 |  915422  | 58013.36 |  7261 |  6266.09 |   3   | 58536.28 |   N/A  |  82.44  |   46   |  N/A |  N/A |  N/A |  N/A |  418 |  0 |  16210302.7 |     80.325    |    1035.531   |
| `ispd2018_test8_metal5`  | 64502998.83 |  2245977 |  161645  | 17306 | 25646.86 |   0   | 148152.8 |   N/A  |  157.16 |   179  |  N/A |  N/A |  N/A |  N/A |  85  |  0 | 37293961.54 |    240.287    |    2340.557   |
| `ispd2018_test10_metal5` | 70733484.53 |  2433401 |  1121715 | 71510 | 31069.37 |   0   | 225196.3 |   N/A  | 7855.22 |   464  |  N/A |  N/A |  N/A |  N/A |  947 |  0 | 46300610.22 |    349.827    |    5110.996   |
| `ispd2019_test1`         |  641959.56  |   38511  |  3007.77 |  251  |   755.8  |  730  |  8615.49 |   18   |   6.8   |   12   |   7  |  10  |   5  |  69  |  N/A |  0 |  554905.94  |     4.261     |    127.176    |
| `ispd2019_test2`         | 24890607.98 |  842315  | 94244.26 |  6479 | 22850.23 | 23382 | 141236.3 |   393  |  125.07 |   561  | 1090 |  728 |  120 | 6445 |  N/A |  0 | 20871863.16 |     82.362    |    1032.044   |
| `ispd2019_test3`         |  833475.12  |   66603  |  5029.43 |  493  |  1635.09 |  716  | 12727.69 |   89   |  23.71  |   21   |  122 |  55  |  25  |  92  |  N/A |  0 |  937290.73  |     4.153     |     61.213    |
| `ispd2019_test4`         | 29667938.82 |  908961  |  372836  | 24608 | 13917.49 |   2   | 136106.7 |  1005  |  784.48 |   78   |  269 |  52  |   0  |   0  |  N/A |  0 | 20104565.48 |     78.067    |    1242.691   |
| `ispd2019_test5`         |  4838491.85 |  137993  | 10874.13 |  1057 |  2059.97 |   20  | 18428.66 |   290  |  72.24  |    6   |  140 |  12  |   0  |   0  |  N/A |  0 |  3262747.56 |     9.596     |    157.241    |
| `ispd2019_test6`         | 66207748.48 |  2087136 | 200662.5 | 14277 | 26733.17 | 12207 | 269974.8 |   571  |  249.34 |   422  |  865 |  367 |  192 | 1087 |  N/A |  0 | 43854576.07 |    226.403    |    2233.286   |
| `ispd2019_test7`         | 122031665.6 |  4069777 | 613477.4 | 40491 | 42521.87 | 24488 | 728772.7 |  4172  |  1454.6 |  1790  | 7085 | 2110 |  874 | 2115 |  N/A |  0 | 88577730.84 |    563.807    |    5154.622   |
| `ispd2019_test8`         |  186799131  |  6450139 | 543674.2 | 52484 |  64283.1 | 36454 | 674924.3 |  2592  |   955   |  2262  | 2376 | 3057 | 1177 | 3234 |  N/A |  0 |  128412302  |    562.334    |    8340.791   |
| `ispd2019_test9`         | 282504596.6 | 10741469 | 841551.2 | 86682 | 106820.7 | 60793 |  1142884 |  5986  | 1849.32 |  3941  | 3972 | 5729 | 2814 | 5275 |  N/A |  0 | 201270654.9 |    895.783    |    12516.36   |
| `ispd2019_test10`        | 279097326.1 | 10334152 | 830984.2 | 62107 | 105927.7 | 61004 |  1632195 |  9379  |   2764  |  4209  | 3927 | 6052 | 2824 | 5124 |  N/A |  0 | 200755027.5 |    874.629    |    10542.66   |
| `ispd2019_test7_metal5`  | 109209599.4 |  4060091 | 706846.1 | 40948 | 41447.75 | 24446 | 735795.2 |  4530  | 1673.74 |  1757  | 6240 | 2124 |  892 | 2255 |  N/A |  0 | 82169292.77 |    335.358    |    5244.894   |
| `ispd2019_test8_metal5`  | 181087032.2 |  6391104 | 628169.1 | 52303 | 61235.21 | 36542 |  719081  |  3748  | 1565.13 |  2285  | 2603 | 3294 | 1201 | 2934 |  N/A |  0 | 126429212.2 |    499.372    |    9060.015   |
| `ispd2019_test9_metal5`  |  273351637  | 10637815 | 921640.6 | 84664 | 101176.9 | 60896 |  1216755 |  7513  | 2817.43 |  4263  | 4171 | 6152 | 2896 | 4767 |  N/A |  0 | 197937335.2 |    592.942    |    13303.77   |

(WL for "wirelength", sv for "single-cut via", og for "out-of-guide", ot for "off-track", ww for "wrong-way")

## 5. License

READ THIS LICENSE AGREEMENT CAREFULLY BEFORE USING THIS PRODUCT. BY USING THIS PRODUCT YOU INDICATE YOUR ACCEPTANCE OF THE TERMS OF THE FOLLOWING AGREEMENT. THESE TERMS APPLY TO YOU AND ANY SUBSEQUENT LICENSEE OF THIS PRODUCT.



License Agreement for CU-GR



Copyright (c) 2019 by The Chinese University of Hong Kong



All rights reserved



CU-SD LICENSE (adapted from the original BSD license) Redistribution of the any code, with or without modification, are permitted provided that the conditions below are met. 



Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.



Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.



Neither the name nor trademark of the copyright holder or the author may be used to endorse or promote products derived from this software without specific prior written permission.



Users are entirely responsible, to the exclusion of the author, for compliance with (a) regulations set by owners or administrators of employed equipment, (b) licensing terms of any other software, and (c) local, national, and international regulations regarding use, including those regarding import, export, and use of encryption software.



THIS FREE SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR ANY CONTRIBUTOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, EFFECTS OF UNAUTHORIZED OR MALICIOUS NETWORK ACCESS; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
