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
This version of code is consistent with the one submitted to contest. For the DAC paper version, please refer to branch [dac2020](https://github.com/cuhk-eda/cu-gr/tree/dac2020))

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

design                | WL           | #sv      | og WL     | og #v | ot WL     | ot #v | ww WL      | #short | short a | #min a | #prl | #eol | #cut | #adj | #cnr | #spc | #o | total score  | GR time (sec) | DR time (sec)
:--------------------:|-------------:|---------:|----------:|------:|----------:|------:|-----------:|-------:|--------:|-------:|-----:|-----:|-----:|-----:|-----:|-----:|---:|-------------:|--------------:|-------------:
`ispd18_test1`        | 429328.48    | 31714    | 1005.75   | 200   | 424.75    | 0     | 6089.48    | N/A    | 0.09    | 0      | N/A  | N/A  | N/A  | N/A  | N/A  | 0    | 0  | 285644.84    | 3.515         | 18.153
`ispd18_test2`        | 7801072.60   | 316056   | 21610.58  | 3575  | 6120.88   | 0     | 62172.03   | N/A    | 1.61    | 0      | N/A  | N/A  | N/A  | N/A  | N/A  | 23   | 0  | 4635372.59   | 26.996        | 140.967
`ispd18_test3`        | 8729246.35   | 315798   | 45635.68  | 3860  | 6641.75   | 0     | 62536.45   | N/A    | 50.92   | 0      | N/A  | N/A  | N/A  | N/A  | N/A  | 83   | 0  | 5178533.42   | 30.531        | 228.936
`ispd18_test4`        | 26352947.48  | 727520   | 150694.27 | 11485 | 18078.91  | 0     | 194751.24  | N/A    | 38.45   | 35     | N/A  | N/A  | N/A  | N/A  | N/A  | 645  | 0  | 15356708.25  | 71.847        | 529.511
`ispd18_test5`        | 27517414.44  | 927478   | 106216.66 | 6497  | 6119.49   | 3     | 51217.81   | N/A    | 83.85   | 86     | N/A  | N/A  | N/A  | N/A  | N/A  | 491  | 0  | 16111081.78  | 92.618        | 1512.641
`ispd18_test6`        | 35598120.15  | 1388115  | 43182.50  | 6085  | 12121.76  | 16    | 72510.56   | N/A    | 2.10    | 111    | N/A  | N/A  | N/A  | N/A  | N/A  | 604  | 0  | 21061695.01  | 116.304       | 900.239
`ispd18_test7`        | 64898420.23  | 2289266  | 90063.50  | 9879  | 26397.49  | 0     | 113797.44  | N/A    | 138.55  | 145    | N/A  | N/A  | N/A  | N/A  | N/A  | 77   | 0  | 37434958.00  | 278.040       | 2960.874
`ispd18_test8`        | 65495356.01  | 2346690  | 127072.50 | 11419 | 26987.33  | 0     | 118007.48  | N/A    | 130.94  | 158    | N/A  | N/A  | N/A  | N/A  | N/A  | 130  | 0  | 37920522.35  | 448.057       | 3629.062
`ispd18_test9`        | 54373386.51  | 2341199  | 107999.00 | 10522 | 21872.35  | 0     | 109948.95  | N/A    | 2.49    | 203    | N/A  | N/A  | N/A  | N/A  | N/A  | 104  | 0  | 32263242.38  | 272.847       | 2342.747
`ispd18_test10`       | 68133979.03  | 2497134  | 690396.81 | 27455 | 33427.33  | 0     | 147805.87  | N/A    | 397.93  | 214    | N/A  | N/A  | N/A  | N/A  | N/A  | 728  | 0  | 40613593.86  | 513.574       | 2718.931
`ispd18_test5_metal5` | 27954490.15  | 915376   | 66048.17  | 7324  | 6293.36   | 2     | 58772.50   | N/A    | 74.30   | 62     | N/A  | N/A  | N/A  | N/A  | N/A  | 386  | 0  | 16204442.07  | 100.219       | 1069.760
`ispd18_test8_metal5` | 64505547.00  | 2246918  | 159157.50 | 17006 | 25446.68  | 0     | 147881.05  | N/A    | 130.02  | 173    | N/A  | N/A  | N/A  | N/A  | N/A  | 86   | 0  | 37277888.94  | 269.102       | 2597.600
`ispd19_test1`        | 642439.07    | 38391    | 2789.77   | 228   | 756.91    | 730   | 8572.51    | 17     | 5.15    | 14     | 4    | 10   | 7    | 3    | 76   | N/A  | 0  | 555557.27    | 4.594         | 139.021
`ispd19_test2`        | 24884334.21  | 842650   | 89021.93  | 6407  | 22759.01  | 23386 | 141154.23  | 377    | 119.09  | 504    | 1098 | 705  | 123  | 99   | 6442 | N/A  | 0  | 20817663.27  | 93.918        | 1121.395
`ispd19_test3`        | 834362.47    | 66662    | 4721.51   | 493   | 1622.67   | 718   | 12569.62   | 79     | 20.55   | 30     | 131  | 69   | 21   | 37   | 87   | N/A  | 0  | 940417.71    | 4.691         | 63.506
`ispd19_test4`        | 29648152.82  | 907408   | 369575.32 | 24421 | 13929.36  | 3     | 137149.40  | 1181   | 979.33  | 79     | 298  | 56   | 0    | 0    | 0    | N/A  | 0  | 20288487.91  | 82.878        | 1358.690
`ispd19_test5`        | 4834862.62   | 137992   | 11694.63  | 1070  | 2076.10   | 20    | 18424.26   | 274    | 67.64   | 6      | 125  | 12   | 0    | 0    | 0    | N/A  | 0  | 3243965.57   | 10.578        | 172.097
`ispd19_test6`        | 66205463.27  | 2086448  | 198648.00 | 14208 | 26714.77  | 12211 | 270306.02  | 537    | 183.46  | 378    | 783  | 335  | 220  | 39   | 1078 | N/A  | 0  | 43733981.54  | 249.987       | 3027.696
`ispd19_test7`        | 122016807.24 | 4069417  | 613492.96 | 40758 | 42478.68  | 24475 | 728431.15  | 4175   | 1420.24 | 1749   | 7022 | 2125 | 797  | 116  | 2073 | N/A  | 0  | 88453085.56  | 891.409       | 8631.894
`ispd19_test8`        | 186800537.95 | 6449831  | 541570.41 | 52462 | 64551.72  | 36427 | 675714.15  | 2525   | 829.44  | 2314   | 2389 | 3109 | 1082 | 76   | 3312 | N/A  | 0  | 128356259.89 | 891.421       | 9159.133
`ispd19_test9`        | 282536337.69 | 10744582 | 836724.85 | 86887 | 107108.97 | 60780 | 1143828.03 | 5999   | 1838.70 | 4040   | 3807 | 5658 | 2837 | 137  | 5352 | N/A  | 0  | 201262621.22 | 921.212       | 12424.096
`ispd19_test10`       | 279098712.91 | 10336349 | 833445.67 | 62212 | 105852.35 | 60987 | 1630162.50 | 9258   | 2738.12 | 4179   | 4029 | 5844 | 2813 | 195  | 5063 | N/A  | 0  | 200594045.39 | 901.040       | 10797.617
`ispd19_test7_metal5` | 109209096.50 | 4061112  | 716525.97 | 40857 | 41417.00  | 24474 | 733310.43  | 4795   | 1719.52 | 1706   | 6374 | 2104 | 830  | 104  | 2358 | N/A  | 0  | 82380132.14  | 377.532       | 5645.986
`ispd19_test8_metal5` | 181012714.44 | 6389106  | 653341.93 | 52228 | 61350.54  | 36541 | 720527.34  | 3495   | 1449.80 | 2260   | 2520 | 3256 | 1224 | 111  | 2918 | N/A  | 0  | 126172994.75 | 911.743       | 17417.198
`ispd19_test9_metal5` | 273355893.14 | 10641041 | 916866.58 | 84418 | 101211.78 | 60894 | 1211065.49 | 7378   | 2766.56 | 4149   | 4085 | 6035 | 2943 | 200  | 4684 | N/A  | 0  | 197686238.02 | 1110.325      | 20038.183

(WL for "wirelength", sv for "single-cut via", og for "out-of-guide", ot for "off-track", ww for "wrong-way")

## 5. License

READ THIS LICENSE AGREEMENT CAREFULLY BEFORE USING THIS PRODUCT. BY USING THIS PRODUCT YOU INDICATE YOUR ACCEPTANCE OF THE TERMS OF THE FOLLOWING AGREEMENT. THESE TERMS APPLY TO YOU AND ANY SUBSEQUENT LICENSEE OF THIS PRODUCT.



License Agreement for CUGR



Copyright (c) 2019 by The Chinese University of Hong Kong



All rights reserved



CU-SD LICENSE (adapted from the original BSD license) Redistribution of the any code, with or without modification, are permitted provided that the conditions below are met.



Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.



Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.



Neither the name nor trademark of the copyright holder or the author may be used to endorse or promote products derived from this software without specific prior written permission.



Users are entirely responsible, to the exclusion of the author, for compliance with (a) regulations set by owners or administrators of employed equipment, (b) licensing terms of any other software, and (c) local, national, and international regulations regarding use, including those regarding import, export, and use of encryption software.



THIS FREE SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR ANY CONTRIBUTOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, EFFECTS OF UNAUTHORIZED OR MALICIOUS NETWORK ACCESS; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
