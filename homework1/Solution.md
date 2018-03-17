# Homework 1

Solution for homework1 by Dingfeng Liu.
## 1. Hello World

Run ``bazel run -c opt //homework1/helloworld:main`` and the results is
```
INFO: Analysed target //homework1/helloworld:main (0 packages loaded).
INFO: Found 1 target...
Target //homework1/helloworld:main up-to-date:
  bazel-bin/homework1/helloworld/main
INFO: Elapsed time: 0.322s, Critical Path: 0.03s
INFO: Build completed successfully, 1 total action

INFO: Running command line: bazel-bin/homework1/helloworld/main
Hello World.
0
1
2
3
4
5
6
7
8
9

```

## 2. Unit Test
Run ``bazel test -c opt //homework1/unittest:car_test``, the test will passed.
```
INFO: Analysed target //homework1/unittest:car_test (0 packages loaded).
INFO: Found 1 test target...
Target //homework1/unittest:car_test up-to-date:
  bazel-bin/homework1/unittest/car_test
INFO: Elapsed time: 1.160s, Critical Path: 0.42s
INFO: Build completed successfully, 6 total actions
//homework1/unittest:car_test                                            PASSED in 0.1s

Executed 1 out of 1 test: 1 test passes.
There were tests whose specified size is too big. Use the --test_verbose_timeout_warnings command line option to see which ones these are.
```

## 3. Protobuf
Run ``bazel test -c opt //homework1/protobuf:canvas_test``, the output is 
```
WARNING: /home/miu/.cache/bazel/_bazel_miu/7be534b8ec93e9ae38f7e77b58b30494/external/gflags/WORKSPACE:1: Workspace name in /home/miu/.cache/bazel/_bazel_miu/7be534b8ec93e9ae38f7e77b58b30494/external/gflags/WORKSPACE (@com_github_gflags_gflags) does not match the name given in the repository's definition (@gflags); this will cause a build error in future versions
INFO: Analysed target //homework1/protobuf:canvas_test (0 packages loaded).
INFO: Found 1 test target...
Target //homework1/protobuf:canvas_test up-to-date:
  bazel-bin/homework1/protobuf/canvas_test
INFO: Elapsed time: 2.845s, Critical Path: 1.08s
INFO: Build completed successfully, 20 total actions
//homework1/protobuf:canvas_test                                         PASSED in 0.2s

Executed 1 out of 1 test: 1 test passes.
There were tests whose specified size is too big. Use the --test_verbose_timeout_warnings command line option to see which ones these are.
```