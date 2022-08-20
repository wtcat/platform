For example(AM437X):

1> rtems build and install:
    ./waf configure --prefix=~/development/rtems/6 --rtems-config=config.am43.ini
    ./waf build
    ./waf install

2> rtems-libbsd build and install:
    ./waf configure --prefix=~/development/rtems/6 --rtems-bsps=arm/am437x_idk_evm --buildset=buildset/default.ini
    ./waf build
    ./waf install

3> Generate compile_commands.json
    ninja -C out -t compdb cxx cc > compile_commands.json
